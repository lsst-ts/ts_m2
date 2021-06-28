# This file is part of ts_m2.
#
# Developed for the Vera Rubin Observatory Telescope and Site Systems.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import logging
import asyncio
import json
import copy

from lsst.ts import salobj
from lsst.ts import tcpip

from . import TopicType

__all__ = ["TcpClient"]


class TcpClient:
    """TCP/IP client.

    Parameters
    ----------
    host : `str`
        Host address.
    port : `int`
        Port to connect.
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    max_n_bytes : `int`, optional
        Read up to n bytes. (the default is 1000)
    maxsize_queue : `int`, optional
        Maximum size of queue. (the default is 1000)

    Attributes
    ----------
    host : `str`
        Host address.
    port : `int`
        Port to connect.
    log : `logging.Logger`
        A logger.
    reader : `asyncio.StreamReader` or None
        Reader of socker.
    writer : `asyncio.StreamWriter` or None
        Writer of the socket.
    max_n_bytes : `int`
        Read up to n bytes.
    queue : `asyncio.Queue`
        Queue of the message.
    """

    def __init__(self, host, port, log=None, max_n_bytes=1000, maxsize_queue=1000):

        # Connection information
        self.host = host
        self.port = int(port)

        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.reader = None
        self.writer = None

        self.max_n_bytes = int(max_n_bytes)

        # Unique ID
        self._uniq_id = salobj.index_generator()

        self.queue = asyncio.Queue(maxsize=int(maxsize_queue))

        # Monitor loop task (asyncio.Task)
        self._monitor_loop_task = None

        # Monitor loop task is done or not (asyncio.Future)
        self._monitor_loop_task_done = None

    async def connect(self, connect_retry_interval=1.0):
        """Connect to the server.

        Parameters
        ----------
        connect_retry_interval : `float`, optional
            How long to wait before trying to reconnect when connection fails.
            (default is 1.0)

        Notes
        -----
        This will wait forever for a connection.
        """

        self.log.info("Open the connection.")

        while not self.is_connected():

            try:
                self.reader, self.writer = await asyncio.open_connection(
                    host=self.host, port=self.port
                )

            except ConnectionRefusedError:
                await asyncio.sleep(connect_retry_interval)

        self.log.info("Connection is on.")

    def is_connected(self):
        """Determines if the client is connected to the server.

        Returns
        -------
        bool
            True if is connected. Else, False.
        """

        return not (
            self.reader is None
            or self.writer is None
            or self.reader.at_eof()
            or self.writer.is_closing()
        )

    async def write(self, topic_type, topic_name, topic_details=None, comp_name=None):
        """Writes message to the server.

        Parameters
        ----------
        topic_type : `TopicType`
            Topic type.
        topic_name : `str`
            Topic name.
        topic_details : `dict` or None, optional
            Topic details. (the default is None)
        comp_name : `str` or None, optional
            Specific component name used in the event or telemetry. (the
            default is None)

        Raises
        ------
        RuntimeError
            When there is no TCP/IP connection.
        ValueError
            When the topic type is not supported.
        """

        if not self.is_connected():
            raise RuntimeError("Client not connected with tcp/ip server.")

        if topic_details is None:
            topic_details_with_header = dict()
        else:
            topic_details_with_header = copy.copy(topic_details)

        if topic_type == TopicType.Command:
            self._add_cmd_header(topic_name, topic_details_with_header)
        elif topic_type == TopicType.Event:
            self._add_evt_header(
                topic_name, topic_details_with_header, comp_name=comp_name
            )
        elif topic_type == TopicType.Telemetry:
            self._add_tel_header(
                topic_name, topic_details_with_header, comp_name=comp_name
            )
        else:
            raise ValueError(f"The topic type: {topic_type} is not supported.")

        await self._write_msg_to_socket(topic_details_with_header)

    def _add_cmd_header(self, topic_name, topic_details):
        """Add the command header.

        Parameters
        ----------
        topic_name : `str`
            Topic name.
        topic_details : `dict`
            Topic details.

        Raises
        ------
        ValueError
            If 'cmdName' is in the topic details already.
        """

        if "cmdName" in topic_details.keys():
            raise ValueError("The 'cmdName' is in the topic details already.")

        topic_details["cmdName"] = topic_name
        topic_details["cmdId"] = next(self._uniq_id)

    def _add_evt_header(self, topic_name, topic_details, comp_name=None):
        """Add the event header.

        Parameters
        ----------
        topic_name : `str`
            Topic name.
        topic_details : `dict`
            Topic details.
        comp_name : `str` or None, optional
            Specific component name. (the default is None)

        Raises
        ------
        ValueError
            If 'evtName' is in the topic details already.
        """

        if "evtName" in topic_details.keys():
            raise ValueError("The 'evtName' is in the topic details already.")

        topic_details["evtName"] = topic_name

        if comp_name is not None:
            topic_details["compName"] = comp_name

    def _add_tel_header(self, topic_name, topic_details, comp_name=None):
        """Add the telemetry header.

        Parameters
        ----------
        topic_name : `str`
            Topic name.
        topic_details : `dict`
            Topic details.
        comp_name : `str` or None, optional
            Specific component name. (the default is None)

        Raises
        ------
        ValueError
            If 'telName' is in the topic details already.
        """

        if "telName" in topic_details.keys():
            raise ValueError("The 'telName' is in the topic details already.")

        topic_details["telName"] = topic_name

        if comp_name is not None:
            topic_details["compName"] = comp_name

    async def _write_msg_to_socket(self, input_msg):
        """Write the message to socket.

        Parameters
        ----------
        input_msg : `dict`
            Input message.
        """

        # Transfer to json string and do the encode
        msg = json.dumps(input_msg, indent=4).encode() + tcpip.TERMINATOR

        self.writer.write(msg)
        await self.writer.drain()

    async def run_monitor_loop(self):
        """Run the monitor loop. The received message from server will be put
        into self.queue.
        """

        # Create the task's future
        loop = asyncio.get_running_loop()
        self._monitor_loop_task_done = loop.create_future()

        # Create the task to monitor the incoming message from server
        self._monitor_loop_task = asyncio.create_task(self._monitor_msg())

        await self._monitor_loop_task_done

    async def _monitor_msg(self):
        """Monitor the message."""

        self.log.info("Begin to monitor the incoming message.")

        while self.is_connected():
            await self.put_read_msg_to_queue()

        self._monitor_loop_task_done.set_result("Monitor is done.")

        self.log.info("Stop to monitor the incoming message.")

    async def put_read_msg_to_queue(self):
        """Put the read message to self.queue."""

        try:
            data = await self.reader.read(n=self.max_n_bytes)

            if data is not None:
                dataDecode = data.decode()
                message = json.loads(dataDecode)
                self.queue.put_nowait(message)

                self._check_queue_size()

        except json.JSONDecodeError:
            self.log.debug(f"Can not decode the message: {dataDecode}.")

    def _check_queue_size(self):
        """Check the size of queue and log the information if needed."""

        queue_size = self.queue.qsize()
        maxsize = self.queue.maxsize
        if queue_size > maxsize // 2:
            self.log.info(f"Size of queue is: {queue_size}/{maxsize}.")

    async def close(self):
        """Kill the tasks and close the connection."""

        self.log.info("Close the connection.")

        # Cancel the task
        if self._monitor_loop_task is not None:
            self.log.debug("Cancel the monitor loop task.")
            self._monitor_loop_task.cancel()

            if not self._monitor_loop_task_done.done():
                self._monitor_loop_task_done.cancel()

        self._monitor_loop_task = None
        self._monitor_loop_task_done = None

        # Write the EOF
        if self.writer.can_write_eof():
            self.writer.write_eof()

        # Close the writer
        await tcpip.close_stream_writer(self.writer)

        # Set the reader and writer to be None
        self.reader = None
        self.writer = None
