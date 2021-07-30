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

from . import MsgType, write_json_packet

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
    queue : `asyncio.Queue`
        Queue of the message.
    """

    TIMEOUT_IN_SECOND = 0.05

    def __init__(self, host, port, log=None, maxsize_queue=1000):

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

        # Unique ID
        self._uniq_id = salobj.index_generator()

        self.queue = asyncio.Queue(maxsize=int(maxsize_queue))

        # Monitor loop task (asyncio.Future)
        self._monitor_loop_task = salobj.make_done_future()

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

        # Create the task to monitor the incoming message from server
        self._monitor_loop_task = asyncio.create_task(self._monitor_msg())

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

    async def _monitor_msg(self):
        """Monitor the message."""

        self.log.info("Begin to monitor the incoming message.")

        try:
            while self.is_connected():
                await self._put_read_msg_to_queue()

        except ConnectionError:
            self.log.info("Reader disconnected; closing client")
            await self._basic_close()

        except asyncio.IncompleteReadError:
            self.log.info("EOF is reached.")

        self.log.info("Stop to monitor the incoming message.")

    async def _put_read_msg_to_queue(self):
        """Put the read message to self.queue."""

        try:
            data = await asyncio.wait_for(
                self.reader.readuntil(separator=tcpip.TERMINATOR),
                self.TIMEOUT_IN_SECOND,
            )

            if data is not None:
                data_decode = data.decode()
                msg = json.loads(data_decode)
                self.queue.put_nowait(msg)

                self._check_queue_size()

        except asyncio.TimeoutError:
            await asyncio.sleep(self.TIMEOUT_IN_SECOND)

        except json.JSONDecodeError:
            self.log.debug(f"Can not decode the message: {data_decode}.")

        except asyncio.IncompleteReadError:
            raise

    def _check_queue_size(self):
        """Check the size of queue and log the information if needed."""

        queue_size = self.queue.qsize()
        maxsize = self.queue.maxsize
        if queue_size > maxsize // 2:
            self.log.info(f"Size of queue is: {queue_size}/{maxsize}.")

    async def _basic_close(self):
        """Cancel the monitor loop and close the connection."""

        # Cancel the task
        self._monitor_loop_task.cancel()

        if self.writer is not None:
            await tcpip.close_stream_writer(self.writer)
            self.writer = None

    async def write(self, msg_type, msg_name, msg_details=None, comp_name=None):
        """Writes message to the server.

        Parameters
        ----------
        msg_type : `MsgType`
            Message type.
        msg_name : `str`
            Message name.
        msg_details : `dict` or None, optional
            Message details. (the default is None)
        comp_name : `str` or None, optional
            Specific component name used in the event or telemetry. (the
            default is None)

        Raises
        ------
        RuntimeError
            When there is no TCP/IP connection.
        ValueError
            If 'id' is in the message details already.
        ValueError
            When the message type is not supported.
        """

        if not self.is_connected():
            raise RuntimeError("Client not connected with tcp/ip server.")

        if msg_details is None:
            msg_details_with_header = dict()
        else:
            msg_details_with_header = copy.copy(msg_details)

        if "id" in msg_details_with_header.keys():
            raise ValueError("The 'id' is in the message details already.")

        if msg_type == MsgType.Command:
            msg_details_with_header = self._add_cmd_header(
                msg_name, msg_details_with_header
            )
        elif msg_type == MsgType.Event:
            msg_details_with_header = self._add_evt_header(
                msg_name, msg_details_with_header, comp_name=comp_name
            )
        elif msg_type == MsgType.Telemetry:
            msg_details_with_header = self._add_tel_header(
                msg_name, msg_details_with_header, comp_name=comp_name
            )
        else:
            raise ValueError(f"The message type: {msg_type} is not supported.")

        await write_json_packet(self.writer, msg_details_with_header)

    def _add_cmd_header(self, msg_name, msg_details):
        """Add the command header.

        Note: This method will modify the input: msg_details.

        Parameters
        ----------
        msg_name : `str`
            Message name.
        msg_details : `dict`
            Message details.

        Returns
        -------
        msg_details : dict
            Message details with the header.
        """

        msg_details["id"] = "cmd_" + msg_name
        msg_details["sequence_id"] = next(self._uniq_id)

        return msg_details

    def _add_evt_header(self, msg_name, msg_details, comp_name=None):
        """Add the event header.

        Note: This method will modify the input: msg_details.

        Parameters
        ----------
        msg_name : `str`
            Message name.
        msg_details : `dict`
            Message details.
        comp_name : `str` or None, optional
            Specific component name. (the default is None)

        Returns
        -------
        msg_details : dict
            Message details with the header.
        """

        msg_details["id"] = "evt_" + msg_name

        if comp_name is not None:
            msg_details["compName"] = comp_name

        return msg_details

    def _add_tel_header(self, msg_name, msg_details, comp_name=None):
        """Add the telemetry header.

        Note: This method will modify the input: msg_details.

        Parameters
        ----------
        msg_name : `str`
            Message name.
        msg_details : `dict`
            Message details.
        comp_name : `str` or None, optional
            Specific component name. (the default is None)

        Returns
        -------
        msg_details : dict
            Message details with the header.
        """

        msg_details["id"] = "tel_" + msg_name

        if comp_name is not None:
            msg_details["compName"] = comp_name

        return msg_details

    async def close(self):
        """Cancel the task and close the connection.

        Note: this function is safe to call even though there is no connection.
        """

        self.log.info("Close the connection.")

        # Write the EOF and close
        if (self.writer is not None) and self.writer.can_write_eof():
            self.writer.write_eof()

        await self._basic_close()

        # Set the reader and writer to be None
        self.reader = None
        self.writer = None
