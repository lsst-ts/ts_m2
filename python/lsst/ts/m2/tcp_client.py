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
from lsst.ts.utils import make_done_future

from . import MsgType, write_json_packet, check_queue_size

__all__ = ["TcpClient"]


class TcpClient:
    """TCP/IP client.

    Parameters
    ----------
    host : `str`
        Host address.
    port : `int`
        Port to connect.
    timeout_in_second : `float`, optional
        Read timeout in second. (the default is 0.05)
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    sequence_generator : `generator` or `None`, optional
        Sequence generator. (the default is None)
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
    timeout_in_second : `float`
        Read timeout in second.
    last_sequence_id : `int`
        Last sequence ID of command.
    queue : `asyncio.Queue`
        Queue of the message.
    """

    def __init__(
        self,
        host,
        port,
        timeout_in_second=0.05,
        log=None,
        sequence_generator=None,
        maxsize_queue=1000,
    ):

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

        self.timeout_in_second = timeout_in_second

        # Sequence ID generator
        if sequence_generator is None:
            self._sequence_id_generator = salobj.index_generator()
        else:
            self._sequence_id_generator = sequence_generator
        self.last_sequence_id = -1

        self.queue = asyncio.Queue(maxsize=int(maxsize_queue))

        # Monitor loop task (asyncio.Future)
        self._monitor_loop_task = make_done_future()

    async def connect(self, connect_retry_interval=1.0, timeout=10.0):
        """Connect to the server.

        Parameters
        ----------
        connect_retry_interval : `float`, optional
            How long to wait before trying to reconnect when connection fails.
            (default is 1.0)
        timeout : `float`, optional
            Timeout in second. This value should be larger than the
            connect_retry_interval. (default is 10.0)

        Raises
        ------
        asyncio.TimeoutError
            Connection timeout.
        """

        self.log.info("Try to open the connection.")

        retry_times_max = timeout // connect_retry_interval
        retry_times = 0
        while not self.is_connected():

            try:
                self.reader, self.writer = await asyncio.open_connection(
                    host=self.host, port=self.port
                )

            except ConnectionRefusedError:
                await asyncio.sleep(connect_retry_interval)

                retry_times += 1
                if retry_times >= retry_times_max:
                    raise asyncio.TimeoutError("Connection timeout.")

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
            self.log.exception("EOF is reached.")

        self.log.info("Stop to monitor the incoming message.")

    async def _put_read_msg_to_queue(self):
        """Put the read message to self.queue."""

        try:
            data = await asyncio.wait_for(
                self.reader.readuntil(separator=tcpip.TERMINATOR),
                self.timeout_in_second,
            )

            if data is not None:
                data_decode = data.decode()
                msg = json.loads(data_decode)
                self.queue.put_nowait(msg)

                check_queue_size(self.queue, self.log)

        except asyncio.TimeoutError:
            await asyncio.sleep(self.timeout_in_second)

        except json.JSONDecodeError:
            self.log.debug(f"Can not decode the message: {data_decode}.")

        except asyncio.QueueFull:
            self.log.exception("Internal queue is full.")

        except asyncio.IncompleteReadError:
            raise

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

        self.last_sequence_id = next(self._sequence_id_generator)
        msg_details["sequence_id"] = self.last_sequence_id

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
