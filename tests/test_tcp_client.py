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


import unittest
import contextlib
import asyncio
import socket
import logging
import json

from lsst.ts import tcpip
from lsst.ts.m2 import MsgType, TcpClient, write_json_packet


# Read timeout in second
READ_TIMEOUT = 1


class TestTcpClient(unittest.IsolatedAsyncioTestCase):
    """Test the TcpClient class."""

    @classmethod
    def setUpClass(cls):
        cls.host = tcpip.LOCAL_HOST
        cls.log = logging.getLogger()

    @contextlib.asynccontextmanager
    async def make_server(self):
        """Instantiate a TCP/IP server for the test."""

        server = tcpip.OneClientServer(
            host=self.host,
            port=0,
            name="test",
            log=self.log,
            family=socket.AF_UNSPEC,
            connect_callback=None,
        )
        await server.start_task
        try:
            yield server
        finally:
            await server.close()

    @contextlib.asynccontextmanager
    async def make_client(self, server):
        """Make the client and do the connection.

        Parameters
        ----------
        server : `lsst.ts.tcpip.OneClientServer`
            TCP/IP server.
        """

        client = TcpClient(server.host, server.port, log=self.log, maxsize_queue=8)
        await client.connect()

        try:
            yield client
        finally:
            await client.close()

    async def test_close(self):
        client = TcpClient(tcpip.LOCAL_HOST, 0)
        await client.close()

    async def test_is_connected(self):

        async with self.make_server() as server, self.make_client(server) as client:

            self.assertTrue(client.is_connected())

            await server.close_client()

            # Need to add a small time to close the client's connection totally
            await asyncio.sleep(0.1)

            self.assertFalse(client.is_connected())

    async def test_write_no_connection(self):

        tcp_client = TcpClient(self.host, 0, log=self.log)

        with self.assertRaises(RuntimeError):
            await tcp_client.write(MsgType.Event, "inPosition")

    async def test_write_cmd(self):
        async with self.make_server() as server, self.make_client(server) as client:

            self.assertEqual(client.last_sequence_id, -1)

            msg_name = "move"
            msg_details = {"x": 1, "y": 2, "z": 3}
            await client.write(MsgType.Command, msg_name, msg_details=msg_details)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "cmd_" + msg_name)
            self.assertEqual(message["sequence_id"], 1)
            self.assertEqual(message["x"], msg_details["x"])
            self.assertEqual(message["y"], msg_details["y"])
            self.assertEqual(message["z"], msg_details["z"])

            self.assertEqual(client.last_sequence_id, 1)

    async def _read_msg_in_server(self, server, timeout):
        """Read the received message in server.

        Parameters
        ----------
        server : `lsst.ts.tcpip.OneClientServer`
            TCP/IP server.
        timeout : `float`
            Timeout to read the message in second.
        """

        data_encode = await asyncio.wait_for(server.reader.read(n=1000), timeout)
        data = data_encode.decode()

        return json.loads(data)

    async def test_write_cmd_multiple(self):
        async with self.make_server() as server, self.make_client(server) as client:

            msg_name = "move"
            msg_details = {"x": 1, "y": 2, "z": 3}
            for count in range(3):
                await client.write(MsgType.Command, msg_name, msg_details=msg_details)

                message = await self._read_msg_in_server(server, READ_TIMEOUT)

                sequence_id_expected = count + 1
                self.assertEqual(message["sequence_id"], sequence_id_expected)
                self.assertEqual(client.last_sequence_id, sequence_id_expected)

    async def test_write_cmd_no_details(self):
        async with self.make_server() as server, self.make_client(server) as client:

            msg_name = "enable"
            await client.write(MsgType.Command, msg_name)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "cmd_" + msg_name)
            self.assertEqual(message["sequence_id"], 1)

    async def test_write_id_error(self):
        async with self.make_server() as server, self.make_client(server) as client:

            msg_name = "move"
            msg_details = {"id": "cmd_name"}
            with self.assertRaises(ValueError):
                await client.write(MsgType.Command, msg_name, msg_details=msg_details)

    async def test_write_evt(self):
        async with self.make_server() as server, self.make_client(server) as client:

            msg_name = "inPosition"
            msg_details = {"status": True}
            comp_name = "MTMount"
            await client.write(
                MsgType.Event, msg_name, msg_details=msg_details, comp_name=comp_name
            )

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "evt_" + msg_name)
            self.assertEqual(message["status"], msg_details["status"])
            self.assertEqual(message["compName"], comp_name)

    async def test_write_evt_no_details(self):
        async with self.make_server() as server, self.make_client(server) as client:

            msg_name = "inPosition"
            comp_name = "MTMount"
            await client.write(MsgType.Event, msg_name, comp_name=comp_name)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "evt_" + msg_name)
            self.assertEqual(message["compName"], comp_name)

    async def test_write_tel(self):
        async with self.make_server() as server, self.make_client(server) as client:

            msg_name = "elevation"
            msg_details = {"measured": 1.1}
            comp_name = "MTMount"
            await client.write(
                MsgType.Telemetry,
                msg_name,
                msg_details=msg_details,
                comp_name=comp_name,
            )

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "tel_" + msg_name)
            self.assertEqual(message["measured"], msg_details["measured"])
            self.assertEqual(message["compName"], comp_name)

    async def test_write_tel_no_details(self):
        async with self.make_server() as server, self.make_client(server) as client:

            msg_name = "inPosition"
            comp_name = "MTMount"
            await client.write(MsgType.Telemetry, msg_name, comp_name=comp_name)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["id"], "tel_" + msg_name)
            self.assertEqual(message["compName"], comp_name)

    async def test_put_read_msg_to_queue(self):
        async with self.make_server() as server, self.make_client(server) as client:

            input_msg = {"val": 1}
            await write_json_packet(server.writer, input_msg)

            # Sleep a short time to let the monitor loop have a chance to run
            await asyncio.sleep(0.01)

            self.assertEqual(client.queue.qsize(), 1)

            data = client.queue.get_nowait()
            self.assertEqual(data["val"], input_msg["val"])

    async def test_run_monitor_loop(self):
        async with self.make_server() as server, self.make_client(server) as client:

            input_msg = {"val": 1}
            await self._write_msg_continuously_at_specific_time(
                2, server, input_msg, 5, 1
            )

            self.assertEqual(client.queue.qsize(), 5)

    async def _write_msg_continuously_at_specific_time(
        self, time, server, input_msg, duration, frequency
    ):
        """Write the message continuously in server at the specific time. This
        is to let the client has the enough time to start running the monitor
        loop.

        Parameters
        ----------
        time : `float`
            Time to wait to write the message in second.
        server : `lsst.ts.tcpip.OneClientServer`
            TCP/IP server.
        input_msg : `dict`
            Input message.
        duration : `float`
            Duration of the writing of message in second.
        frequency : `float`
            Frequency of the writing of message in Hz.
        """

        await asyncio.sleep(time)

        count_total = int(duration * frequency)
        sleep_time = 1 / frequency
        for count in range(count_total):
            await write_json_packet(server.writer, input_msg)
            await asyncio.sleep(sleep_time)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
