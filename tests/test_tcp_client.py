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

from lsst.ts.m2 import TopicType, TcpClient


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
            await tcp_client.write(TopicType.Event, "inPosition")

    async def test_write_cmd(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "move"
            topic_details = {"x": 1, "y": 2, "z": 3}
            await client.write(TopicType.Command, topic, topic_details=topic_details)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["cmdName"], topic)
            self.assertEqual(message["cmdId"], 1)
            self.assertEqual(message["x"], topic_details["x"])
            self.assertEqual(message["y"], topic_details["y"])
            self.assertEqual(message["z"], topic_details["z"])

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

            topic = "move"
            topic_details = {"x": 1, "y": 2, "z": 3}
            for count in range(3):
                await client.write(
                    TopicType.Command, topic, topic_details=topic_details
                )

                message = await self._read_msg_in_server(server, READ_TIMEOUT)
                self.assertEqual(message["cmdId"], count + 1)

    async def test_write_cmd_no_details(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "enable"
            await client.write(TopicType.Command, topic)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["cmdName"], topic)
            self.assertEqual(message["cmdId"], 1)

    async def test_write_cmd_error(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "move"
            topic_details = {"cmdName": "name"}
            with self.assertRaises(ValueError):
                await client.write(
                    TopicType.Command, topic, topic_details=topic_details
                )

    async def test_write_evt(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "inPosition"
            topic_details = {"status": True}
            comp_name = "MTMount"
            await client.write(
                TopicType.Event, topic, topic_details=topic_details, comp_name=comp_name
            )

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["evtName"], topic)
            self.assertEqual(message["status"], topic_details["status"])
            self.assertEqual(message["compName"], comp_name)

    async def test_write_evt_no_details(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "inPosition"
            comp_name = "MTMount"
            await client.write(TopicType.Event, topic, comp_name=comp_name)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["evtName"], topic)
            self.assertEqual(message["compName"], comp_name)

    async def test_write_evt_error(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "inPosition"
            topic_details = {"evtName": topic}
            comp_name = "MTMount"
            with self.assertRaises(ValueError):
                await client.write(
                    TopicType.Event,
                    topic,
                    topic_details=topic_details,
                    comp_name=comp_name,
                )

    async def test_write_tel(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "elevation"
            topic_details = {"measured": 1.1}
            comp_name = "MTMount"
            await client.write(
                TopicType.Telemetry,
                topic,
                topic_details=topic_details,
                comp_name=comp_name,
            )

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["telName"], topic)
            self.assertEqual(message["measured"], topic_details["measured"])
            self.assertEqual(message["compName"], comp_name)

    async def test_write_tel_no_details(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "inPosition"
            comp_name = "MTMount"
            await client.write(TopicType.Telemetry, topic, comp_name=comp_name)

            message = await self._read_msg_in_server(server, READ_TIMEOUT)

            self.assertEqual(message["telName"], topic)
            self.assertEqual(message["compName"], comp_name)

    async def test_write_tel_error(self):
        async with self.make_server() as server, self.make_client(server) as client:

            topic = "inPosition"
            topic_details = {"telName": topic}
            comp_name = "MTMount"
            with self.assertRaises(ValueError):
                await client.write(
                    TopicType.Telemetry,
                    topic,
                    topic_details=topic_details,
                    comp_name=comp_name,
                )

    async def test_put_read_msg_to_queue(self):
        async with self.make_server() as server, self.make_client(server) as client:

            input_msg = {"val": 1}
            await self._write_msg_in_server(server, input_msg)

            await client.put_read_msg_to_queue()

            self.assertEqual(client.queue.qsize(), 1)

            data = client.queue.get_nowait()
            self.assertEqual(data["val"], input_msg["val"])

    async def _write_msg_in_server(self, server, input_msg):
        """Write the message in server.

        Parameters
        ----------
        server : `lsst.ts.tcpip.OneClientServer`
            TCP/IP server.
        input_msg : `dict`
            Input message.
        """

        msg = json.dumps(input_msg, indent=4).encode() + tcpip.TERMINATOR

        server.writer.write(msg)
        await server.writer.drain()

    async def test_run_monitor_loop(self):
        async with self.make_server() as server, self.make_client(server) as client:

            input_msg = {"val": 1}
            await asyncio.gather(
                client.run_monitor_loop(),
                self._write_msg_continuously_at_specific_time(
                    2, server, input_msg, 5, 1
                ),
                self._close_client_at_specific_time(10, server),
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
            await self._write_msg_in_server(server, input_msg)
            await asyncio.sleep(sleep_time)

    async def _close_client_at_specific_time(self, time, server):
        """Close the connection with client at the specific time.

        Parameters
        ----------
        time : `float`
            Time to close the connection with client in second.
        server : `lsst.ts.tcpip.OneClientServer`
            TCP/IP server.
        """

        await asyncio.sleep(time)

        await server.close_client()

        # Need to add a small time to close the connection totally
        await asyncio.sleep(0.1)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
