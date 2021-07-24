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
import logging

from lsst.ts import tcpip
from lsst.ts.idl.enums import MTM2

from lsst.ts.m2 import MsgType, DetailedState, TcpClient, MockServer


class TestMockServer(unittest.IsolatedAsyncioTestCase):
    """Test the MockServer class."""

    @classmethod
    def setUpClass(cls):
        cls.host = tcpip.LOCAL_HOST
        cls.log = logging.getLogger()
        cls.maxsize_queue = 1000

    @contextlib.asynccontextmanager
    async def make_server(self):
        """Instantiate the mock server of M2 for the test."""

        server = MockServer(self.host, port_command=0, port_telemetry=0, log=self.log)
        await server.start()

        try:
            yield server
        finally:
            await server.close()

        self.assertFalse(server._welcome_message_sent)

        self.assertIsNone(server._message_event)
        self.assertIsNone(server._message_telemetry)

        self.assertFalse(server._command.system_enabled)

    @contextlib.asynccontextmanager
    async def make_clients(self, server):
        """Make two TCP/IP clients that talk to the server and wait for it to
        connect.

        Returns (client_cmd, client_tel).
        """

        client_cmd = TcpClient(
            server.server_command.host,
            server.server_command.port,
            log=self.log,
            maxsize_queue=self.maxsize_queue,
        )
        client_tel = TcpClient(
            server.server_telemetry.host,
            server.server_telemetry.port,
            log=self.log,
            maxsize_queue=self.maxsize_queue,
        )

        await asyncio.gather(client_cmd.connect(), client_tel.connect())

        try:
            yield (client_cmd, client_tel)
        finally:
            await client_cmd.close()
            await client_tel.close()

    async def test_are_servers_connected(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            self.assertTrue(server.are_servers_connected())

            # Check the one-time message
            await asyncio.sleep(0.5)
            self.assertEqual(client_cmd.queue.qsize(), 8)

            # Check the TCP/IP connection
            msg_tcpip = client_cmd.queue.get_nowait()
            self.assertEqual(msg_tcpip["id"], "tcpIpConnected")
            self.assertTrue(msg_tcpip["isConnected"])

            # Check the commandable by DDS
            msg_commandable_by_dds = client_cmd.queue.get_nowait()
            self.assertEqual(msg_commandable_by_dds["id"], "commandableByDDS")
            self.assertTrue(msg_commandable_by_dds["state"])

            # Check the hardpoint list
            msg_hardpoints = client_cmd.queue.get_nowait()
            self.assertEqual(msg_hardpoints["id"], "hardpointList")
            self.assertEqual(msg_hardpoints["actuators"], [6, 16, 26, 74, 76, 78])

            # Check the interlock
            msg_interlock = client_cmd.queue.get_nowait()
            self.assertEqual(msg_interlock["id"], "interlock")
            self.assertFalse(msg_interlock["state"])

            # Check the inclination telemetry source
            msg_tel_src = client_cmd.queue.get_nowait()
            self.assertEqual(msg_tel_src["id"], "inclinationTelemetrySource")
            self.assertEqual(
                msg_tel_src["source"], MTM2.InclinationTelemetrySource.ONBOARD
            )

            # Check the temperature offset
            msg_temp_offset = client_cmd.queue.get_nowait()
            self.assertEqual(msg_temp_offset["id"], "temperatureOffset")

            temp_offset = 21.0
            self.assertEqual(msg_temp_offset["ring"], [temp_offset] * 12)
            self.assertEqual(msg_temp_offset["intake"], [temp_offset] * 2)
            self.assertEqual(msg_temp_offset["exhaust"], [temp_offset] * 2)

            # Check the detailed states
            msg_detailed_state_publish_only = client_cmd.queue.get_nowait()
            self.assertEqual(msg_detailed_state_publish_only["id"], "detailedState")
            self.assertEqual(
                msg_detailed_state_publish_only["detailedState"],
                DetailedState.PublishOnly,
            )

            msg_detailed_state_available = client_cmd.queue.get_nowait()
            self.assertEqual(msg_detailed_state_available["id"], "detailedState")
            self.assertEqual(
                msg_detailed_state_available["detailedState"], DetailedState.Available
            )

    async def test_monitor_msg_cmd(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "applyForces")
            await asyncio.sleep(0.5)

            queue = self._skip_init_events(client_cmd.queue, 8)
            self.assertEqual(queue.qsize(), 2)

            msg_ack = queue.get_nowait()
            self.assertEqual(msg_ack["id"], "ack")
            self.assertEqual(msg_ack["sequence_id"], 1)

            msg_success = queue.get_nowait()
            self.assertEqual(msg_success["id"], "success")
            self.assertEqual(msg_success["sequence_id"], 1)

    def _skip_init_events(self, queue, num_init_events):
        """Skip the initial events in queue.

        Parameters
        ----------
        queue : `asyncio.Queue`
            Encoded input message.
        num_init_events : `int`
            Number of the initial events to skip.

        Returns
        -------
        queue : `asyncio.Queue`
            Queue that the initial events have been skipped.
        """
        for idx in range(num_init_events):
            queue.get_nowait()

        return queue

    async def test_cmd_enable(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "enable")
            await asyncio.sleep(0.5)

            # The above short sleep time will not get the acknowledgement of
            # success
            self.assertEqual(client_cmd.queue.qsize(), 10)

            queue = self._skip_init_events(client_cmd.queue, 9)
            msg_fb = queue.get_nowait()

            self.assertEqual(msg_fb["id"], "forceBalanceSystemStatus")
            self.assertTrue(msg_fb["status"])

            # Get the success of command because of sleeping time in enabled
            # command
            await asyncio.sleep(8)
            self.assertEqual(client_cmd.queue.qsize(), 1)

            self.assertTrue(server._command.system_enabled)

    async def test_cmd_disable(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "disable")
            await asyncio.sleep(0.5)

            self.assertEqual(client_cmd.queue.qsize(), 11)

            queue = self._skip_init_events(client_cmd.queue, 9)
            msg_fb = queue.get_nowait()

            self.assertEqual(msg_fb["id"], "forceBalanceSystemStatus")
            self.assertFalse(msg_fb["status"])

    async def test_cmd_exit_control(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "exitControl")
            await asyncio.sleep(0.5)

            self.assertEqual(client_cmd.queue.qsize(), 12)

            queue = self._skip_init_events(client_cmd.queue, 9)
            msg_detailed_state = queue.get_nowait()

            self.assertEqual(msg_detailed_state["id"], "detailedState")
            self.assertEqual(
                msg_detailed_state["detailedState"], DetailedState.PublishOnly
            )

    async def test_cmd_switch_force_balance_system(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            status = True
            await client_cmd.write(
                MsgType.Command,
                "switchForceBalanceSystem",
                msg_details={"status": status},
            )
            await asyncio.sleep(0.5)

            self.assertEqual(client_cmd.queue.qsize(), 11)

            queue = self._skip_init_events(client_cmd.queue, 9)
            msg_fb = queue.get_nowait()

            self.assertEqual(msg_fb["id"], "forceBalanceSystemStatus")
            self.assertEqual(msg_fb["status"], status)

    async def test_cmd_select_inclination_source(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            source = MTM2.InclinationTelemetrySource.MTMOUNT
            await client_cmd.write(
                MsgType.Command,
                "selectInclinationSource",
                msg_details={"source": source},
            )
            await asyncio.sleep(0.5)

            self.assertEqual(client_cmd.queue.qsize(), 11)

            queue = self._skip_init_events(client_cmd.queue, 9)
            msg_inclination_src = queue.get_nowait()

            self.assertEqual(msg_inclination_src["id"], "inclinationTelemetrySource")
            self.assertEqual(msg_inclination_src["source"], int(source))

    async def test_cmd_set_temperature_offset(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            ring = [11.0] * 12
            intake = [13.0] * 2
            exhaust = [14.0] * 2
            await client_cmd.write(
                MsgType.Command,
                "setTemperatureOffset",
                msg_details={"ring": ring, "intake": intake, "exhaust": exhaust},
            )
            await asyncio.sleep(0.5)

            self.assertEqual(client_cmd.queue.qsize(), 11)

            queue = self._skip_init_events(client_cmd.queue, 9)
            msg_temp_offset = queue.get_nowait()

            self.assertEqual(msg_temp_offset["id"], "temperatureOffset")
            self.assertEqual(msg_temp_offset["ring"], ring)
            self.assertEqual(msg_temp_offset["intake"], intake)
            self.assertEqual(msg_temp_offset["exhaust"], exhaust)

    async def test_telemetry(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            # Check the telemetry
            await asyncio.sleep(1)
            self.assertGreater(client_tel.queue.qsize(), 10)

    async def test_telemetry_power_status(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            # Check the initial telemetry
            await asyncio.sleep(0.5)

            msg_power_status = client_tel.queue.get_nowait()
            self.assertEqual(msg_power_status["id"], "powerStatus")
            self.assertLess(abs(msg_power_status["motorVoltage"]), 1)
            self.assertLess(abs(msg_power_status["motorCurrent"]), 1)
            self.assertGreater(abs(msg_power_status["commVoltage"]), 20)
            self.assertGreater(abs(msg_power_status["commCurrent"]), 5)

            # Enter the Enabled state
            await client_cmd.write(MsgType.Command, "enable")
            await asyncio.sleep(8)

            # Check the motor is on in the Enabled state
            while not client_tel.queue.empty():
                msg = client_tel.queue.get_nowait()
                if msg["id"] == "powerStatus":
                    msg_power_status = msg.copy()

            self.assertGreater(abs(msg_power_status["motorVoltage"]), 20)
            self.assertGreater(abs(msg_power_status["motorCurrent"]), 5)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
