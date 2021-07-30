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

import pathlib
import unittest
import contextlib
import asyncio
import logging
import numpy as np

from lsst.ts import tcpip
from lsst.ts.idl.enums import MTM2

from lsst.ts.m2 import MsgType, DetailedState, TcpClient, MockServer


class TestMockServer(unittest.IsolatedAsyncioTestCase):
    """Test the MockServer class."""

    @classmethod
    def setUpClass(cls):
        cls.config_dir = pathlib.Path(__file__).parents[0]
        cls.host = tcpip.LOCAL_HOST
        cls.log = logging.getLogger()
        cls.maxsize_queue = 1000

    @contextlib.asynccontextmanager
    async def make_server(self):
        """Instantiate the mock server of M2 for the test."""

        server = MockServer(
            self.config_dir,
            "harrisLUT",
            self.host,
            port_command=0,
            port_telemetry=0,
            log=self.log,
        )
        await server.start()

        try:
            yield server
        finally:
            await server.close()

        self.assertFalse(server._welcome_message_sent)

        self.assertIsNone(server._message_event)
        self.assertIsNone(server._message_telemetry)

        self.assertFalse(server.model.force_balance_system_status)
        self.assertFalse(server.model.actuator_power_on)
        self.assertTrue(server.model.error_cleared)

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
            self.assertGreaterEqual(client_cmd.queue.qsize(), 8)

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

    async def test_monitor_msg_cmd_ack(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "resetForceOffsets")
            await asyncio.sleep(0.5)

            msg_ack = self._get_latest_message(client_cmd.queue, "ack")

            self.assertEqual(msg_ack["id"], "ack")
            self.assertEqual(msg_ack["sequence_id"], 1)

    def _get_latest_message(self, queue, name):
        """Get the latest message.

        Parameters
        ----------
        queue : `asyncio.Queue`
            Queue of message.
        name : `str`
            Name of message.

        Returns
        -------
        msg_latest : `dict`
            Latest message.
        """

        msg_latest = dict()
        while not queue.empty():
            msg = queue.get_nowait()
            if msg["id"] == name:
                msg_latest = msg.copy()

        return msg_latest

    async def test_monitor_msg_cmd_success(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "resetForceOffsets")
            await asyncio.sleep(0.5)

            msg_success = self._get_latest_message(client_cmd.queue, "success")

            self.assertEqual(msg_success["id"], "success")
            self.assertEqual(msg_success["sequence_id"], 1)

    async def test_cmd_unknown(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "unknown")
            await asyncio.sleep(0.5)

            msg_noack = self._get_latest_message(client_cmd.queue, "noack")

            self.assertEqual(msg_noack["id"], "noack")

    async def test_cell_temperature_high_warning(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            server.model.temperature["exhaust"] = [99, 99]
            await asyncio.sleep(0.5)

            msg_high_temp = self._get_latest_message(
                client_cmd.queue, "cellTemperatureHiWarning"
            )

            self.assertTrue(msg_high_temp["hiWarning"])

    async def test_cmd_enable_noack_success(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "enable")
            await asyncio.sleep(0.5)

            # The above short sleep time will not get the acknowledgement of
            # success
            msg_success = self._get_latest_message(client_cmd.queue, "success")

            self.assertEqual(len(msg_success), 0)

    async def test_cmd_enable_success(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "enable")
            await asyncio.sleep(8)

            # Get the success of command because of sleeping time in enabled
            # command
            msg_success = self._get_latest_message(client_cmd.queue, "success")

            self.assertEqual(msg_success["id"], "success")

            self.assertTrue(server.model.actuator_power_on)
            self.assertTrue(server.model.force_balance_system_status)

    async def test_cmd_disable(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            server.model.actuator_power_on = True
            server.model.force_balance_system_status = True

            await client_cmd.write(MsgType.Command, "disable")
            await asyncio.sleep(0.5)

            msg_fb = self._get_latest_message(
                client_cmd.queue, "forceBalanceSystemStatus"
            )

            self.assertFalse(msg_fb["status"])

            self.assertFalse(server.model.actuator_power_on)
            self.assertFalse(server.model.force_balance_system_status)

    async def test_cmd_standby(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            await client_cmd.write(MsgType.Command, "standby")
            await asyncio.sleep(0.5)

            msg_success = self._get_latest_message(client_cmd.queue, "success")

            self.assertEqual(msg_success["id"], "success")

    async def test_cmd_start(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            await client_cmd.write(MsgType.Command, "start")
            await asyncio.sleep(0.5)

            msg_success = self._get_latest_message(client_cmd.queue, "success")

            self.assertEqual(msg_success["id"], "success")

    async def test_cmd_enter_control(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            await client_cmd.write(MsgType.Command, "enterControl")
            await asyncio.sleep(0.5)

            msg_success = self._get_latest_message(client_cmd.queue, "success")

            self.assertEqual(msg_success["id"], "success")

    async def test_cmd_exit_control(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(MsgType.Command, "exitControl")
            await asyncio.sleep(0.5)

            # Skip the message of detailed state from welcome messages
            queue = self._skip_init_events(client_cmd.queue, 9)
            msg_detailed_state = self._get_latest_message(queue, "detailedState")

            self.assertEqual(
                msg_detailed_state["detailedState"], DetailedState.Available
            )

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

    async def test_cmd_apply_forces(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            force_axial = [1] * server.model.n_actuators
            force_tangent = [2] * server.model.n_tangent_actuators

            await client_cmd.write(
                MsgType.Command,
                "applyForces",
                msg_details={"axial": force_axial, "tangent": force_tangent},
            )

            await asyncio.sleep(0.5)

            np.testing.assert_array_equal(
                server.model.axial_forces["applied"], force_axial
            )
            np.testing.assert_array_equal(
                server.model.tangent_forces["applied"], force_tangent
            )

    async def test_cmd_position_mirror(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            server.model.actuator_power_on = True
            mirror_position_set_point = dict(
                [(axis, 1.0) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
            )

            await client_cmd.write(
                MsgType.Command,
                "positionMirror",
                msg_details=mirror_position_set_point,
            )

            await asyncio.sleep(3.0)

            msg_in_position = self._get_latest_message(
                client_cmd.queue, "m2AssemblyInPosition"
            )

            self.assertTrue(msg_in_position["inPosition"])

            self.assertEqual(server.model.mirror_position, mirror_position_set_point)

    async def test_cmd_reset_force_offsets(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            server.model.axial_forces["applied"] = np.ones(server.model.n_actuators)
            server.model.tangent_forces["applied"] = np.ones(
                server.model.n_tangent_actuators
            )

            await client_cmd.write(MsgType.Command, "resetForceOffsets")
            await asyncio.sleep(0.5)

            self.assertEqual(np.sum(np.abs(server.model.axial_forces["applied"])), 0)
            self.assertEqual(np.sum(np.abs(server.model.tangent_forces["applied"])), 0)

    async def test_cmd_clear_errors(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            server.model.error_cleared = False

            await client_cmd.write(MsgType.Command, "clearErrors")
            await asyncio.sleep(0.5)

            self.assertTrue(server.model.error_cleared)

    async def test_cmd_switch_force_balance_system_fail(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            await client_cmd.write(
                MsgType.Command,
                "switchForceBalanceSystem",
                msg_details={"status": True},
            )
            await asyncio.sleep(0.5)

            msg_fb = self._get_latest_message(
                client_cmd.queue, "forceBalanceSystemStatus"
            )

            self.assertEqual(msg_fb["status"], False)

            self.assertFalse(server.model.force_balance_system_status)

    async def test_cmd_switch_force_balance_system_success(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            server.model.actuator_power_on = True
            await client_cmd.write(
                MsgType.Command,
                "switchForceBalanceSystem",
                msg_details={"status": True},
            )
            await asyncio.sleep(0.5)

            msg_fb = self._get_latest_message(
                client_cmd.queue, "forceBalanceSystemStatus"
            )

            self.assertEqual(msg_fb["status"], True)

            self.assertTrue(server.model.force_balance_system_status)

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

            msg_inclination_src = self._get_latest_message(
                client_cmd.queue, "inclinationTelemetrySource"
            )

            self.assertEqual(msg_inclination_src["source"], int(source))

            self.assertEqual(server.model.inclination_source, source)

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

            msg_temp_offset = self._get_latest_message(
                client_cmd.queue, "temperatureOffset"
            )

            self.assertEqual(msg_temp_offset["ring"], ring)
            self.assertEqual(msg_temp_offset["intake"], intake)
            self.assertEqual(msg_temp_offset["exhaust"], exhaust)

            np.testing.assert_array_equal(server.model.temperature["ring"], ring)
            np.testing.assert_array_equal(server.model.temperature["intake"], intake)
            np.testing.assert_array_equal(server.model.temperature["exhaust"], exhaust)

    async def test_telemetry_no_actuator_power(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):

            # Check the telemetry
            await asyncio.sleep(1)
            self.assertGreater(client_tel.queue.qsize(), 10)

    async def test_telemetry_with_actuator_power(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            server.model.actuator_power_on = True
            server.model.switch_force_balance_system(True)

            # Check the telemetry
            await asyncio.sleep(3)
            self.assertGreater(client_tel.queue.qsize(), 500)

    async def test_telemetry_get_mtmount_elevation(self):
        async with self.make_server() as server, self.make_clients(server) as (
            client_cmd,
            client_tel,
        ):
            elevation_angle = 30.0
            await client_tel.write(
                MsgType.Telemetry,
                "elevation",
                msg_details={"actualPosition": elevation_angle},
                comp_name="MTMount",
            )

            await asyncio.sleep(0.5)

            self.assertEqual(server.model.zenith_angle, 90.0 - elevation_angle)

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
            msg_power_status = self._get_latest_message(client_tel.queue, "powerStatus")

            self.assertGreater(abs(msg_power_status["motorVoltage"]), 20)
            self.assertGreater(abs(msg_power_status["motorCurrent"]), 5)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
