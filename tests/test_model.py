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

import asyncio
import logging
import sys
import contextlib
import unittest
import pathlib

from lsst.ts import tcpip
from lsst.ts import salobj
from lsst.ts.m2 import MockServer, Model, CommandStatus, MsgType


class TestModel(unittest.IsolatedAsyncioTestCase):
    """Test the Model class."""

    @classmethod
    def setUpClass(cls):
        cls.config_dir = pathlib.Path(__file__).parents[0]
        cls.host = tcpip.LOCAL_HOST
        cls.timeout_in_second = 0.05

        logging.basicConfig(
            level=logging.INFO, handlers=[logging.StreamHandler(sys.stdout)]
        )
        cls.log = logging.getLogger()

    @contextlib.asynccontextmanager
    async def make_server(self):
        """Instantiate the mock server of M2 for the test."""

        server = MockServer(
            self.host,
            timeout_in_second=self.timeout_in_second,
            port_command=0,
            port_telemetry=0,
            log=self.log,
        )
        server.model.configure(self.config_dir, "harrisLUT")
        await server.start()

        try:
            yield server
        finally:
            await server.close()

    @contextlib.asynccontextmanager
    async def make_model(self, server):
        """Make the model (or TCP/IP client) that talks to the server and wait
        for it to connect.

        Returns Model.
        """

        model = Model(log=self.log)
        model.start(
            server.server_command.host,
            server.server_command.port,
            server.server_telemetry.port,
        )

        # Wait a little time to construct the connection
        await asyncio.sleep(2)

        try:
            yield model
        finally:
            await model.close()

        self.assertFalse(model._start_connection)
        self.assertIsNone(model.client_command)
        self.assertIsNone(model.client_telemetry)
        self.assertEqual(model.last_command_status, CommandStatus.Unknown)

    async def test_close(self):
        model = Model()
        await model.close()

        model.start(tcpip.LOCAL_HOST, 0, 0)
        await model.close()

    async def test_are_clients_connected(self):
        async with self.make_server() as server, self.make_model(server) as model:
            self.assertTrue(model.are_clients_connected())

    def test_are_clients_connected_no_connection(self):
        model = Model()

        self.assertFalse(model.are_clients_connected())

    async def test_task_connection(self):
        async with self.make_server() as server, self.make_model(server) as model:

            # Connection is on in the initial beginning
            self.assertTrue(model.are_clients_connected())
            self.assertTrue(model._start_connection)

            # Close the connection between the client and server
            await asyncio.gather(
                server.server_command.close_client(),
                server.server_telemetry.close_client(),
            )
            self.assertFalse(model.are_clients_connected())

            # Wait a little time to reconstruct the connection
            await asyncio.sleep(1)
            self.assertTrue(model.are_clients_connected())

    async def test_task_analyze_message(self):
        async with self.make_server() as server, self.make_model(server) as model:

            # Wait a little time to collect the event messages
            await asyncio.sleep(1)
            self.assertEqual(model.queue_event.qsize(), 9)

    async def test_controller_state(self):
        async with self.make_server() as server, self.make_model(server) as model:

            # Wait a little time to collect the event messages
            await asyncio.sleep(1)
            self.assertEqual(model.controller_state, salobj.State.OFFLINE)

            # Check to get the Fault state
            server.model.fault()
            await asyncio.sleep(1)

            self.assertEqual(model.controller_state, salobj.State.FAULT)

    async def test_last_command_status_ack_success(self):
        async with self.make_server() as server, self.make_model(server) as model:

            await model.client_command.write(MsgType.Command, "enable")

            # Wait a little time to collect the messages
            await asyncio.sleep(2)
            self.assertEqual(model.last_command_status, CommandStatus.Ack)

            # Wait a little time to collect the messages
            await asyncio.sleep(7)
            self.assertEqual(model.last_command_status, CommandStatus.Success)

    async def test_write_command_to_server_success(self):
        async with self.make_server() as server, self.make_model(server) as model:

            await model.write_command_to_server(
                "enterControl", controller_state_expected=salobj.State.STANDBY
            )

            self.assertEqual(model.controller_state, salobj.State.STANDBY)

    async def test_write_command_to_server_wrong_expectation(self):
        async with self.make_server() as server, self.make_model(server) as model:

            with self.assertRaises(RuntimeError):
                await model.write_command_to_server(
                    "enterControl", controller_state_expected=salobj.State.ENABLED
                )

    async def test_write_command_to_server_short_timeout(self):
        async with self.make_server() as server, self.make_model(server) as model:

            with self.assertRaises(RuntimeError):
                await model.write_command_to_server(
                    "enable",
                    timeout=2.0,
                    controller_state_expected=salobj.State.ENABLED,
                )

    async def test_write_command_to_server_fail(self):
        async with self.make_server() as server, self.make_model(server) as model:

            with self.assertRaises(RuntimeError):
                await model.write_command_to_server(
                    "switchForceBalanceSystem", message_details={"status": True}
                )

    async def test_write_command_to_closed_server(self):
        async with self.make_server() as server, self.make_model(server) as model:

            await server.close()

            with self.assertRaises(OSError):
                await model.write_command_to_server(
                    "switchForceBalanceSystem", message_details={"status": True}
                )

    async def test_write_command_to_server_no_this_command(self):
        async with self.make_server() as server, self.make_model(server) as model:

            with self.assertRaises(RuntimeError):
                await model.write_command_to_server("noThisCommand")

    async def test_write_command_to_server_no_connection(self):

        model = Model()
        with self.assertRaises(OSError):
            await model.write_command_to_server("noConnection")

    async def test_clear_errors(self):
        async with self.make_server() as server, self.make_model(server) as model:

            # Fake the error
            server.model.fault()
            await asyncio.sleep(1)
            self.assertEqual(model.controller_state, salobj.State.FAULT)

            # Clear the error
            await model.clear_errors()

            # Check the controller's state
            self.assertTrue(server.model.error_cleared)
            self.assertEqual(model.controller_state, salobj.State.OFFLINE)

    def test_is_error_code_zero(self):

        model = Model()

        # Error code is 0
        message = {"id": "errorCode", "errorCode": 0}
        self.assertTrue(model._is_error_code_zero(message))

        # Error code is not 0
        message = {"id": "errorCode", "errorCode": 1}
        self.assertFalse(model._is_error_code_zero(message))

    def test_is_controller_state(self):

        model = Model()

        # Is controller's state
        message = {"id": "summaryState", "summaryState": 1}
        self.assertTrue(model._is_controller_state(message))

        # Isn't controller's state
        message = {"id": "temp", "temp": 1}
        self.assertFalse(model._is_controller_state(message))

    def test_assert_controller_state(self):

        model = Model()

        # Allowed state
        model.assert_controller_state("enterControl", [salobj.State.OFFLINE])

        # Disallowed state
        self.assertRaises(
            ValueError,
            model.assert_controller_state,
            "enterControl",
            [salobj.State.ENABLED],
        )


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
