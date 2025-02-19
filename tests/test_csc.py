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
import unittest

import numpy as np
from lsst.ts import salobj
from lsst.ts.m2 import M2
from lsst.ts.m2com import (
    DEFAULT_ENABLED_FAULTS_MASK,
    LIMIT_FORCE_AXIAL_CLOSED_LOOP,
    LIMIT_FORCE_TANGENT_CLOSED_LOOP,
    NUM_ACTUATOR,
    NUM_INNER_LOOP_CONTROLLER,
    NUM_TANGENT_LINK,
    NUM_TEMPERATURE_EXHAUST,
    NUM_TEMPERATURE_INTAKE,
    NUM_TEMPERATURE_RING,
    TEST_DIGITAL_INPUT_NO_POWER,
    TEST_DIGITAL_OUTPUT_NO_POWER,
    DigitalOutputStatus,
    LimitSwitchType,
    MockErrorCode,
)
from lsst.ts.xml.enums import MTM2

# Timeout for fast operations (seconds)
STD_TIMEOUT = 15

SLEEP_TIME_SHORT = 3
SLEEP_TIME_MEDIUM = 5
SLEEP_TIME_LONG = 10


class TestM2CSC(salobj.BaseCscTestCase, unittest.IsolatedAsyncioTestCase):
    def basic_make_csc(
        self,
        initial_state: salobj.State,
        config_dir: salobj.PathType | None,
        simulation_mode: int,
    ) -> M2:
        return M2(
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

    def setUp(self) -> None:
        super().setUp()
        self.csc_mtmount: salobj.Controller | None = None

    async def _simulate_csc_mount(self, elevation_angle: float) -> None:
        """Simulate the MTMount CSC.

        Parameters
        ----------
        elevation_angle : `float`
            Elevation angle in degree.
        """

        self.csc_mtmount = salobj.Controller("MTMount")
        await self.csc_mtmount.start_task

        await self.csc_mtmount.tel_elevation.set_write(actualPosition=elevation_angle)

        # Wait for some time to publish the telemetry of MTMount
        await asyncio.sleep(SLEEP_TIME_SHORT)

    async def asyncTearDown(self) -> None:
        if self.csc_mtmount is not None:
            await self.csc_mtmount.close()

    async def test_bin_script(self) -> None:
        await self.check_bin_script(
            name="MTM2",
            index=None,
            exe_name="run_mtm2",
            default_initial_state=salobj.State.STANDBY,
        )

        await self.check_bin_script(
            name="MTM2",
            index=None,
            exe_name="run_mtm2",
            default_initial_state=salobj.State.STANDBY,
            cmdline_args=["--host", "127.0.0.1", "--ports", "0", "1"],
        )

    async def test_instantiation_m2_normal_mode(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=0
        ):
            self.assertIsNone(self.csc.controller_cell.mock_server)
            self.assertFalse(self.csc.controller_cell.are_clients_connected())

    async def test_is_gui_commander(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Change the commander to the GUI
            await self.csc.controller_cell.mock_server._message_event.write_commandable_by_dds(
                False
            )
            await asyncio.sleep(SLEEP_TIME_SHORT)

            self.assertFalse(self.csc.is_csc_commander())

            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_clearErrors.set_start(timeout=STD_TIMEOUT)

    async def test_start(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Flush the topics
            topics = [
                "tcpIpConnected",
                "commandableByDDS",
                "hardpointList",
                "interlock",
                "inclinationTelemetrySource",
                "temperatureOffset",
                "digitalOutput",
                "digitalInput",
                "config",
                "closedLoopControlMode",
                "enabledFaultsMask",
                "configurationFiles",
                "powerSystemState",
                "disabledILC",
            ]
            for topic in topics:
                getattr(self.remote, f"evt_{topic}").flush()

            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Check the summary state
            self.assertEqual(self.csc.summary_state, salobj.State.DISABLED)

            data_summary_state = await self.remote.evt_summaryState.aget(
                timeout=STD_TIMEOUT
            )
            self.assertEqual(
                data_summary_state.summaryState, int(salobj.State.DISABLED)
            )

            # Wait a little time for the data to come from server
            await asyncio.sleep(SLEEP_TIME_MEDIUM)

            # Check the TCP/IP connection is on
            self.assertIsNotNone(self.csc.controller_cell.mock_server)
            self.assertTrue(self.csc.controller_cell.are_clients_connected())

            # Simulate the MTMount CSC
            elevation_angle = 10.0
            await self._simulate_csc_mount(elevation_angle)

            # There should be no update of inclinometer angle from MTMount CSC
            mock_model = self.csc.controller_cell.mock_server.model
            self.assertEqual(mock_model.inclinometer_angle, 90.0)

            # Check the welcome messages
            await self.assert_next_sample(
                self.remote.evt_tcpIpConnected, timeout=STD_TIMEOUT, isConnected=True
            )

            await self.assert_next_sample(
                self.remote.evt_commandableByDDS, timeout=STD_TIMEOUT, state=True
            )

            await self.assert_next_sample(
                self.remote.evt_hardpointList,
                timeout=STD_TIMEOUT,
                actuators=[6, 16, 26, 74, 76, 78],
            )

            ilcs = [False] * NUM_INNER_LOOP_CONTROLLER
            ilcs[5] = True
            await self.assert_next_sample(
                self.remote.evt_disabledILC,
                timeout=STD_TIMEOUT,
                ilcs=ilcs,
            )

            await self.assert_next_sample(
                self.remote.evt_interlock, timeout=STD_TIMEOUT, state=False
            )

            await self.assert_next_sample(
                self.remote.evt_inclinationTelemetrySource,
                timeout=STD_TIMEOUT,
                source=int(MTM2.InclinationTelemetrySource.ONBOARD),
            )

            await self.assert_next_sample(
                self.remote.evt_temperatureOffset,
                timeout=STD_TIMEOUT,
                ring=[21.0] * NUM_TEMPERATURE_RING,
                intake=[0.0] * NUM_TEMPERATURE_INTAKE,
                exhaust=[0.0] * NUM_TEMPERATURE_EXHAUST,
            )

            await self.assert_next_sample(
                self.remote.evt_digitalOutput,
                timeout=STD_TIMEOUT,
                value=TEST_DIGITAL_OUTPUT_NO_POWER,
            )

            await self.assert_next_sample(
                self.remote.evt_digitalInput,
                timeout=STD_TIMEOUT,
                value=hex(TEST_DIGITAL_INPUT_NO_POWER),
            )

            await self.assert_next_sample(
                self.remote.evt_config,
                timeout=STD_TIMEOUT,
                cellTemperatureDelta=2.0,
            )

            await self.assert_next_sample(
                self.remote.evt_closedLoopControlMode,
                timeout=STD_TIMEOUT,
                mode=MTM2.ClosedLoopControlMode.Idle,
            )

            await self.assert_next_sample(
                self.remote.evt_enabledFaultsMask,
                timeout=STD_TIMEOUT,
                mask=hex(DEFAULT_ENABLED_FAULTS_MASK),
            )

            data_configuration_files = await self.remote.evt_configurationFiles.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertNotEqual(data_configuration_files.files, "")

            await self.assert_next_sample(
                self.remote.evt_powerSystemState,
                timeout=STD_TIMEOUT,
                powerType=MTM2.PowerType.Communication,
                status=False,
                state=MTM2.PowerSystemState.Init,
            )

            await self.assert_next_sample(
                self.remote.evt_powerSystemState,
                timeout=STD_TIMEOUT,
                powerType=MTM2.PowerType.Motor,
                status=False,
                state=MTM2.PowerSystemState.Init,
            )

            self.assertTrue(self.csc.is_csc_commander())

            # Check the telemetry in OFFLINE state
            data_power_status = await self.remote.tel_powerStatus.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertLess(abs(data_power_status.motorVoltage), 1)
            self.assertLess(abs(data_power_status.motorCurrent), 1)
            self.assertLess(abs(data_power_status.commVoltage), 1)
            self.assertLess(abs(data_power_status.commCurrent), 1)

            data_power_status_raw = await self.remote.tel_powerStatusRaw.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertLess(abs(data_power_status_raw.motorVoltage), 1)
            self.assertLess(abs(data_power_status_raw.motorCurrent), 1)
            self.assertLess(abs(data_power_status_raw.commVoltage), 1)
            self.assertLess(abs(data_power_status_raw.commCurrent), 1)

            data_disp_sensors = await self.remote.tel_displacementSensors.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertNotEqual(data_disp_sensors.thetaZ, [0] * 6)
            self.assertNotEqual(data_disp_sensors.deltaZ, [0] * 6)

    async def test_select_inclination_source(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            control_parameters = self.csc.controller_cell.control_parameters
            control_parameters["use_external_elevation_angle"] = True
            control_parameters["enable_angle_comparison"] = True
            control_parameters["max_angle_difference"] = 3.0

            self.csc._select_inclination_source(
                MTM2.InclinationTelemetrySource.ONBOARD, 2.0, False
            )

            self.assertFalse(control_parameters["use_external_elevation_angle"])
            self.assertFalse(control_parameters["enable_angle_comparison"])
            self.assertEqual(control_parameters["max_angle_difference"], 2.0)

    async def test_standby_no_fault(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)
            self.assertTrue(self.csc.controller_cell.are_clients_connected())

            # Overwrite some internal data
            self.csc._is_overwritten_hardpoints = True
            self.csc._is_overwritten_configuration_file = True

            # Do the standby to disconnect the server
            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)
            self.assertFalse(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # The internal data should be reset
            self.assertFalse(self.csc._is_overwritten_hardpoints)
            self.assertFalse(self.csc._is_overwritten_configuration_file)

            # Check the summary state
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

    async def test_standby_fault(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Enabled state first
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            # Flush the topics
            for topic in ("errorCode", "summaryFaultsStatus"):
                getattr(self.remote, f"evt_{topic}").flush()

            # Make the server fault
            mock_model = self.csc.controller_cell.mock_server.model
            mock_model.fault(MockErrorCode.LimitSwitchTriggeredClosedloop.value)
            await asyncio.sleep(SLEEP_TIME_MEDIUM)

            self.assertEqual(self.csc.summary_state, salobj.State.FAULT)

            # Check the events
            await self.assert_next_sample(
                self.remote.evt_summaryFaultsStatus,
                timeout=STD_TIMEOUT,
                status=hex(2**6),
            )

            await self.assert_next_sample(
                self.remote.evt_errorCode,
                timeout=STD_TIMEOUT,
                errorCode=1,
            )

            await self.assert_next_sample(
                self.remote.evt_errorCode,
                timeout=STD_TIMEOUT,
                errorCode=6056,
                errorReport="Actuator Limit Switch Triggered [Closed-loop]",
            )

            # Do the standby to disconnect the server
            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)
            self.assertFalse(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # Check the summary state
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

            # Check the server fault
            self.assertFalse(mock_model.error_handler.exists_error())

    async def test_standby_fault_accidental(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Put the csc to the Fault state
            await self.csc.fault(code=0, report="test fault")

            # Transition to the Standby state
            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)

            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

    async def test_enable(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Change the hardpoints
            await self.remote.cmd_setHardpointList.set_start(
                actuators=[3, 13, 23, 73, 75, 77]
            )
            await asyncio.sleep(SLEEP_TIME_SHORT)

            # Put the overwritten to False to let the setting in configuration
            # file can be used.
            self.csc._is_overwritten_hardpoints = False

            # Flush the topics
            for topic in ("config", "hardpointList"):
                getattr(self.remote, f"evt_{topic}").flush()

            # Go to the Enabled state
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            await self.assert_next_sample(
                self.remote.evt_config,
                timeout=STD_TIMEOUT,
                configuration="Configurable_File_Description_20180831T091922_M2_optical.csv",
            )

            await self.assert_next_sample(
                self.remote.evt_hardpointList,
                timeout=STD_TIMEOUT,
                actuators=[6, 16, 26, 74, 76, 78],
            )

            # Check all ILCs are enabled and the system is under the
            # closed-loop control
            self.assertTrue(self.csc.controller_cell.are_ilc_modes_enabled())
            self.assertEqual(
                self.csc.controller_cell.closed_loop_control_mode,
                MTM2.ClosedLoopControlMode.ClosedLoop,
            )

            # Check events
            await asyncio.sleep(SLEEP_TIME_LONG)

            data_power_system_state = self.remote.evt_powerSystemState.get()
            self.assertTrue(data_power_system_state.status)

            data_inner_loop_control_mode = self.remote.evt_innerLoopControlMode.get()
            self.assertEqual(data_inner_loop_control_mode.address, NUM_ACTUATOR - 1)

            self._assert_open_loop_max_limit(False)

    def _assert_open_loop_max_limit(self, expected_status: bool) -> None:
        data = self.remote.evt_openLoopMaxLimit.get()
        self.assertEqual(data.status, expected_status)

    async def test_enable_power_on(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Fake the condition that the M2 GUI powered on the system already
            mock_server = self.csc.controller_cell.mock_server
            power_communication = mock_server.model.power_communication
            power_motor = mock_server.model.power_motor

            powers = [power_communication, power_motor]
            power_types = [MTM2.PowerType.Communication, MTM2.PowerType.Motor]
            for power, power_type in zip(powers, power_types):
                await power.power_on()
                await power.wait_power_fully_on()

                await mock_server._message_event.write_power_system_state(
                    power_type,
                    power.is_power_on(),
                    power.state,
                )

            # Sleep a short time to handle the events
            await asyncio.sleep(SLEEP_TIME_SHORT)

            # Go to the Enabled state
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            await asyncio.sleep(SLEEP_TIME_SHORT)

            controller_cell = self.csc.controller_cell
            self.assertTrue(controller_cell.are_ilc_modes_enabled())
            self.assertEqual(
                controller_cell.closed_loop_control_mode,
                MTM2.ClosedLoopControlMode.ClosedLoop,
            )
            self.assertTrue(self.csc.system_is_ready)

    async def test_enable_interlock(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Go to the Enabled state
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            # Trigger the interlock fault
            await self.csc.controller_cell.set_bit_digital_status(
                2, DigitalOutputStatus.BinaryLowLevel
            )

            # Wait a little time to process the data
            await asyncio.sleep(SLEEP_TIME_SHORT)

            # The interlock event should transition the system to Fault state
            self.assertEqual(self.csc.summary_state, salobj.State.FAULT)

    async def test_disable(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Go to the Enabled state
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            # Go to the Disabled state
            await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)

            # Check the motor's power is off and the communication power is
            # still on
            power_system_status = self.csc.controller_cell.power_system_status
            self.assertFalse(power_system_status["motor_power_is_on"])
            self.assertTrue(power_system_status["communication_power_is_on"])

            # The system will only publish the telemetry
            self.assertEqual(
                self.csc.controller_cell.closed_loop_control_mode,
                MTM2.ClosedLoopControlMode.TelemetryOnly,
            )

    async def test_command_fail_wrong_state(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)

            # Because there is no connection now. This command will transition
            # the system into FAULT state
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)

    async def test_check_standard_state_transitions(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # BaseCscTestCase.check_standard_state_transitions() can not be
            # used here because I need time to let the connection to be
            # constructed

            self.assertFalse(self.csc.system_is_ready)

            await self.assert_next_summary_state(
                salobj.State.STANDBY, timeout=STD_TIMEOUT
            )

            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.DISABLED, timeout=STD_TIMEOUT
            )

            self.assertFalse(self.csc.system_is_ready)

            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.ENABLED, timeout=STD_TIMEOUT
            )

            self.assertTrue(self.csc.system_is_ready)

            await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.DISABLED, timeout=STD_TIMEOUT
            )

            self.assertFalse(self.csc.system_is_ready)

            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.STANDBY, timeout=STD_TIMEOUT
            )

            self.assertFalse(self.csc.system_is_ready)

    async def test_set_summary_state(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            self.assertEqual(self.csc.summary_state, salobj.State.ENABLED)

    async def test_connection_multiple_times(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # First time of connection

            # Enter the Enabled state to construct the connection and make
            # sure I can get the controller's state event
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)
            self.assertTrue(self.csc.controller_cell.are_clients_connected())
            self.assertTrue(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # Check the last sequence ID
            self.assertEqual(
                self.csc.controller_cell.client_command.last_sequence_id, 21
            )

            # Enter the Standby state to close the connection
            await salobj.set_summary_state(self.remote, salobj.State.STANDBY)
            self.assertFalse(self.csc.controller_cell.are_clients_connected())
            self.assertFalse(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # Second time of connection

            # Enter the Enabled state to construct the connection and make
            # sure I can get the controller's state event
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)
            self.assertTrue(self.csc.controller_cell.are_clients_connected())
            self.assertTrue(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # Check the last sequence ID. Note the value should be 49 instead
            # of 21 from the previous connection.
            self.assertEqual(
                self.csc.controller_cell.client_command.last_sequence_id, 49
            )

    async def test_telemetry_loop(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            for tel in self.remote.salinfo.telemetry_names:
                with self.subTest(telemetry=tel):
                    await getattr(self.remote, f"tel_{tel}").next(
                        flush=True, timeout=STD_TIMEOUT
                    )

    async def test_connection_monitor_loop(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await salobj.set_summary_state(self.remote, salobj.State.DISABLED)
            self.assertTrue(self.csc.controller_cell.are_clients_connected())

            time_wait_connection_monitor_check = 2
            await asyncio.sleep(time_wait_connection_monitor_check)

            await self.csc.controller_cell.mock_server.close()

            # Sleep some time to make sure the CSC detects the connection is
            # closed
            await asyncio.sleep(time_wait_connection_monitor_check)

            self.assertFalse(self.csc.controller_cell.are_clients_connected())
            self.assertFalse(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            self.assertEqual(self.csc.summary_state, salobj.State.FAULT)

    async def test_applyForces_wrong_state(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            axial = np.round(
                np.random.normal(
                    size=len(self.remote.cmd_applyForces.DataType().axial)
                ),
                decimals=5,
            )
            tangent = np.round(
                np.random.normal(
                    size=len(self.remote.cmd_applyForces.DataType().tangent)
                ),
                decimals=5,
            )

            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_applyForces.set_start(
                    axial=axial, tangent=tangent, timeout=STD_TIMEOUT
                )

    async def test_applyForces_wrong_control_mode(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            axial = np.round(
                np.random.normal(
                    size=len(self.remote.cmd_applyForces.DataType().axial)
                ),
                decimals=5,
            )
            tangent = np.round(
                np.random.normal(
                    size=len(self.remote.cmd_applyForces.DataType().tangent)
                ),
                decimals=5,
            )

            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_applyForces.set_start(
                    axial=axial, tangent=tangent, timeout=STD_TIMEOUT
                )

    async def test_applyForces(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            axial = np.round(
                np.random.normal(
                    size=len(self.remote.cmd_applyForces.DataType().axial)
                ),
                decimals=5,
            )
            tangent = np.round(
                np.random.normal(
                    size=len(self.remote.cmd_applyForces.DataType().tangent)
                ),
                decimals=5,
            )

            # Wait for m2AssemblyInPosition to be in position before applying
            # the force.
            in_position = (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition
            while not in_position:
                in_position = (
                    await self.remote.evt_m2AssemblyInPosition.next(
                        flush=False, timeout=STD_TIMEOUT
                    )
                ).inPosition

            await self.remote.cmd_applyForces.set_start(
                axial=axial, tangent=tangent, timeout=STD_TIMEOUT
            )

            mock_model = self.csc.controller_cell.mock_server.model
            np.testing.assert_array_equal(
                mock_model.control_closed_loop.axial_forces["applied"], axial
            )
            np.testing.assert_array_equal(
                mock_model.control_closed_loop.tangent_forces["applied"], tangent
            )

            await asyncio.sleep(SLEEP_TIME_SHORT)

            self.remote.tel_tangentForce.flush()
            self.remote.tel_axialForce.flush()

            tangent_forces = await self.remote.tel_tangentForce.next(
                flush=True, timeout=STD_TIMEOUT
            )
            axial_forces = await self.remote.tel_axialForce.next(
                flush=True, timeout=STD_TIMEOUT
            )

            self.assertFalse(
                np.any(tangent_forces.applied != tangent),
                f"tangentForcesApplied{tangent_forces.applied} != requested{tangent}",
            )
            self.assertFalse(
                np.any(axial_forces.applied != axial),
                f"axialForcesApplied{axial_forces.applied} != requested{axial}",
            )

            await self.remote.cmd_resetForceOffsets.start(timeout=STD_TIMEOUT)

            self.assertEqual(
                np.sum(np.abs(mock_model.control_closed_loop.axial_forces["applied"])),
                0,
            )
            self.assertEqual(
                np.sum(
                    np.abs(mock_model.control_closed_loop.tangent_forces["applied"])
                ),
                0,
            )

            await asyncio.sleep(SLEEP_TIME_SHORT)

            self.remote.tel_tangentForce.flush()
            self.remote.tel_axialForce.flush()

            tangent_forces = await self.remote.tel_tangentForce.next(flush=True)
            axial_forces = await self.remote.tel_axialForce.next(flush=True)

            self.assertFalse(
                np.any(tangent_forces.applied != np.zeros_like(tangent)),
                f"tangentForcesApplied{tangent_forces.applied} != requested{tangent}",
            )
            self.assertFalse(
                np.any(axial_forces.applied != np.zeros_like(axial)),
                f"axialForcesApplied{axial_forces.applied} != requested{axial}",
            )

            # Check sending axial forces out of limit
            mock_model = self.csc.controller_cell.mock_server.model

            n_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
            set_axial_force = np.zeros(n_axial_actuators)
            set_axial_force[0] = LIMIT_FORCE_AXIAL_CLOSED_LOOP + 1.0

            with self.assertRaises(
                salobj.AckError,
                msg="Axial set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    axial=set_axial_force, timeout=STD_TIMEOUT
                )

            # Check sending tangent forces out of limit
            set_tangent_force = np.zeros(NUM_TANGENT_LINK)
            set_tangent_force[0] = LIMIT_FORCE_TANGENT_CLOSED_LOOP + 1.0

            with self.assertRaises(
                salobj.AckError,
                msg="Tangent set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    tangent=set_tangent_force, timeout=STD_TIMEOUT
                )

    async def test_check_applied_forces_in_range(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # Wait until we have the force data from simulator
            await self.assert_next_sample(
                self.remote.tel_tangentForce,
                timeout=STD_TIMEOUT,
            )
            await self.assert_next_sample(
                self.remote.tel_axialForce,
                timeout=STD_TIMEOUT,
            )

            # Force is out of range
            applied_force_axial = [0] * (NUM_ACTUATOR - NUM_TANGENT_LINK)
            applied_force_tangent = [
                LIMIT_FORCE_TANGENT_CLOSED_LOOP + 1.0
            ] * NUM_TANGENT_LINK
            self.assertRaises(
                ValueError,
                self.csc._check_applied_forces_in_range,
                applied_force_axial,
                applied_force_tangent,
            )

    async def test_positionMirror(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # Wait for some time for the closed-loop control to be done
            await asyncio.sleep(STD_TIMEOUT)

            # Wait for m2AssemblyInPosition to be in position before applying
            # the force.
            in_position = (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition
            while not in_position:
                in_position = (
                    await self.remote.evt_m2AssemblyInPosition.next(
                        flush=False, timeout=STD_TIMEOUT
                    )
                ).inPosition

            # Move the rigid body to the new position. Note the units are um
            # and arcsec.
            position_send = {
                "x": 1,
                "y": 2,
                "z": 3,
                "xRot": -1,
                "yRot": -2,
                "zRot": -3,
            }

            await self.remote.cmd_positionMirror.set_start(
                **position_send, timeout=STD_TIMEOUT
            )

            # Wait for some time for the movement to be done
            await asyncio.sleep(SLEEP_TIME_LONG)

            # Check the new position
            self.remote.tel_position.flush()

            tolerance = 0.3

            position_set = await self.remote.tel_position.next(
                flush=False, timeout=STD_TIMEOUT
            )
            for axis in ("x", "y", "z", "xRot", "yRot", "zRot"):
                with self.subTest(telemetry="position", axis=axis):
                    self.assertLess(
                        abs(getattr(position_set, axis) - position_send[axis]),
                        tolerance,
                    )

    async def test_selectInclinationSource(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # Simulate the MTMount CSC
            elevation = np.random.random() * 60.0 + 20.0
            await self._simulate_csc_mount(elevation)

            incl_source = await self.remote.evt_inclinationTelemetrySource.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertEqual(
                incl_source.source, MTM2.InclinationTelemetrySource.ONBOARD
            )

            n_samples = 10
            zenith_angle_values = np.zeros(n_samples)
            for i in range(n_samples):
                zenith_angle = await self.remote.tel_zenithAngle.next(
                    flush=True, timeout=STD_TIMEOUT
                )

                zenith_angle_values[i] = zenith_angle.measured

            # Because the source is still ONBOARD, the M2 should not care about
            # the value from the MTMount. The default value is 0.
            # The value of 0.94 here comes from the calibrated offset.
            inclinometer_rms = 0.05
            self.assertAlmostEqual(
                np.mean(zenith_angle_values),
                0.94,
                int(np.ceil(-np.log10(inclinometer_rms))) - 1,
            )

            self.remote.evt_inclinationTelemetrySource.flush()

            # Change the source of inclination

            # This should fail in the ENABLED state
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_selectInclinationSource.set_start(
                    source=MTM2.InclinationTelemetrySource.MTMOUNT
                )

            # This can only be done in the DISABLED state
            await salobj.set_summary_state(self.remote, salobj.State.DISABLED)
            await self.remote.cmd_selectInclinationSource.set_start(
                source=MTM2.InclinationTelemetrySource.MTMOUNT
            )

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)
            incl_source = await self.remote.evt_inclinationTelemetrySource.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertEqual(
                incl_source.source, MTM2.InclinationTelemetrySource.MTMOUNT
            )

            # Workaround of the mypy checking
            assert self.csc_mtmount is not None

            await self.csc_mtmount.tel_elevation.set_write(actualPosition=elevation)
            await asyncio.sleep(SLEEP_TIME_SHORT)

            self.assertEqual(
                self.csc.controller_cell.mock_server.model.inclinometer_angle_external,
                elevation,
            )

    async def test_enableLutTemperature(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # By default, the LUT temperature correction is disabled.
            self.assertFalse(
                self.csc.controller_cell.mock_server.model.control_parameters[
                    "enable_lut_temperature"
                ]
            )

            # This should fail in the ENABLED state
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_enableLutTemperature.set_start(status=True)

            # This can only be done in the DISABLED state
            await salobj.set_summary_state(self.remote, salobj.State.DISABLED)
            await self.remote.cmd_enableLutTemperature.set_start(status=True)

            self.assertTrue(
                self.csc.controller_cell.mock_server.model.control_parameters[
                    "enable_lut_temperature"
                ]
            )

    async def test_switchForceBalanceSystem(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # The default force balance system is on when controlled by SAL
            await asyncio.sleep(SLEEP_TIME_MEDIUM)
            force_balance_status = self.remote.evt_forceBalanceSystemStatus.get()

            self.assertTrue(force_balance_status.status)

            # Switch the force balance system off
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            await asyncio.sleep(SLEEP_TIME_MEDIUM)
            force_balance_status = self.remote.evt_forceBalanceSystemStatus.get()

            self.assertFalse(force_balance_status.status)

            # Switch the force balance system on
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=True)

            # Wait for system to be in position
            while not (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition:
                pass

            # Check axial forces
            axial_forces = await self.remote.tel_axialForce.next(
                flush=True, timeout=STD_TIMEOUT
            )

            # LUT gravity absolute value should be larger than zero
            self.assertTrue(np.all(np.abs(axial_forces.lutGravity) > 0.0))

            # LUT temperature value should be zero.
            self.assertEqual(np.sum(np.abs(axial_forces.lutTemperature)), 0.0)

    async def test_clearErrors(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Fault the mock model and wait for some time to get the state
            # event
            mock_model = self.csc.controller_cell.mock_server.model
            mock_model.fault(MockErrorCode.LimitSwitchTriggeredClosedloop.value)
            await asyncio.sleep(SLEEP_TIME_SHORT)

            self.assertTrue(self.csc.controller_cell.error_handler.exists_error())

            # Clear the error
            await self.remote.cmd_clearErrors.set_start(timeout=STD_TIMEOUT)

            self.assertFalse(mock_model.error_handler.exists_error())
            await asyncio.sleep(SLEEP_TIME_SHORT)

    async def test_setTemperatureOffset(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.DISABLED)

            # Change the offset successfully
            ring = [19.0] * NUM_TEMPERATURE_RING
            intake = [19.0] * NUM_TEMPERATURE_INTAKE
            exhaust = [19.0] * NUM_TEMPERATURE_EXHAUST
            await self.remote.cmd_setTemperatureOffset.set_start(
                ring=ring, intake=intake, exhaust=exhaust
            )

            data_temp_offset = await self.remote.evt_temperatureOffset.aget(
                timeout=STD_TIMEOUT
            )
            np.testing.assert_array_equal(data_temp_offset.ring, ring)
            np.testing.assert_array_equal(
                data_temp_offset.intake, [0.0] * NUM_TEMPERATURE_INTAKE
            )
            np.testing.assert_array_equal(
                data_temp_offset.exhaust, [0.0] * NUM_TEMPERATURE_EXHAUST
            )

    async def test_bypassErrorCode(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # This should fail in the Standby state
            code = 6077
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_bypassErrorCode.set_start(code=code)

            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Check the default enabled faults mask
            self._assert_enabled_faults_mask(DEFAULT_ENABLED_FAULTS_MASK)

            # Unexisted error code should fail
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_bypassErrorCode.set_start(code=12345)

            # Bypass the error code
            await self.remote.cmd_bypassErrorCode.set_start(code=code)

            self.assertEqual(self.csc._error_codes_bypass, {code})

            # Enter the Enabled state to check the mask
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            self._assert_enabled_faults_mask(0xFF000003FFFFFFF8)

    def _assert_enabled_faults_mask(self, expected_mask: int) -> None:
        data = self.remote.evt_enabledFaultsMask.get()
        self.assertEqual(data.mask, hex(expected_mask))

    async def test_resetEnabledFaultsMask(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # This should fail in the Standby state
            code = 6077
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_bypassErrorCode.set_start(code=code)

            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Bypass the error code
            await self.remote.cmd_bypassErrorCode.set_start(code=code)

            # Reset the mask
            await self.remote.cmd_resetEnabledFaultsMask.set_start()

            self.assertEqual(len(self.csc._error_codes_bypass), 0)

    async def test_setConfigurationFile(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # This should fail in the Standby state
            configuration_file = (
                "Configurable_File_Description_20180831T091922_M2_optical.csv"
            )
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_setConfigurationFile.set_start(
                    file=configuration_file
                )

            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # This should fail for the wrong file
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_setConfigurationFile.set_start(file="abc")
            self.assertFalse(self.csc._is_overwritten_configuration_file)

            # Set the correct configuraion file
            await self.remote.cmd_setConfigurationFile.set_start(
                file=configuration_file
            )
            self.assertTrue(self.csc._is_overwritten_configuration_file)

            # Check the related event
            await asyncio.sleep(SLEEP_TIME_SHORT)

            data = self.remote.evt_config.get()
            self.assertEqual(data.version, "20180831T091922")
            self.assertEqual(
                data.controlParameters, "CtrlParameterFiles_2018-07-19_104257_m2"
            )

    async def test_enableOpenLoopMaxLimit(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)
            self._assert_open_loop_max_limit(False)

            # This should fail in the closed-loop control
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_enableOpenLoopMaxLimit.set_start(status=True)

            # Transition to the open-loop control
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            # Enable the max limit
            await self.remote.cmd_enableOpenLoopMaxLimit.set_start(status=True)

            # Check the related event
            await asyncio.sleep(SLEEP_TIME_LONG)
            self._assert_open_loop_max_limit(True)

    async def test_moveActuator(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # This should fail in the closed-loop control
            actuator = 1
            step = 1000
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_moveActuator.set_start(
                    actuator=actuator, step=step
                )

            # Switch the force balance system off
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            # This should fail because both of the step and displacement are
            # not zero.
            displacement = 60
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_moveActuator.set_start(
                    actuator=actuator, displacement=displacement, step=step
                )

            # Move the actuator displacement
            position_before = self._get_axial_actuator_position(actuator)

            await self.remote.cmd_moveActuator.set_start(
                actuator=actuator, displacement=displacement
            )

            await asyncio.sleep(SLEEP_TIME_LONG)
            position_after = self._get_axial_actuator_position(actuator)

            self.assertGreater((position_after - position_before), 30)

            # Move the actuator step
            step_before = self._get_axial_actuator_step(actuator)

            await self.remote.cmd_moveActuator.set_start(actuator=actuator, step=step)

            await asyncio.sleep(SLEEP_TIME_LONG)
            step_after = self._get_axial_actuator_step(actuator)

            self.assertEqual((step_after - step_before), step)

    def _get_axial_actuator_position(self, actuator: int) -> float:
        data = self.remote.tel_axialEncoderPositions.get()
        return data.position[actuator]

    def _get_axial_actuator_step(self, actuator: int) -> int:
        data = self.remote.tel_axialActuatorSteps.get()
        return data.steps[actuator]

    async def test_resetActuatorSteps(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # This should fail in the closed-loop control
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_resetActuatorSteps.set_start()

            # Switch the force balance system off
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            # Move the actuator step and reset
            actuator = 1
            step_before = self._get_axial_actuator_step(actuator)

            step = 10000
            await self.remote.cmd_moveActuator.set_start(actuator=actuator, step=step)
            await self.remote.cmd_resetActuatorSteps.set_start()

            step_after = self._get_axial_actuator_step(actuator)

            self.assertLess((step_after - step_before), step)

    async def test_actuatorBumpTest(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # Axial actuator
            force = 5.26
            period = 3.0
            await self.remote.cmd_actuatorBumpTest.set_start(
                actuator=1, force=force, period=period
            )

            # Check the events
            await self._check_events_bump_test(1)

            # Tangent link
            await self.remote.cmd_actuatorBumpTest.set_start(
                actuator=73, force=force, period=period
            )

            # Check the event
            await self._check_events_bump_test(73)

    async def _check_events_bump_test(self, actuator: int) -> None:

        for status in [
            MTM2.BumpTest.TESTINGPOSITIVE,
            MTM2.BumpTest.TESTINGPOSITIVEWAIT,
            MTM2.BumpTest.TESTINGNEGATIVE,
            MTM2.BumpTest.TESTINGNEGATIVEWAIT,
            MTM2.BumpTest.PASSED,
        ]:
            await self.assert_next_sample(
                self.remote.evt_actuatorBumpTestStatus,
                timeout=STD_TIMEOUT,
                actuator=actuator,
                status=status,
            )

    async def test_actuatorBumpTest_running_error(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # Run the bump test
            force = 5.26
            period = 3.0
            await self.remote.cmd_actuatorBumpTest.set_start(
                actuator=1, force=force, period=period
            )

            await asyncio.sleep(1.0)

            # This should fail because there is a running bump test now
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_actuatorBumpTest.set_start(
                    actuator=2, force=force, period=period
                )

            # Kill the running bump test
            self.csc._task_bump_test.cancel()
            await self.csc._task_bump_test

    async def test_killActuatorBumpTest(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            await self.remote.cmd_actuatorBumpTest.set_start(
                actuator=1, force=5.26, period=6.0
            )

            await asyncio.sleep(1.0)
            self.assertFalse(self.csc._is_bump_test_done())

            await self.remote.cmd_killActuatorBumpTest.set_start()

            await asyncio.sleep(1.0)
            data_status = self.remote.evt_actuatorBumpTestStatus.get()
            self.assertEqual(data_status.status, MTM2.BumpTest.FAILED)

            self.assertTrue(self.csc._is_bump_test_done())

    async def test_setHardpointList(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.DISABLED)

            # Bad hardpoints
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_setHardpointList.set_start(
                    actuators=[5, 6, 7, 73, 75, 77]
                )
            self.assertFalse(self.csc._is_overwritten_hardpoints)

            # Good hardpoints
            self.remote.evt_hardpointList.flush()
            await self.remote.cmd_setHardpointList.set_start(
                actuators=[3, 13, 23, 73, 75, 77]
            )
            self.assertTrue(self.csc._is_overwritten_hardpoints)

            data_hardpoints = await self.remote.evt_hardpointList.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data_hardpoints.actuators, [4, 14, 24, 74, 76, 78])

    async def test_check_limit_switch(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Trigger the limit switch
            error_handler = self.csc.controller_cell.mock_server.model.error_handler
            error_handler.add_new_limit_switch(1, LimitSwitchType.Retract)
            error_handler.add_new_limit_switch(2, LimitSwitchType.Extend)

            await asyncio.sleep(SLEEP_TIME_LONG)

            # Check the limit switch event
            data_limit_switch_retract = self.remote.evt_limitSwitchRetract.get()
            self.assertEqual(data_limit_switch_retract.actuatorId, 1)

            data_limit_switch_extend = self.remote.evt_limitSwitchExtend.get()
            self.assertEqual(data_limit_switch_extend.actuatorId, 2)

    async def test_check_is_inclinometer_out_of_tma_range(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            self.csc._is_inclinometer_out_of_tma_range = True
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            self.assertFalse(self.csc._is_inclinometer_out_of_tma_range)

            self.csc.controller_cell.mock_server.model.set_inclinometer_angle(89.0)
            await asyncio.sleep(SLEEP_TIME_LONG)

            self.assertTrue(self.csc._is_inclinometer_out_of_tma_range)


if __name__ == "__main__":
    unittest.main()
