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
SHORT_TIMEOUT = 1

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

            await self._flush_kafka_topic("evt_commandableByDDS")

            # Change the commander to the GUI
            await self.csc.controller_cell.mock_server._message_event.write_commandable_by_dds(
                False
            )

            await self.assert_next_sample(
                self.remote.evt_commandableByDDS,
                timeout=STD_TIMEOUT,
                state=False,
            )

            self.assertFalse(self.csc.is_csc_commander())

            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_clearErrors.set_start(timeout=STD_TIMEOUT)

    async def _flush_kafka_topic(self, name: str) -> None:

        # Workaround the kafka to discard the topic from an old component
        topic = getattr(self.remote, name)

        try:
            await self.assert_next_sample(topic, flush=False, timeout=SHORT_TIMEOUT)
        except TimeoutError:
            pass

        topic.flush()

    async def test_start(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Flush the event topics
            topics = [
                "summaryState",
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
                await self._flush_kafka_topic(f"evt_{topic}")

            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Check the summary state
            await self.assert_next_summary_state(
                salobj.State.DISABLED, timeout=STD_TIMEOUT
            )

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

            # Check the telemetry

            # Flush the telemetry topics
            topics = [
                "powerStatus",
                "powerStatusRaw",
                "displacementSensors",
            ]
            for topic in topics:
                await self._flush_kafka_topic(f"tel_{topic}")

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
            await self._flush_kafka_topic("evt_summaryState")

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
            await self._flush_kafka_topic("evt_summaryState")

            # Enter the Enabled state first
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            # Flush the topics
            for topic in ("errorCode", "summaryFaultsStatus", "summaryState"):
                await self._flush_kafka_topic(f"evt_{topic}")

            # Make the server fault
            mock_model = self.csc.controller_cell.mock_server.model
            mock_model.fault(MockErrorCode.LimitSwitchTriggeredClosedloop.value)

            # Check the events
            await self.assert_next_summary_state(
                salobj.State.FAULT, timeout=STD_TIMEOUT
            )

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
            await self.assert_next_summary_state(
                salobj.State.STANDBY, timeout=STD_TIMEOUT
            )

            # Check the server fault
            self.assertFalse(mock_model.error_handler.exists_error())

    async def test_enable(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_summaryState")

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
            for topic in ("config", "hardpointList", "summaryState"):
                await self._flush_kafka_topic(f"evt_{topic}")

            # Go to the Enabled state
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            await self.assert_next_summary_state(
                salobj.State.ENABLED, timeout=STD_TIMEOUT
            )

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
            data_power_system_state = self.remote.evt_powerSystemState.get()
            self.assertTrue(data_power_system_state.status)

            data_inner_loop_control_mode = self.remote.evt_innerLoopControlMode.get()
            self.assertEqual(data_inner_loop_control_mode.address, NUM_ACTUATOR - 1)

    async def test_enable_power_on(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_summaryState")

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

            # Go to the Enabled state
            self.remote.evt_summaryState.flush()

            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            await self.assert_next_summary_state(
                salobj.State.ENABLED, timeout=STD_TIMEOUT
            )

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
            # Go to the Enabled state
            await self._set_summary_state(salobj.State.ENABLED)

            self.remote.evt_summaryState.flush()

            # Trigger the interlock fault
            await self.csc.controller_cell.set_bit_digital_status(
                2, DigitalOutputStatus.BinaryLowLevel
            )

            # The interlock event should transition the system to Fault state
            await self.assert_next_summary_state(
                salobj.State.FAULT, timeout=STD_TIMEOUT
            )

    async def _set_summary_state(self, state: salobj.State) -> None:

        # Workaround the kafka to discard the topic from an old component
        await self._flush_kafka_topic("evt_summaryState")

        await salobj.set_summary_state(self.remote, state)

    async def test_disable(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_summaryState")

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

    async def test_check_standard_state_transitions(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # BaseCscTestCase.check_standard_state_transitions() can not be
            # used here because I need time to let the connection to be
            # constructed

            await self._flush_kafka_topic("evt_summaryState")

            self.assertFalse(self.csc.system_is_ready)

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

    async def test_connection_multiple_times(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_summaryState")

            # First time of connection

            # Enter the Enabled state to construct the connection and make
            # sure I can get the controller's state event
            await self._set_summary_state(salobj.State.ENABLED)
            self.assertTrue(self.csc.controller_cell.are_clients_connected())
            self.assertTrue(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # Check the last sequence ID
            self.assertEqual(
                self.csc.controller_cell.client_command.last_sequence_id, 21
            )

            # Enter the Standby state to close the connection
            await self._set_summary_state(salobj.State.STANDBY)
            self.assertFalse(self.csc.controller_cell.are_clients_connected())
            self.assertFalse(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # Second time of connection

            # Enter the Enabled state to construct the connection and make
            # sure I can get the controller's state event
            await self._set_summary_state(salobj.State.ENABLED)
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
            await self._set_summary_state(salobj.State.ENABLED)

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
            await self._set_summary_state(salobj.State.DISABLED)
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
            await self._set_summary_state(salobj.State.ENABLED)
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
            await self._flush_kafka_topic("evt_m2AssemblyInPosition")
            await self._set_summary_state(salobj.State.ENABLED)

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

            # Apply the force
            n_axial_actuators = NUM_ACTUATOR - NUM_TANGENT_LINK
            axial = [0.0] * n_axial_actuators
            axial[0] = 1.0

            tangent = [0.0] * NUM_TANGENT_LINK
            tangent[0] = 2.0

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

            await asyncio.sleep(SLEEP_TIME_MEDIUM)

            await self._flush_kafka_topic("tel_tangentForce")
            await self._flush_kafka_topic("tel_axialForce")

            tangent_forces = await self.remote.tel_tangentForce.next(
                flush=True, timeout=STD_TIMEOUT
            )
            axial_forces = await self.remote.tel_axialForce.next(
                flush=True, timeout=STD_TIMEOUT
            )

            self.assertEqual(tangent_forces.applied[0], 2.0)
            self.assertEqual(axial_forces.applied[0], 1.0)

            await self.remote.cmd_resetForceOffsets.start(timeout=STD_TIMEOUT)

            await asyncio.sleep(SLEEP_TIME_MEDIUM)

            self.remote.tel_tangentForce.flush()
            self.remote.tel_axialForce.flush()

            tangent_forces = await self.remote.tel_tangentForce.next(flush=True)
            axial_forces = await self.remote.tel_axialForce.next(flush=True)

            self.assertEqual(tangent_forces.applied[0], 0.0)
            self.assertEqual(axial_forces.applied[0], 0.0)

            # Check sending axial forces out of limit
            mock_model = self.csc.controller_cell.mock_server.model

            set_axial_force = np.zeros(n_axial_actuators)
            set_axial_force[0] = LIMIT_FORCE_AXIAL_CLOSED_LOOP

            with self.assertRaises(
                salobj.AckError,
                msg="Axial set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    axial=set_axial_force, timeout=STD_TIMEOUT
                )

            # Check sending tangent forces out of limit
            set_tangent_force = np.zeros(NUM_TANGENT_LINK)
            set_tangent_force[0] = LIMIT_FORCE_TANGENT_CLOSED_LOOP

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
            await self._set_summary_state(salobj.State.ENABLED)

            await self._flush_kafka_topic("tel_tangentForce")
            await self._flush_kafka_topic("tel_axialForce")

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
            await self._flush_kafka_topic("evt_m2AssemblyInPosition")
            await self._set_summary_state(salobj.State.ENABLED)

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
                "x": 1.0,
                "y": 2.0,
                "z": 3.0,
            }

            await self.remote.cmd_positionMirror.set_start(
                **position_send, timeout=STD_TIMEOUT
            )

            # Wait for some time for the movement to be done
            await asyncio.sleep(SLEEP_TIME_LONG)

            # Check the new position
            await self._flush_kafka_topic("tel_position")

            tolerance = 0.5

            position_set = await self.remote.tel_position.next(
                flush=False, timeout=STD_TIMEOUT
            )
            for axis in ("x", "y", "z"):
                with self.subTest(telemetry="position", axis=axis):
                    self.assertLess(
                        abs(getattr(position_set, axis) - position_send[axis]),
                        tolerance,
                    )

    async def test_selectInclinationSource(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_inclinationTelemetrySource")
            await self._set_summary_state(salobj.State.ENABLED)

            # Simulate the MTMount CSC
            elevation = np.random.random() * 60.0 + 20.0
            await self._simulate_csc_mount(elevation)

            await self.assert_next_sample(
                self.remote.evt_inclinationTelemetrySource,
                timeout=STD_TIMEOUT,
                source=MTM2.InclinationTelemetrySource.ONBOARD,
            )

            await self._flush_kafka_topic("tel_zenithAngle")

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
            await self._set_summary_state(salobj.State.DISABLED)
            await self.remote.cmd_selectInclinationSource.set_start(
                source=MTM2.InclinationTelemetrySource.MTMOUNT
            )

            await self._set_summary_state(salobj.State.ENABLED)

            await self.assert_next_sample(
                self.remote.evt_inclinationTelemetrySource,
                timeout=STD_TIMEOUT,
                source=MTM2.InclinationTelemetrySource.MTMOUNT,
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
            await self._set_summary_state(salobj.State.ENABLED)

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
            await self._set_summary_state(salobj.State.DISABLED)
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
            await self._flush_kafka_topic("evt_forceBalanceSystemStatus")
            await self._set_summary_state(salobj.State.ENABLED)

            # From the welcome messages
            await self.assert_next_sample(
                self.remote.evt_forceBalanceSystemStatus,
                timeout=STD_TIMEOUT,
                status=False,
            )

            # The default force balance system is on when controlled by SAL
            await self.assert_next_sample(
                self.remote.evt_forceBalanceSystemStatus,
                timeout=STD_TIMEOUT,
                status=True,
            )

            # Switch the force balance system off
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            await self.assert_next_sample(
                self.remote.evt_forceBalanceSystemStatus,
                timeout=STD_TIMEOUT,
                status=False,
            )

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
            await self._flush_kafka_topic("tel_axialForce")
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
            await self._flush_kafka_topic("evt_summaryState")

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

    async def test_setTemperatureOffset(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_temperatureOffset")
            await self._set_summary_state(salobj.State.DISABLED)

            self.remote.evt_temperatureOffset.flush()

            # Change the offset successfully
            ring = [19.0] * NUM_TEMPERATURE_RING
            intake = [19.0] * NUM_TEMPERATURE_INTAKE
            exhaust = [19.0] * NUM_TEMPERATURE_EXHAUST
            await self.remote.cmd_setTemperatureOffset.set_start(
                ring=ring, intake=intake, exhaust=exhaust
            )

            await self.assert_next_sample(
                self.remote.evt_temperatureOffset,
                timeout=STD_TIMEOUT,
                ring=ring,
                intake=[0.0] * NUM_TEMPERATURE_INTAKE,
                exhaust=[0.0] * NUM_TEMPERATURE_EXHAUST,
            )

    async def test_bypassErrorCode(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_summaryState")
            await self._flush_kafka_topic("evt_enabledFaultsMask")

            # This should fail in the Standby state
            code = 6077
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_bypassErrorCode.set_start(code=code)

            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Check the default enabled faults mask
            await self.assert_next_sample(
                self.remote.evt_enabledFaultsMask,
                timeout=STD_TIMEOUT,
                mask=hex(DEFAULT_ENABLED_FAULTS_MASK),
            )

            # Unexisted error code should fail
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_bypassErrorCode.set_start(code=12345)

            # Bypass the error code
            await self.remote.cmd_bypassErrorCode.set_start(code=code)

            self.assertEqual(self.csc._error_codes_bypass, {code})

            # Enter the Enabled state to check the mask
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            await self.assert_next_sample(
                self.remote.evt_enabledFaultsMask,
                timeout=STD_TIMEOUT,
                mask=hex(0xFF000003FFFFFFF8),
            )

    async def test_resetEnabledFaultsMask(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # This should fail in the Standby state
            code = 6077
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_bypassErrorCode.set_start(code=code)

            # Enter the Disabled state to construct the connection
            await self._flush_kafka_topic("evt_summaryState")
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
            await self._flush_kafka_topic("evt_summaryState")
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # This should fail for the wrong file
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_setConfigurationFile.set_start(file="abc")
            self.assertFalse(self.csc._is_overwritten_configuration_file)

            # Set the correct configuraion file
            await self._flush_kafka_topic("evt_config")

            await self.remote.cmd_setConfigurationFile.set_start(
                file=configuration_file
            )
            self.assertTrue(self.csc._is_overwritten_configuration_file)

            # Check the related event
            await self.assert_next_sample(
                self.remote.evt_config,
                timeout=STD_TIMEOUT,
                version="20180831T091922",
                controlParameters="CtrlParameterFiles_2018-07-19_104257_m2",
            )

    async def test_enableOpenLoopMaxLimit(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_openLoopMaxLimit")
            await self._set_summary_state(salobj.State.ENABLED)

            await self.assert_next_sample(
                self.remote.evt_openLoopMaxLimit,
                timeout=STD_TIMEOUT,
                status=False,
            )

            # This should fail in the closed-loop control
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_enableOpenLoopMaxLimit.set_start(status=True)

            # Transition to the open-loop control
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            # Enable the max limit
            await self.remote.cmd_enableOpenLoopMaxLimit.set_start(status=True)

            # Check the related event
            await self.assert_next_sample(
                self.remote.evt_openLoopMaxLimit,
                timeout=STD_TIMEOUT,
                status=True,
            )

    async def test_moveActuator(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._set_summary_state(salobj.State.ENABLED)

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
            displacement = 6
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_moveActuator.set_start(
                    actuator=actuator, displacement=displacement, step=step
                )

            # Move the actuator displacement
            position_before = await self._get_axial_actuator_position(actuator)

            await self.remote.cmd_moveActuator.set_start(
                actuator=actuator, displacement=displacement
            )

            await asyncio.sleep(SLEEP_TIME_LONG)
            position_after = await self._get_axial_actuator_position(actuator)

            self.assertGreater((position_after - position_before), 3)

            # Move the actuator step
            step_before = await self._get_axial_actuator_step(actuator)

            await self.remote.cmd_moveActuator.set_start(actuator=actuator, step=step)

            await asyncio.sleep(SLEEP_TIME_LONG)
            step_after = await self._get_axial_actuator_step(actuator)

            self.assertEqual((step_after - step_before), step)

    async def _get_axial_actuator_position(self, actuator: int) -> float:
        await self._flush_kafka_topic("tel_axialEncoderPositions")

        data = await self.remote.tel_axialEncoderPositions.next(
            flush=True, timeout=STD_TIMEOUT
        )
        return data.position[actuator]

    async def _get_axial_actuator_step(self, actuator: int) -> int:
        await self._flush_kafka_topic("tel_axialActuatorSteps")

        data = await self.remote.tel_axialActuatorSteps.next(
            flush=True, timeout=STD_TIMEOUT
        )
        return data.steps[actuator]

    async def test_resetActuatorSteps(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._set_summary_state(salobj.State.ENABLED)

            # This should fail in the closed-loop control
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_resetActuatorSteps.set_start()

            # Switch the force balance system off
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            # Move the actuator step and reset
            actuator = 1
            step_before = await self._get_axial_actuator_step(actuator)

            step = 10000
            await self.remote.cmd_moveActuator.set_start(actuator=actuator, step=step)
            await self.remote.cmd_resetActuatorSteps.set_start()

            step_after = await self._get_axial_actuator_step(actuator)

            self.assertLess((step_after - step_before), step)

    async def test_actuatorBumpTest(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._set_summary_state(salobj.State.ENABLED)
            await self._flush_kafka_topic("evt_actuatorBumpTestStatus")

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
            await self._set_summary_state(salobj.State.ENABLED)

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
            await self._set_summary_state(salobj.State.ENABLED)

            await self.remote.cmd_actuatorBumpTest.set_start(
                actuator=1, force=5.26, period=30.0
            )

            await asyncio.sleep(1.0)
            self.assertFalse(self.csc._is_bump_test_done())

            await self._flush_kafka_topic("evt_actuatorBumpTestStatus")

            await self.remote.cmd_killActuatorBumpTest.set_start()

            await self.assert_next_sample(
                self.remote.evt_actuatorBumpTestStatus,
                timeout=STD_TIMEOUT,
                status=MTM2.BumpTest.FAILED_NONTESTEDPROBLEM,
            )

            self.assertTrue(self.csc._is_bump_test_done())

    async def test_setHardpointList(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_summaryState")
            await self._set_summary_state(salobj.State.DISABLED)

            # Bad hardpoints
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_setHardpointList.set_start(
                    actuators=[5, 6, 7, 73, 75, 77]
                )
            self.assertFalse(self.csc._is_overwritten_hardpoints)

            # Good hardpoints
            await self._flush_kafka_topic("evt_hardpointList")
            await self.remote.cmd_setHardpointList.set_start(
                actuators=[3, 13, 23, 73, 75, 77]
            )
            self.assertTrue(self.csc._is_overwritten_hardpoints)

            await self.assert_next_sample(
                self.remote.evt_hardpointList,
                timeout=STD_TIMEOUT,
                actuators=[4, 14, 24, 74, 76, 78],
            )

    async def test_check_limit_switch(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self._flush_kafka_topic("evt_summaryState")

            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Trigger the limit switch
            error_handler = self.csc.controller_cell.mock_server.model.error_handler
            error_handler.add_new_limit_switch(1, LimitSwitchType.Retract)
            error_handler.add_new_limit_switch(2, LimitSwitchType.Extend)

            # Check the limit switch event
            await self.assert_next_sample(
                self.remote.evt_limitSwitchRetract,
                timeout=STD_TIMEOUT,
                actuatorId=1,
            )
            await self.assert_next_sample(
                self.remote.evt_limitSwitchExtend,
                timeout=STD_TIMEOUT,
                actuatorId=2,
            )

    async def test_check_is_inclinometer_out_of_tma_range(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            self.csc._is_inclinometer_out_of_tma_range = True
            await self._set_summary_state(salobj.State.ENABLED)

            self.assertFalse(self.csc._is_inclinometer_out_of_tma_range)

            self.csc.controller_cell.mock_server.model.set_inclinometer_angle(89.0)

            num_to_poke = int(SLEEP_TIME_LONG)
            num = 0
            for num in range(num_to_poke):
                if self.csc._is_inclinometer_out_of_tma_range:
                    break
                else:
                    await asyncio.sleep(1.0)

            self.assertLess(num, num_to_poke - 1)


if __name__ == "__main__":
    unittest.main()
