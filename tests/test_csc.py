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

    async def test_start(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
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
            self.assertEqual(mock_model.control_open_loop.inclinometer_angle, 90.0)

            # Check the welcome messages
            data_tcpIp = await self.remote.evt_tcpIpConnected.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertTrue(data_tcpIp.isConnected)

            data_commandable = await self.remote.evt_commandableByDDS.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertTrue(data_commandable.state)

            data_hardpoints = await self.remote.evt_hardpointList.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data_hardpoints.actuators, [6, 16, 26, 74, 76, 78])

            data_interlock = await self.remote.evt_interlock.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data_interlock.state)

            data_inclination_src = (
                await self.remote.evt_inclinationTelemetrySource.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            )
            self.assertEqual(
                data_inclination_src.source,
                int(MTM2.InclinationTelemetrySource.ONBOARD),
            )

            data_temp_offset = await self.remote.evt_temperatureOffset.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data_temp_offset.ring, [21.0] * NUM_TEMPERATURE_RING)
            self.assertEqual(data_temp_offset.intake, [0.0] * NUM_TEMPERATURE_INTAKE)
            self.assertEqual(data_temp_offset.exhaust, [0.0] * NUM_TEMPERATURE_EXHAUST)

            data_digital_output = await self.remote.evt_digitalOutput.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data_digital_output.value, TEST_DIGITAL_OUTPUT_NO_POWER)

            data_digital_input = await self.remote.evt_digitalInput.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data_digital_input.value, hex(TEST_DIGITAL_INPUT_NO_POWER))

            data_config = await self.remote.evt_config.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data_config.cellTemperatureDelta, 2.0)

            data_closed_loop_control_mode = (
                await self.remote.evt_closedLoopControlMode.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            )
            self.assertEqual(
                data_closed_loop_control_mode.mode, MTM2.ClosedLoopControlMode.Idle
            )

            data_enabled_faults_mask = await self.remote.evt_enabledFaultsMask.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(
                data_enabled_faults_mask.mask, hex(DEFAULT_ENABLED_FAULTS_MASK)
            )

            data_configuration_files = await self.remote.evt_configurationFiles.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertNotEqual(data_configuration_files.files, "")

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

    async def test_standby_no_fault(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)
            self.assertTrue(self.csc.controller_cell.are_clients_connected())

            # Do the standby to disconnect the server
            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)
            self.assertFalse(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # Check the summary state
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

    async def test_standby_fault(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Enabled state first
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            # Make the server fault
            mock_model = self.csc.controller_cell.mock_server.model
            mock_model.fault(MockErrorCode.LimitSwitchTriggeredClosedloop.value)
            await asyncio.sleep(SLEEP_TIME_MEDIUM)

            self.assertEqual(self.csc.summary_state, salobj.State.FAULT)

            # Check the events
            data_summary_faults_status = self.remote.evt_summaryFaultsStatus.get()
            self.assertEqual(data_summary_faults_status.status, hex(2**6))

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

            # Go to the Enabled state
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

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
                self.csc.controller_cell.client_command.last_sequence_id, 20
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

            # Check the last sequence ID. Note the value should be 47 instead
            # of 20 from the previous connection.
            self.assertEqual(
                self.csc.controller_cell.client_command.last_sequence_id, 47
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

            # m2AssemblyInPosition is now in position, ready to apply force.
            self.remote.evt_m2AssemblyInPosition.flush()

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

            in_position = (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition

            self.assertFalse(in_position)

            in_position = (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition

            self.assertTrue(in_position)

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

            # Test reset forces now
            self.remote.evt_m2AssemblyInPosition.flush()

            await self.remote.cmd_resetForceOffsets.start(timeout=STD_TIMEOUT)

            in_position = (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition

            self.assertFalse(in_position)

            in_position = (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition

            self.assertTrue(in_position)

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

            # Check we have the force data from simulator or not
            await asyncio.sleep(SLEEP_TIME_SHORT)
            self.assertTrue(self.csc.tel_axialForce.has_data)
            self.assertTrue(self.csc.tel_tangentForce.has_data)

            # Force is out of range
            applied_force_axial = [0] * (NUM_ACTUATOR - NUM_TANGENT_LINK)
            applied_force_tangent = [5000] * NUM_TANGENT_LINK
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

            # m2AssemblyInPosition is now in position, ready to reposition it.
            self.remote.evt_m2AssemblyInPosition.flush()

            # Wait for some time for the closed-loop control to be done
            await asyncio.sleep(STD_TIMEOUT)

            # Move the rigid body to the new position. Note the units are um
            # and arcsec.
            position_send = {
                "x": 100,
                "y": 200,
                "z": 300,
                "xRot": 150,
                "yRot": 250,
                "zRot": 350,
            }
            await self.remote.cmd_positionMirror.set_start(
                **position_send, timeout=STD_TIMEOUT
            )

            in_position = (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition

            self.assertFalse(in_position)

            in_position = (
                await self.remote.evt_m2AssemblyInPosition.next(
                    flush=False, timeout=STD_TIMEOUT
                )
            ).inPosition

            self.assertTrue(in_position)

            # Check the new position
            tolerance = 50

            position_set = self.remote.tel_position.get()
            for axis in ("x", "y", "z", "xRot", "yRot", "zRot"):
                with self.subTest(telemetry="position", axis=axis):
                    self.assertLess(
                        abs(getattr(position_set, axis) - position_send[axis]),
                        tolerance,
                    )

            position_ims_set = self.remote.tel_positionIMS.get()
            for axis in ("x", "y", "z", "xRot", "yRot", "zRot"):
                with self.subTest(telemetry="positionIMS", axis=axis):
                    self.assertLess(
                        abs(getattr(position_ims_set, axis) - position_send[axis]),
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
            inclinometer_rms = 0.05
            self.assertAlmostEqual(
                np.mean(zenith_angle_values),
                0,
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

            # LUT gravity and temperature absolute values should be larger
            # than zero
            self.assertTrue(np.all(np.abs(axial_forces.lutGravity) > 0.0))
            self.assertTrue(np.all(np.abs(axial_forces.lutTemperature) > 0.0))

            # Check the force error
            force_error = self.csc.controller_cell.mock_server.model.control_closed_loop._get_force_error()[
                0
            ]

            force_rms = 0.5
            self.assertLess(np.std(force_error), force_rms * 4.0)

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

            self._assert_enabled_faults_mask(0xFF000003FFFFFFFC)

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

            # Set the correct configuraion file
            await self.remote.cmd_setConfigurationFile.set_start(
                file=configuration_file
            )

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
            displacement = 100
            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_moveActuator.set_start(
                    actuator=actuator, displacement=displacement, step=step
                )

            # Move the actuator displacement
            position_before = self._get_axial_actuator_position(actuator)

            await self.remote.cmd_moveActuator.set_start(
                actuator=actuator, displacement=displacement
            )

            await asyncio.sleep(SLEEP_TIME_SHORT)
            position_after = self._get_axial_actuator_position(actuator)

            self.assertGreater((position_after - position_before), 90)

            # Move the actuator step
            step_before = self._get_axial_actuator_step(actuator)

            await self.remote.cmd_moveActuator.set_start(actuator=actuator, step=step)

            await asyncio.sleep(SLEEP_TIME_SHORT)
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

            # This is to keep the backward compatibility of ts_xml v20.0.0 that
            # does not have the 'actuatorBumpTestStatus' event defined in xml.
            # TODO: Remove this after ts_xml v20.1.0.
            if hasattr(self.remote, "evt_actuatorBumpTestStatus"):
                data_status = await self.remote.evt_actuatorBumpTestStatus.next(
                    flush=False, timeout=STD_TIMEOUT
                )
                self.assertEqual(data_status.actuator, 1)
                self.assertEqual(data_status.status, MTM2.BumpTest.TESTINGPOSITIVE)

            await asyncio.sleep(5 * period)

            # This is to keep the backward compatibility of ts_xml v20.0.0 that
            # does not have the 'actuatorBumpTestStatus' event defined in xml.
            # TODO: Remove this after ts_xml v20.1.0.
            if hasattr(self.remote, "evt_actuatorBumpTestStatus"):
                data_status = self.remote.evt_actuatorBumpTestStatus.get()
                self.assertEqual(data_status.actuator, 1)
                self.assertEqual(data_status.status, MTM2.BumpTest.PASSED)

            # Tangent link
            await self.remote.cmd_actuatorBumpTest.set_start(
                actuator=73, force=force, period=period
            )

            await asyncio.sleep(5 * period)

            # Check the event

            # This is to keep the backward compatibility of ts_xml v20.0.0 that
            # does not have the 'actuatorBumpTestStatus' event defined in xml.
            # TODO: Remove this after ts_xml v20.1.0.
            if hasattr(self.remote, "evt_actuatorBumpTestStatus"):
                data_status = self.remote.evt_actuatorBumpTestStatus.get()
                self.assertEqual(data_status.actuator, 73)
                self.assertEqual(data_status.status, MTM2.BumpTest.PASSED)

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

    async def test_killActuatorBumpTest(self) -> None:
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # This is to keep the backward compatibility of ts_xml v20.0.0 that
            # does not have the 'actuatorBumpTestStatus' event defined in xml.
            # TODO: Remove this after ts_xml v20.1.0.
            if hasattr(self.remote, "cmd_killActuatorBumpTest"):
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


if __name__ == "__main__":
    unittest.main()
