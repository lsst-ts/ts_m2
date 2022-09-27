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
from lsst.ts.idl.enums.MTM2 import InclinationTelemetrySource
from lsst.ts.m2 import M2
from lsst.ts.m2com import NUM_ACTUATOR, NUM_TANGENT_LINK, DetailedState

# Timeout for fast operations (seconds)
STD_TIMEOUT = 10


class TestM2CSC(salobj.BaseCscTestCase, unittest.IsolatedAsyncioTestCase):
    def basic_make_csc(self, initial_state, config_dir, simulation_mode):
        return M2(
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

    def setUp(self):
        super().setUp()
        self.csc_mtmount = None

    async def _simulate_csc_mount(self, elevation_angle):
        """Simulate the MTMount CSC.

        Parameters
        ----------
        elevation_angle : `float`
            Elevation angle in degree.
        """

        self.csc_mtmount = salobj.Controller("MTMount")
        await self.csc_mtmount.start_task

        await self.csc_mtmount.tel_elevation.set_write(actualPosition=elevation_angle)
        await self.csc_mtmount.evt_elevationInPosition.set_write(inPosition=True)

        # Wait for some time to publish the telemetry of MTMount
        await asyncio.sleep(1)

    async def asyncTearDown(self) -> None:
        if self.csc_mtmount is not None:
            await self.csc_mtmount.close()

    async def test_bin_script(self):
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

    async def test_instantiation_m2_normal_mode(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=0
        ):

            self.assertIsNone(self.csc.controller_cell.mock_server)
            self.assertFalse(self.csc.controller_cell.are_clients_connected())

    async def test_start(self):
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
            await asyncio.sleep(4)

            # Check the TCP/IP connection is on
            self.assertIsNotNone(self.csc.controller_cell.mock_server)
            self.assertTrue(self.csc.controller_cell.are_clients_connected())

            # Simulate the MTMount CSC
            elevation_angle = 10
            await self._simulate_csc_mount(elevation_angle)

            # There should be no update of inclinometer angle from MTMount CSC
            mock_model = self.csc.controller_cell.mock_server.model
            self.assertEqual(mock_model.control_open_loop.inclinometer_angle, 90)

            # Check the update of mount is in position or not from MTMount CSC
            self.assertEqual(mock_model.mtmount_in_position, True)

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
                data_inclination_src.source, int(InclinationTelemetrySource.ONBOARD)
            )

            data_temp_offset = await self.remote.evt_temperatureOffset.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data_temp_offset.ring, [21] * 12)
            self.assertEqual(data_temp_offset.intake, [21] * 2)
            self.assertEqual(data_temp_offset.exhaust, [21] * 2)

            data_detailed_state_1 = await self.remote.evt_detailedState.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(
                data_detailed_state_1.detailedState, DetailedState.PublishOnly
            )

            data_detailed_state_2 = await self.remote.evt_detailedState.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(
                data_detailed_state_2.detailedState, DetailedState.Available
            )

            data_controller_state = await self.remote.evt_controllerState.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(
                data_controller_state.controllerState, int(salobj.State.OFFLINE)
            )

            # Check the telemetry in OFFLINE state
            data_power_status = await self.remote.tel_powerStatus.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertLess(abs(data_power_status.motorVoltage), 1)
            self.assertLess(abs(data_power_status.motorCurrent), 1)
            self.assertLess(abs(data_power_status.commVoltage), 1)
            self.assertLess(abs(data_power_status.commCurrent), 1)

            data_disp_sensors = await self.remote.tel_displacementSensors.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertNotEqual(data_disp_sensors.thetaZ, [0] * 6)
            self.assertNotEqual(data_disp_sensors.deltaZ, [0] * 6)

    async def test_standby_no_fault(self):
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

    async def test_standby_fault(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Make the server fault
            mock_model = self.csc.controller_cell.mock_server.model
            mock_model.fault()
            await asyncio.sleep(1)

            self.assertEqual(self.csc.summary_state, salobj.State.FAULT)

            # Do the standby to disconnect the server
            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)
            self.assertFalse(
                self.csc.controller_cell.mock_server.are_servers_connected()
            )

            # Check the summary state
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

            # Check the server fault
            self.assertTrue(mock_model.error_cleared)

    async def test_standby_fault_accidental(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Put the csc to the Fault state
            await self.csc.fault(code=0, report="test fault")

            # Transition to the Standby state
            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)

            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

    async def test_enable(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Go to the Enabled state
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            # Check the event of controller's state
            data_controller_state = await self.remote.evt_controllerState.aget(
                timeout=STD_TIMEOUT
            )
            self.assertEqual(
                data_controller_state.controllerState, int(salobj.State.ENABLED)
            )

    async def test_disable(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Go to the Enabled state
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            # Go to the Disabled state
            await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)

            # Check the event of controller's state
            data_controller_state = await self.remote.evt_controllerState.aget(
                timeout=STD_TIMEOUT
            )
            self.assertEqual(
                data_controller_state.controllerState, int(salobj.State.OFFLINE)
            )

    async def test_command_fail_wrong_state(self):
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
            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)

    async def test_check_standard_state_transitions(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # BaseCscTestCase.check_standard_state_transitions() can not be
            # used here because I need time to let the connection to be
            # constructed

            await self.assert_next_summary_state(
                salobj.State.STANDBY, timeout=STD_TIMEOUT
            )

            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.DISABLED, timeout=STD_TIMEOUT
            )

            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.ENABLED, timeout=STD_TIMEOUT
            )

            await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.DISABLED, timeout=STD_TIMEOUT
            )

            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.STANDBY, timeout=STD_TIMEOUT
            )

    async def test_set_summary_state(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            self.assertEqual(self.csc.summary_state, salobj.State.ENABLED)

    async def test_connection_multiple_times(self):
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
                self.csc.controller_cell.client_command.last_sequence_id, 3
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

            # Check the last sequence ID. Note the value should be 9 instead of
            # 3 from the previous connection.
            self.assertEqual(
                self.csc.controller_cell.client_command.last_sequence_id, 9
            )

    async def test_telemetry_loop(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            for tel in self.remote.salinfo.telemetry_names:
                with self.subTest(telemetry=tel):
                    await getattr(self.remote, f"tel_{tel}").next(
                        flush=True, timeout=STD_TIMEOUT
                    )

    async def test_connection_monitor_loop(self):
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

    async def test_applyForces_wrong_controller_state(self):
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

    async def test_applyForces(self):
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
            set_axial_force[0] = mock_model.control_closed_loop.LIMIT_FORCE_AXIAL + 1.0

            with self.assertRaises(
                salobj.AckError,
                msg="Axial set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    axial=set_axial_force, timeout=STD_TIMEOUT
                )

            # Check sending tangent forces out of limit
            set_tangent_force = np.zeros(NUM_TANGENT_LINK)
            set_tangent_force[0] = (
                mock_model.control_closed_loop.LIMIT_FORCE_TANGENT + 1.0
            )

            with self.assertRaises(
                salobj.AckError,
                msg="Tangent set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    tangent=set_tangent_force, timeout=STD_TIMEOUT
                )

    async def test_positionMirror(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            position_send = dict(
                [
                    (axis, np.random.normal())
                    for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
                ]
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

            # m2AssemblyInPosition is now in position, ready to reposition it.
            self.remote.evt_m2AssemblyInPosition.flush()

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

            position_set = await self.remote.tel_position.next(
                flush=True, timeout=STD_TIMEOUT
            )

            for axis in ("x", "y", "z", "xRot", "yRot", "zRot"):
                with self.subTest(telemetry="position", axis=axis):
                    self.assertAlmostEqual(
                        getattr(position_set, axis),
                        position_send[axis],
                        1,
                        f"Position axis {axis}: position sent ({position_send[axis]}) "
                        f"different than received {getattr(position_set, axis)}",
                    )

            position_ims_set = await self.remote.tel_positionIMS.next(
                flush=True, timeout=STD_TIMEOUT
            )

            for axis in ("x", "y", "z", "xRot", "yRot", "zRot"):
                with self.subTest(telemetry="positionIMS", axis=axis):
                    self.assertAlmostEqual(
                        getattr(position_set, axis),
                        position_send[axis],
                        1,
                        f"PositionIMS axis {axis}: position sent ({position_send[axis]}) "
                        f"different than received {getattr(position_ims_set, axis)}",
                    )

    async def test_selectInclinationSource(self):
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

            self.assertEqual(incl_source.source, InclinationTelemetrySource.ONBOARD)

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
            await self.remote.cmd_selectInclinationSource.set_start(
                source=InclinationTelemetrySource.MTMOUNT
            )

            incl_source = await self.remote.evt_inclinationTelemetrySource.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertEqual(incl_source.source, InclinationTelemetrySource.MTMOUNT)

            await self.csc_mtmount.tel_elevation.set_write(actualPosition=elevation + 1)
            await asyncio.sleep(2)

            zenith_angle = self.remote.tel_zenithAngle.get()

            # Need to consider the internal random variable in the simulation
            # of zenith angle in mock model.
            self.assertLess(abs(zenith_angle.inclinometerRaw - elevation - 1), 3)

    async def test_switchForceBalanceSystem(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # The default force balance system is on when controlled by SAL
            force_balance_status = await self.remote.evt_forceBalanceSystemStatus.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertTrue(force_balance_status.status)

            # Switch the force balance system off
            await self.remote.cmd_switchForceBalanceSystem.set_start(status=False)

            force_balance_status = await self.remote.evt_forceBalanceSystemStatus.next(
                flush=False, timeout=STD_TIMEOUT
            )

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

    async def test_clearErrors(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            # Enter the Disabled state to construct the connection
            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

            # Fault the mock model and wait for some time to get the state
            # event
            mock_model = self.csc.controller_cell.mock_server.model
            mock_model.fault()
            await asyncio.sleep(1)

            self.assertEqual(
                self.csc.controller_cell.controller_state, salobj.State.FAULT
            )

            data_controller_state = await self.remote.evt_controllerState.aget(
                timeout=STD_TIMEOUT
            )
            self.assertEqual(
                data_controller_state.controllerState, int(salobj.State.FAULT)
            )

            # Clear the error
            await self.remote.cmd_clearErrors.set_start(timeout=STD_TIMEOUT)

            self.assertTrue(mock_model.error_cleared)
            await asyncio.sleep(1)

            self.assertEqual(
                self.csc.controller_cell.controller_state, salobj.State.OFFLINE
            )

    async def test_setTemperatureOffset(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # Change the offset successfully
            ring = [19] * 12
            intake = [19] * 2
            exhaust = [19] * 2
            await self.remote.cmd_setTemperatureOffset.set_start(
                ring=ring, intake=intake, exhaust=exhaust
            )

            data_temp_offset = await self.remote.evt_temperatureOffset.aget(
                timeout=STD_TIMEOUT
            )
            np.testing.assert_array_equal(data_temp_offset.ring, ring)
            np.testing.assert_array_equal(data_temp_offset.intake, intake)
            np.testing.assert_array_equal(data_temp_offset.exhaust, exhaust)

            # Fail to change the offset
            with self.assertRaises(salobj.AckError):
                exhaust = [18] * 2
                await self.remote.cmd_setTemperatureOffset.set_start(
                    ring=ring, intake=intake, exhaust=exhaust
                )


if __name__ == "__main__":
    unittest.main()
