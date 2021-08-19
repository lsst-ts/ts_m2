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
import asynctest

import numpy as np

from lsst.ts import salobj
from lsst.ts.m2 import M2, DetailedState
from lsst.ts.idl.enums.MTM2 import InclinationTelemetrySource

# Timeout for fast operations (seconds)
STD_TIMEOUT = 10


class TestM2CSC(salobj.BaseCscTestCase, asynctest.TestCase):
    def basic_make_csc(self, initial_state, config_dir, simulation_mode):
        return M2(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
        )

    def setUp(self):
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

        self.csc_mtmount.tel_elevation.set_put(actualPosition=elevation_angle)

        # Wait for some time to publish the telemetry of MTMount
        await asyncio.sleep(2)

    async def tearDown(self):
        if self.csc_mtmount is not None:
            await self.csc_mtmount.close()

    async def test_bin_script(self):
        await self.check_bin_script(
            name="MTM2",
            index=None,
            exe_name="run_mtm2.py",
            default_initial_state=salobj.State.OFFLINE,
        )

        await self.check_bin_script(
            name="MTM2",
            index=None,
            exe_name="run_mtm2.py",
            default_initial_state=salobj.State.OFFLINE,
            cmdline_args=["--host", "127.0.0.1", "--ports", "0", "1"],
        )

    async def test_instantiation_m2_normal_mode(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=0
        ):

            self.assertIsNone(self.csc._mock_server)
            self.assertFalse(self.csc.model.are_clients_connected())

    async def test_instantiation_m2_simulation_mode(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):

            # Simulate the MTMount CSC
            elevation_angle = 10
            await self._simulate_csc_mount(elevation_angle)

            # Check the TCP/IP connection is on
            self.assertIsNotNone(self.csc._mock_server)
            self.assertTrue(self.csc.model.are_clients_connected())

            # Check the update of zenith angle from MTMount CSC
            mock_model = self.csc._mock_server.model
            self.assertEqual(mock_model.zenith_angle, 90 - elevation_angle)

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

            data_summary_state = await self.remote.evt_summaryState.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data_summary_state.summaryState, int(salobj.State.OFFLINE))

            # Check the telemetry in OFFLINE state
            data_power_status = await self.remote.tel_powerStatus.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertLess(abs(data_power_status.motorVoltage), 1)
            self.assertLess(abs(data_power_status.motorCurrent), 1)
            self.assertGreater(abs(data_power_status.commVoltage), 20)
            self.assertGreater(abs(data_power_status.commCurrent), 5)

            data_disp_sensors = await self.remote.tel_displacementSensors.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertNotEqual(data_disp_sensors.thetaZ, [0] * 6)
            self.assertNotEqual(data_disp_sensors.deltaZ, [0] * 6)

    async def test_command_fail_wrong_state(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):

            with self.assertRaises(salobj.AckError):
                await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)

    async def test_enterControl(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):

            await self.assert_next_summary_state(
                salobj.State.OFFLINE, timeout=STD_TIMEOUT
            )

            await self.remote.cmd_enterControl.set_start(timeout=STD_TIMEOUT)

            # Check the summary state
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)

            await self.assert_next_summary_state(
                salobj.State.STANDBY, timeout=STD_TIMEOUT
            )

    async def test_check_standard_state_transitions(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):
            # BaseCscTestCase.check_standard_state_transitions() can not be
            # used here because I need to test the enterControl() as well.

            await self.assert_next_summary_state(
                salobj.State.OFFLINE, timeout=STD_TIMEOUT
            )

            await self.remote.cmd_enterControl.set_start(timeout=STD_TIMEOUT)
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

            await self.remote.cmd_exitControl.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(
                salobj.State.OFFLINE, timeout=STD_TIMEOUT
            )

    async def test_set_summary_state(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):

            self.assertEqual(self.csc.summary_state, salobj.State.OFFLINE)

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            self.assertEqual(self.csc.summary_state, salobj.State.ENABLED)

    async def test_telemetry_loop(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            for tel in self.remote.salinfo.telemetry_names:
                with self.subTest(telemetry=tel):
                    await getattr(self.remote, f"tel_{tel}").next(
                        flush=True, timeout=STD_TIMEOUT
                    )

    async def test_applyForces(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
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

            mock_model = self.csc._mock_server.model
            np.testing.assert_array_equal(mock_model.axial_forces["applied"], axial)
            np.testing.assert_array_equal(mock_model.tangent_forces["applied"], tangent)

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

            self.assertEqual(np.sum(np.abs(mock_model.axial_forces["applied"])), 0)
            self.assertEqual(np.sum(np.abs(mock_model.tangent_forces["applied"])), 0)

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
            mock_model = self.csc._mock_server.model
            set_axial_force = np.zeros(mock_model.n_actuators)
            random_actuators = np.random.randint(
                0,
                mock_model.n_actuators,
                size=np.random.randint(1, mock_model.n_actuators),
            )
            set_axial_force[random_actuators] = mock_model.max_axial_force + 1.0

            with self.assertRaises(
                salobj.AckError,
                msg="Axial set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    axial=set_axial_force, timeout=STD_TIMEOUT
                )

            # Check sending tangent forces out of limit
            set_tangent_force = np.zeros(mock_model.n_tangent_actuators)
            random_actuators = np.random.randint(
                0,
                mock_model.n_tangent_actuators,
                size=np.random.randint(1, mock_model.n_tangent_actuators),
            )
            set_tangent_force[random_actuators] = mock_model.max_tangent_force + 1.0

            with self.assertRaises(
                salobj.AckError,
                msg="Tangent set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    tangent=set_tangent_force, timeout=STD_TIMEOUT
                )

    async def test_positionMirror(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
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
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
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

            inclinometer_rms = 0.05
            self.assertAlmostEqual(
                np.mean(zenith_angle_values),
                90.0 - elevation,
                int(np.ceil(-np.log10(inclinometer_rms))) - 1,
            )

            self.remote.evt_inclinationTelemetrySource.flush()

            # Change the source of inclination in DISABLED state
            await self.remote.cmd_disable.set_start(timeout=STD_TIMEOUT)
            await self.remote.cmd_selectInclinationSource.set_start(
                source=InclinationTelemetrySource.MTMOUNT
            )

            incl_source = await self.remote.evt_inclinationTelemetrySource.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertEqual(incl_source.source, InclinationTelemetrySource.MTMOUNT)

            # Transition back to the ENABLED state to get the telemetry of
            # zenith angle
            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)

            zenith_angle = await self.remote.tel_zenithAngle.next(
                flush=True, timeout=STD_TIMEOUT
            )

            self.assertEqual(zenith_angle.measured, 90.0 - elevation)

    async def test_switchForceBalanceSystem(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            # The default force balance system is on when controlled by SAL
            force_balance_status = await self.remote.evt_forceBalanceSystemStatus.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertTrue(force_balance_status.status)

            self.remote.evt_m2AssemblyInPosition.flush()

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

            # Measured value should be lutGravity+lutTemperature with some
            # variation due to introduced noise.
            expected = np.array(axial_forces.lutTemperature) + np.array(
                axial_forces.lutGravity
            )
            measured = np.array(axial_forces.measured)
            std_dev = np.std(measured - expected)
            force_rms = 0.5
            self.assertTrue(std_dev < force_rms * 2.0)

            # Check tangent forces
            tangent_forces = await self.remote.tel_tangentForce.next(
                flush=True, timeout=STD_TIMEOUT
            )

            mock_model = self.csc._mock_server.model
            force = (
                mock_model.mirror_weight
                * np.sin(np.radians(mock_model.zenith_angle))
                / 4.0
                / np.cos(np.radians(30.0))
            )
            tangent_forces_expected = np.array(
                [0.0, -force, force, 0.0, -force, -force]
            )

            for i, f in enumerate(tangent_forces_expected):
                with self.subTest(test="Tangent actuator force.", actuator=i):
                    self.assertEqual(f, tangent_forces.lutGravity[i])

    async def test_clearErrors(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):

            self.assertEqual(self.csc.summary_state, salobj.State.OFFLINE)

            # Fault the mock model and wait for some time to get the state
            # event
            mock_model = self.csc._mock_server.model
            mock_model.fault()
            await asyncio.sleep(1)

            self.assertEqual(self.csc.summary_state, salobj.State.FAULT)

            data_summary_state = await self.remote.evt_summaryState.aget(
                timeout=STD_TIMEOUT
            )
            self.assertEqual(data_summary_state.summaryState, int(salobj.State.FAULT))

            # Clear the error
            await self.remote.cmd_clearErrors.set_start(timeout=STD_TIMEOUT)

            self.assertTrue(mock_model.error_cleared)

            self.assertEqual(self.csc.summary_state, salobj.State.OFFLINE)

            data_summary_state = await self.remote.evt_summaryState.aget(
                timeout=STD_TIMEOUT
            )
            self.assertEqual(data_summary_state.summaryState, int(salobj.State.OFFLINE))

    async def test_setTemperatureOffset(self):
        async with self.make_csc(
            initial_state=salobj.State.OFFLINE, config_dir=None, simulation_mode=1
        ):

            await salobj.set_summary_state(self.remote, salobj.State.DISABLED)

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
