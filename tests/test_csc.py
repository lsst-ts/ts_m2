import asyncio
import pathlib
import unittest
import asynctest

import numpy as np

from lsst.ts import salobj
from lsst.ts.m2 import M2
from lsst.ts.idl.enums.MTM2 import InclinationTelemetrySource

TEST_CONFIG_DIR = pathlib.Path(__file__).resolve().parent.joinpath("data", "config")

# Timeout for fast operations (seconds)
STD_TIMEOUT = 10


class TestM2CSC(salobj.BaseCscTestCase, asynctest.TestCase):
    def basic_make_csc(self, initial_state, config_dir, simulation_mode):

        return M2(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
        )

    async def test_standard_state_transitions(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=0
        ):
            await self.check_standard_state_transitions(
                enabled_commands=(
                    "applyForces",
                    "positionMirror",
                    "resetForceOffsets",
                    "selectInclinationSource",
                    "setTemperatureOffset",
                    "switchForceBalanceSystem",
                ),
                skip_commands="clearError",
            )

    async def test_telemetry_loop(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=0
        ):

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            for tel in self.remote.salinfo.telemetry_names:
                with self.subTest(telemetry=tel):
                    await getattr(self.remote, f"tel_{tel}").next(
                        flush=True, timeout=STD_TIMEOUT
                    )

    async def test_applyForces(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=0
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
            set_axial_force = np.zeros(self.csc.n_actuators)
            random_actuators = np.random.randint(
                0,
                self.csc.n_actuators,
                size=np.random.randint(1, self.csc.n_actuators),
            )
            set_axial_force[random_actuators] = self.csc.max_axial_force + 1.0

            with self.assertRaises(
                salobj.base.AckError,
                msg="Axial set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    axial=set_axial_force, timeout=STD_TIMEOUT
                )

            # Check sending tangent forces out of limit
            set_tangent_force = np.zeros(self.csc.n_tangent_actuators)
            random_actuators = np.random.randint(
                0,
                self.csc.n_tangent_actuators,
                size=np.random.randint(1, self.csc.n_tangent_actuators),
            )
            set_tangent_force[random_actuators] = self.csc.max_tangent_force + 1.0

            with self.assertRaises(
                salobj.base.AckError,
                msg="Tangent set points failed to check force limit.",
            ):
                await self.remote.cmd_applyForces.set_start(
                    tangent=set_tangent_force, timeout=STD_TIMEOUT
                )

    async def test_positionMirror(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=0
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
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=0
        ):

            async def mtmount_emulator(elevation):
                async with salobj.Controller("MTMount") as mtmount:

                    mtmount.tel_elevation.set(angleActual=elevation)

                    while mtmount.isopen:
                        mtmount.tel_elevation.put()
                        await asyncio.sleep(1.0)

            elevation = np.random.random() * 60.0 + 20.0
            publish_mtmout_elevation_task = asyncio.create_task(
                mtmount_emulator(elevation)
            )

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

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

            self.assertAlmostEqual(
                np.mean(zenith_angle_values),
                90.0 - elevation,
                int(np.ceil(-np.log10(self.csc.inclinometer_rms))) - 1,
            )

            self.remote.evt_inclinationTelemetrySource.flush()

            await self.remote.cmd_selectInclinationSource.set_start(
                source=InclinationTelemetrySource.MTMOUNT
            )

            incl_source = await self.remote.evt_inclinationTelemetrySource.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertEqual(incl_source.source, InclinationTelemetrySource.MTMOUNT)

            zenith_angle = await self.remote.tel_zenithAngle.next(
                flush=True, timeout=STD_TIMEOUT
            )

            self.assertEqual(zenith_angle.measured, 90.0 - elevation)

            publish_mtmout_elevation_task.cancel()

    async def test_switchForceBalanceSystem(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=0
        ):

            await salobj.set_summary_state(self.remote, salobj.State.ENABLED)

            force_balance_status = await self.remote.evt_forceBalanceSystemStatus.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertFalse(force_balance_status.status)

            self.remote.evt_m2AssemblyInPosition.flush()

            await self.remote.cmd_switchForceBalanceSystem.set_start(status=True)

            force_balance_status = await self.remote.evt_forceBalanceSystemStatus.next(
                flush=False, timeout=STD_TIMEOUT
            )

            self.assertTrue(force_balance_status.status)

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
            self.assertTrue(std_dev < self.csc.force_rms * 2.0)

            # Check tangent forces
            tangent_forces = await self.remote.tel_tangentForce.next(
                flush=True, timeout=STD_TIMEOUT
            )

            force = (
                self.csc.mirror_weight
                * np.sin(np.radians(self.csc.zenith_angle))
                / 4.0
                / np.cos(np.radians(30.0))
            )
            tangent_forces_expected = np.array(
                [0.0, -force, force, 0.0, -force, -force]
            )

            for i, f in enumerate(tangent_forces_expected):
                with self.subTest(test="Tangent actuator force.", actuator=i):
                    self.assertEqual(f, tangent_forces.lutGravity[i])


if __name__ == "__main__":
    unittest.main()
