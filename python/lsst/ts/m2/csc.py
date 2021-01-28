import sys
import asyncio
import pathlib
import logging
import argparse
import traceback

import numpy as np
import pandas as pd

from lsst.ts import salobj
from lsst.ts.idl.enums import MTM2

__all__ = ["M2"]

TELEMETRY_LOOP_DIED = 101
"""
Error code for when telemetry loop dies while in ENABLE or DISABLE state.
"""
POSITION_MIRROR_ERROR = 102
"""
Error code for when it fails to reposition the position.
"""


class M2(salobj.ConfigurableCsc):
    """This is a test CSC for the M2 component with salobj.
    """

    def __init__(
        self,
        config_dir=None,
        initial_state=salobj.State.STANDBY,
        simulation_mode=0,
        verbose=False,
    ):
        schema_path = (
            pathlib.Path(__file__).resolve().parents[4].joinpath("schema", "m2.yaml")
        )
        super().__init__(
            "MTM2",
            index=0,
            schema_path=schema_path,
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

        if verbose:
            stream_handler = logging.StreamHandler(sys.stdout)
            self.log.addHandler(stream_handler)
            self.log.setLevel(logging.DEBUG)

        self.error_cleared = True
        self.telemetry_loop_task = None
        self.run_telemetry_loop = False
        self.stop_loop_timeout = 5.0
        self.telemetry_interval = self.heartbeat_interval

        self.inclinometer_rms = 0.05  # inclinometer rms variations in deg
        self.temperature_rms = 0.05  # temperature rms variation in deg_C
        self.force_rms = 0.5  # force rms variation in Newton
        self.position_rms = 0.005  # position rms variation in microns
        self.angle_rms = 0.005  # angle rms variation in arcsec

        self.n_actuators = 72  # number of axial actuators
        self.n_tangent_actuators = 6  # number of tangent actuators
        self.n_ring_temperatures = 12
        self.n_intake_temperatures = 2
        self.n_exhaust_temperatures = 2

        self.mirror_position = dict(
            [(axis, 0.0) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
        )
        self.mirror_position_speed = dict(
            [(axis, 0.5) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
        )
        self.zenith_angle = 0.0
        self.axial_forces = np.zeros(self.n_actuators)
        self.tangent_forces = np.zeros(self.n_tangent_actuators)

        # Coefficients to convert from actuator force to position.
        # position = force_to_position[0] * force + force_to_position[1]
        self.force_to_position = [-0.0011, 0.224]
        self.force_rate = 100.0  # N/sec
        self.position_to_steps = 1.0 / 1.99675366010000e-5  # mm/step
        self.encoder_to_step = 8.19562539783576  # counts/step

        self.config = None

        self.temperature_reference = 21.0  # reference temp. for LUTs (deg_C)

        self.lut_factors = set(
            [
                "F_0.csv",
                "F_A.csv",
                "F_E.csv",
                "F_F.csv",
                "Tr.csv",
                "Tu.csv",
                "Tx.csv",
                "Ty.csv",
                "temp_inv.txt",
            ]
        )
        self.lut = dict()

        # Remote to listen to MTMount position
        self.mtmount = salobj.Remote(self.domain, "MTMount", include=["elevation"])

        self.mtmount.tel_elevation.callback = self.set_mount_elevation_callback

        self.evt_inclinationTelemetrySource.set(
            source=MTM2.InclinationTelemetrySource.ONBOARD
        )

        self.evt_forceBalanceSystemStatus.set(status=False)

        self.evt_tcpIpConnected.set(isConnected=True)
        self.evt_interlock.set(state=False)
        self.evt_commandableByDDS.set(state=True)
        self.evt_cellTemperatureHiWarning.set(hiWarning=False)

        self.tel_zenithAngle.set(
            measured=3.5, inclinometerRaw=3.5, inclinometerProcessed=3.5,
        )

        self.tel_temperature.set(
            ring=np.zeros(self.n_ring_temperatures) + 11.0,
            intake=np.zeros(self.n_intake_temperatures) + 11.0,
            exhaust=np.zeros(self.n_exhaust_temperatures) + 11.0,
        )
        self.max_temperature = 20.
        self.min_temperature = 0.

    async def handle_summary_state(self):
        """Handle summary state changes.
        """

        self.log.debug(f"Handle summary state: {self.disabled_or_enabled}")

        if not self.error_cleared:
            raise RuntimeError(
                "Error not cleared. Send clearError command before sending component to standby."
            )
        elif self.disabled_or_enabled and self.telemetry_loop_task is None:
            self.log.debug("Starting telemetry loop task.")
            self.telemetry_loop_task = asyncio.create_task(self.telemetry_loop())
        elif not self.disabled_or_enabled and self.telemetry_loop_task is not None:
            self.log.debug("Closing telemetry loop task.")
            await self.stop_telemetry_loop()

    async def telemetry_loop(self):
        """Update and output telemetry information from component.
        """

        self.run_telemetry_loop = True

        self.evt_inclinationTelemetrySource.put()
        self.evt_forceBalanceSystemStatus.put()
        self.evt_tcpIpConnected.put()
        self.evt_interlock.put()
        self.evt_commandableByDDS.put()
        self.evt_cellTemperatureHiWarning.put()

        self.log.debug("Starting telemetry loop.")

        while self.run_telemetry_loop:

            try:

                self.handle_forces()

                self.tel_displacementSensors.put()
                self.tel_forceBalance.put()
                self.tel_ilcData.put()

                self.tel_netForcesTotal.put()
                self.tel_netMomentsTotal.put()

                # Set position
                self.tel_position.set(
                    **dict(
                        [
                            (
                                axis,
                                self.mirror_position[axis]
                                + np.random.normal(scale=self.position_rms),
                            )
                            for axis in ("x", "y", "z")
                        ]
                    )
                )
                # Set angles
                self.tel_position.set(
                    **dict(
                        [
                            (
                                axis,
                                self.mirror_position[axis]
                                + np.random.normal(scale=self.angle_rms),
                            )
                            for axis in ("xRot", "yRot", "zRot")
                        ]
                    )
                )
                # Set position
                self.tel_positionIMS.set(
                    **dict(
                        [
                            (
                                axis,
                                self.mirror_position[axis]
                                + np.random.normal(scale=self.position_rms),
                            )
                            for axis in ("x", "y", "z")
                        ]
                    )
                )
                # Set angles
                self.tel_positionIMS.set(
                    **dict(
                        [
                            (
                                axis,
                                self.mirror_position[axis]
                                + np.random.normal(scale=self.angle_rms),
                            )
                            for axis in ("xRot", "yRot", "zRot")
                        ]
                    )
                )

                self.tel_position.put()
                self.tel_positionIMS.put()

                self.tel_powerStatus.put()

                temperature_change = np.random.normal(
                    scale=self.temperature_rms, size=self.n_ring_temperatures
                )
                if np.any(np.array(self.tel_temperature.data.ring)
                          > self.max_temperature):
                    temperature_change = - abs(temperature_change)
                if np.any(np.array(self.tel_temperature.data.ring)
                          < self.min_temperature):
                    temperature_change = abs(temperature_change)
                self.tel_temperature.set_put(
                    ring=self.tel_temperature.data.ring
                    + temperature_change,
                    intake=np.mean(np.array(
                        self.tel_temperature.data.ring).reshape(-1, 2), axis=0) - 2
                    + np.random.normal(
                        scale=self.temperature_rms, size=self.n_intake_temperatures
                    ),
                    exhaust=np.mean(np.array(
                        self.tel_temperature.data.ring).reshape(-1, 2), axis=0) + 2
                    + np.random.normal(
                        scale=self.temperature_rms, size=self.n_exhaust_temperatures
                    ),
                )

                inclinometer_value = self.zenith_angle + np.random.normal(
                    scale=self.inclinometer_rms
                )
                if (
                    self.evt_inclinationTelemetrySource.data.source
                    == MTM2.InclinationTelemetrySource.ONBOARD
                ):
                    self.tel_zenithAngle.set_put(
                        measured=inclinometer_value,
                        inclinometerRaw=1000.0 * inclinometer_value,
                        inclinometerProcessed=inclinometer_value,
                    )
                else:
                    self.tel_zenithAngle.set_put(
                        measured=self.zenith_angle,
                        inclinometerRaw=1000.0 * inclinometer_value,
                        inclinometerProcessed=inclinometer_value,
                    )

                await asyncio.sleep(self.telemetry_interval)
            except Exception:
                self.log.exception("Exception in telemetry loop.")
                self.fault(
                    code=TELEMETRY_LOOP_DIED,
                    report="Exception in telemetry loop.",
                    traceback=traceback.format_exc(),
                )

        self.log.debug(f"Telemetry loop closed. {self.run_telemetry_loop}")

    async def do_applyForces(self, data):
        """Apply force.
        """
        self.assert_enabled()

        # Check limits
        self.check_axial_force_limit(data.axial)
        self.check_tangent_force_limit(data.tangent)

        self.evt_m2AssemblyInPosition.set_put(inPosition=False)
        self.tel_axialForce.set(applied=data.axial)
        self.tel_tangentForce.set(applied=data.tangent)

    async def do_positionMirror(self, data):
        """Position Mirror.
        """
        self.assert_enabled()
        self.evt_m2AssemblyInPosition.set_put(inPosition=False)

        await self.handle_position_mirror(
            dict(
                [
                    (axis, getattr(data, axis))
                    for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
                ]
            )
        )

        self.evt_m2AssemblyInPosition.set_put(inPosition=True)

    async def do_resetForceOffsets(self, data):
        """Resets user defined forces to zeros.
        """
        self.assert_enabled()
        self.evt_m2AssemblyInPosition.set_put(inPosition=False)

        self.tel_axialForce.set(applied=np.zeros_like(self.tel_axialForce.data.applied))

        self.tel_tangentForce.set(
            applied=np.zeros_like(self.tel_tangentForce.data.applied)
        )

    async def do_clearErrors(self, data):
        """Emulate clearError command.
        """

        if self.summary_state != salobj.State.FAULT:
            raise RuntimeError("clearErrors command only valid in FAULT state.")

        await asyncio.sleep(self.heartbeat_interval)
        self.error_cleared = True

    async def do_selectInclinationSource(self, data):
        """Command to select source of inclination data.
        """
        try:
            self.evt_inclinationTelemetrySource.set_put(
                source=MTM2.InclinationTelemetrySource(data.source)
            )
        except ValueError:
            raise RuntimeError(
                f"Command rejected. Invalid inclination source {data.source}. "
                f"Must be one of {[value for value in MTM2.InclinationTelemetrySource]}"
            )

    async def do_setTemperatureOffset(self, data):
        """Command to set temperature offset for the LUT temperature
        correction.
        """
        raise NotImplementedError("Command not implemented")

    async def do_switchForceBalanceSystem(self, data):
        """Command to switch force balance system on and off.
        """

        if self.evt_forceBalanceSystemStatus.data.status == data.status:
            raise RuntimeError(
                f"Command rejected. Force balance system status already {data.status}."
            )

        self.evt_forceBalanceSystemStatus.set_put(status=data.status)

    def set_mount_elevation_callback(self, data):
        """Callback function to set the mount elevation.
        """
        self.zenith_angle = 90.0 - data.angleActual

    async def handle_position_mirror(self, mirror_position_set_point):
        """Handle positining the mirror.

        Parameters
        ----------
        mirror_position_set_point : `dict`
            Dictionary with the same format as `self.mirror_position`.

        """

        # Check limits
        # This is a simplication of the radial position, considering that
        # zRot = 0
        radial_position = np.sqrt(
            (
                mirror_position_set_point["x"]
                * np.cos(np.radians(mirror_position_set_point["yRot"] * 60.0 * 60.0))
            )
            ** 2.0
            + (
                mirror_position_set_point["y"]
                * np.cos(np.radians(mirror_position_set_point["xRot"] * 60.0 * 60.0))
            )
            ** 2.0
        )
        if radial_position > self.position_limit_radial:
            raise RuntimeError(
                f"Requested position outside of radial limits: {radial_position} "
                f"(limit: {self.position_limit_radial})"
            )

        # Another simplication of the limit in the optical direction.
        if mirror_position_set_point["z"] > self.position_limit_z:
            raise RuntimeError(
                f"Requested position out of limits: {mirror_position_set_point['z']} "
                f"(limit: {self.position_limit_z})"
            )

        mirror_distance = dict(
            [
                (axis, mirror_position_set_point[axis] - self.mirror_position[axis])
                for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
            ]
        )

        time_to_position = dict(
            [
                (axis, abs(mirror_distance[axis]) / self.mirror_position_speed[axis])
                for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
            ]
        )

        time = max([time_to_position[axis] for axis in time_to_position])

        if time <= self.telemetry_interval:
            await asyncio.sleep(time)
            for axis in self.mirror_position:
                self.mirror_position[axis] = mirror_position_set_point[axis]
        else:
            n_beats = int(np.ceil(time / self.telemetry_interval))
            beats = 0
            self.log.debug(f"It will take {n_beats} cycles to reposition the mirror.")
            while np.any(
                np.array([abs(mirror_distance[axis]) for axis in mirror_distance]) > 0.0
            ):
                self.log.debug(f"{mirror_distance}")
                if beats > n_beats + 1:
                    self.fault(
                        code=POSITION_MIRROR_ERROR,
                        report="Failed to position the mirror.",
                        traceback="",
                    )
                    raise RuntimeError("Failed to position the mirror.")
                beats += 1

                for axis in self.mirror_position:
                    if (
                        abs(mirror_distance[axis]) > 0.0
                        and abs(mirror_distance[axis])
                        < self.mirror_position_speed[axis] * self.telemetry_interval
                    ):
                        self.mirror_position[axis] = mirror_position_set_point[axis]
                        mirror_distance[axis] = 0.0
                        self.log.debug(f"{axis} in position.")
                    elif (
                        abs(mirror_distance[axis])
                        > self.mirror_position_speed[axis] * self.telemetry_interval
                    ):
                        delta_pos = (
                            self.mirror_position_speed[axis]
                            * self.telemetry_interval
                            * (1.0 if mirror_distance[axis] > 0.0 else -1.0)
                        )
                        self.mirror_position[axis] += delta_pos
                        mirror_distance[axis] -= delta_pos
                await asyncio.sleep(self.telemetry_interval)

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def configure(self, config):
        """Configure CSC.

        Parameters
        ----------
        config : `types.SimpleNamespace`
            Namespace with configuration values.

        """
        self.config = config

        self.log.debug(f"LUT directory: {self.config.lut_path}.")

        for lut in self.lut_factors:
            self.log.debug(f"Reading {lut}...")
            name, ext = lut.split(".")
            if ext == "csv":
                data = pd.read_csv(
                    self.config_dir / self.config.lut_path / lut,
                    header="infer" if name.startswith("F") else None,
                )
                self.lut[name] = np.float64(data)
                if name == "F_E":
                    self.lut["lutInAngle"] = np.float64(data.keys())
            else:
                self.lut[name] = np.loadtxt(
                    self.config_dir / self.config.lut_path / lut
                )

    async def stop_telemetry_loop(self):

        self.run_telemetry_loop = False
        try:
            await asyncio.wait_for(
                self.telemetry_loop_task, timeout=self.stop_loop_timeout
            )
        except asyncio.TimeoutError:
            self.log.debug("Timed out waiting for telemetry loop to finish. Canceling.")
            self.telemetry_loop_task.cancel()
            try:
                await self.telemetry_loop_task
            except asyncio.CancelledError:
                self.log.debug("Telemetry loop cancelled.")
        finally:
            self.telemetry_loop_task = None

    async def close_tasks(self):
        """Stop telemetry loop.
        """

        try:
            await self.stop_telemetry_loop()
        except Exception:
            self.log.exception("Exception while stopping telemetry loop.")
        finally:
            await super().close_tasks()

    def fault(self, code=None, report="", traceback=""):
        """Enter the fault state and output the ``errorCode`` event.

        Override parent method to stop telemetry loop before going to FAULT.

        Parameters
        ----------
        code : `int` (optional)
            Error code for the ``errorCode`` event.
            If `None` then ``errorCode`` is not output and you should
            output it yourself. Specifying `None` is deprecated;
            please always specify an integer error code.
        report : `str` (optional)
            Description of the error.
        traceback : `str` (optional)
            Description of the traceback, if any.
        """
        self.error_cleared = False
        self.run_telemetry_loop = False
        self.evt_forceBalanceSystemStatus.set_put(status=False)

        # Resets data
        self.tel_axialForce.data = self.tel_axialForce.DataType()
        self.tel_tangentForce.data = self.tel_tangentForce.DataType()

        super().fault(code=code, report=report, traceback=traceback)

    def handle_forces(self):
        """ Handle forces.

        The method will check if the force balance system is activated and,
        if yes, will compute the forces based on the look up tables. If not,
        the LUT forces are set to zero. Then, it make sure the forces are
        inside range and compute the forces dynamics. The final step is to
        update and publish the telemetry topics.
        """

        self.lookUpForces()
        if self.evt_forceBalanceSystemStatus.data.status:
            self.tel_axialForce.set(
                hardpointCorrection=np.random.normal(scale=self.force_rms,
                                                     size=self.n_actuators,),
            )
            self.tel_tangentForce.set(
                hardpointCorrection=np.random.normal(scale=self.force_rms,
                                                     size=self.n_tangent_actuators,),
            )
            self.tel_forceBalance.set(
                **dict(
                    [
                        (
                            axis,
                            np.random.normal(scale=self.force_rms),
                        )
                        for axis in ("fx", "fy", "fz", "mx", "my", "mz")
                    ]
                )
            )

        demanded_axial_force = self.check_axial_force_limit()
        demanded_tangent_force = self.check_tangent_force_limit()

        in_position_axial, self.axial_forces = self.force_dynamics(
            demand=demanded_axial_force, current=self.axial_forces
        )

        in_position_tangent, self.tangent_forces = self.force_dynamics(
            demand=demanded_tangent_force, current=self.tangent_forces
        )

        self.evt_m2AssemblyInPosition.set_put(
            inPosition=(in_position_axial and in_position_tangent)
        )

        self.tel_axialForce.set_put(
            measured=self.axial_forces
            + np.random.normal(scale=self.force_rms, size=self.n_actuators,)
        )

        axial_actuator_position = (
            self.force_to_position[0] * self.tel_axialForce.data.measured
            + self.force_to_position[1]
        )
        axial_actuator_steps = np.array(
            axial_actuator_position * self.position_to_steps, dtype=int
        )
        self.tel_axialActuatorSteps.set_put(steps=axial_actuator_steps)
        self.tel_axialEncoderPositions.set_put(position=axial_actuator_position)

        self.tel_tangentForce.set_put(
            measured=self.tangent_forces
            + np.random.normal(
                scale=self.force_rms, size=len(self.tel_tangentForce.data.applied),
            )
        )

        tangent_actuator_position = (
            self.force_to_position[0] * self.tel_tangentForce.data.measured
            + self.force_to_position[1]
        )
        tangent_actuator_steps = np.array(
            tangent_actuator_position * self.position_to_steps, dtype=int
        )

        self.tel_tangentActuatorSteps.set_put(steps=tangent_actuator_steps)
        self.tel_tangentEncoderPositions.set_put(position=tangent_actuator_position)

    def force_dynamics(self, demand, current):
        """Handle force dynamics.

        The method works by comparing the `demand` forces with the `current`.
        forces. For each actuator, if the force differs by less than the
        ammount of "force change per cycle" then the current value is set to
        the demand value. If the difference is larger, than the current value
        is modified by the ammount of force change per cycle.

        If the difference in force is larger than the force change per
        cycle for any element, the system is considered "not in position".

        Parameters
        ----------
        demand : `np.array`
            Demand forces.
        current : `np.array`
            Current forces.

        Returns
        -------
        in_position : `bool`
            Are forces in accepted range?
        final_force : `np.array`
            Resulting forces.

        """
        final_force = np.array(current, copy=True)
        in_position = True

        force_diff = demand - current

        if np.std(force_diff) > 2.0 * self.force_rms:
            in_position = False

        # how much force per cycle it can apply
        force_per_cycle = self.force_rate * self.telemetry_interval

        actuators_in_cycle = np.where(force_diff <= force_per_cycle)[0]

        final_force[actuators_in_cycle] = demand[actuators_in_cycle]

        actuators_outof_cycle = np.where(force_diff > force_per_cycle)[0]

        for actuator in actuators_outof_cycle:
            final_force[actuator] += (
                force_per_cycle * 1.0 if force_diff[actuator] > 0.0 else -1.0
            )
        return in_position, final_force

    def check_axial_force_limit(self, apply=None):
        """Check if axial forces are out of bounds.

        Parameters
        ----------
        apply : `np.array` or `None`
            Forces to apply. If `None` use current setup.

        Returns
        -------
        demanded_axial_force : `np.array`
            Array with the combined total force per actuator.

        Raises
        ------
        RuntimeError
            If force limit is out of range.
        """

        demanded_axial_force = (
            (apply if apply is not None else self.tel_axialForce.data.applied)
            + self.tel_axialForce.data.lutGravity
            + self.tel_axialForce.data.lutTemperature
            + self.tel_axialForce.data.hardpointCorrection
        )

        if np.any(demanded_axial_force > self.max_axial_force):
            actuators = np.where(demanded_axial_force > self.max_axial_force)[0]
            raise RuntimeError(
                f"Maximum axial force limit [{self.max_axial_force}N] reached in actuators {actuators}."
            )

        return demanded_axial_force

    def check_tangent_force_limit(self, apply=None):
        """Check if tangent forces are out of bounds.

        Parameters
        ----------
        apply : `np.array` or `None`
            Forces to apply. If `None` use current setup.

        Returns
        -------
        demanded_tangent_force : `np.array`
            Array with the combined total force per actuator.

        Raises
        ------
        RuntimeError
            If force limit is out of range.
        """

        demanded_tanget_force = np.array(
            (apply if apply is not None else self.tel_tangentForce.data.applied)
            + self.tel_tangentForce.data.lutGravity
            + self.tel_tangentForce.data.lutTemperature
            + self.tel_tangentForce.data.hardpointCorrection
        )

        if np.any(demanded_tanget_force > self.max_tangent_force):
            actuators = np.where(demanded_tanget_force > self.max_tangent_force)[0]
            raise RuntimeError(
                f"Maximum axial force limit [{self.max_tangent_force}N] reached in actuators {actuators}."
            )

        return demanded_tanget_force

    def lookUpForces(self):
        """Compute LUT forces using current system state (position and
        temperature).

        Returns
        -------
        forces : `np.ndarray`
            LUT forces in Newton.

        """

        temperature_m2 = np.concatenate(
            (
                self.tel_temperature.data.ring,
                self.tel_temperature.data.intake,
                self.tel_temperature.data.exhaust,
            )
        )

        # Order temperature data based on a12_temperature.ipynb
        temperature_bin = temperature_m2[
            [0, 1, 2, 3, 12, 15, 14, 13, 8, 9, 10, 11, 4, 5, 6, 7]
        ]
        temperature_lut = temperature_bin[[1, 2, 3, 12, 9, 8, 13, 14, 15, 11, 10, 0]]

        tcoef = self.lut["temp_inv"].dot(temperature_lut - self.temperature_reference)

        myfe = np.zeros(self.n_actuators)
        myf0 = np.zeros(self.n_actuators)
        myfa = np.zeros(self.n_actuators)
        myff = np.zeros(self.n_actuators)

        lut_angle = 90.0 - self.zenith_angle

        self.log.debug(
            f"Requested zAngle = {self.zenith_angle}," f"LUT angle = {lut_angle}."
        )

        for i in range(self.n_actuators):
            myfe[i] = np.interp(
                lut_angle, self.lut["lutInAngle"], self.lut["F_E"][i, :]
            )
            myf0[i] = np.interp(
                lut_angle, self.lut["lutInAngle"], self.lut["F_0"][i, :]
            )
            myfa[i] = np.interp(
                lut_angle, self.lut["lutInAngle"], self.lut["F_A"][i, :]
            )
            myff[i] = np.interp(
                lut_angle, self.lut["lutInAngle"], self.lut["F_F"][i, :]
            )

        self.tel_axialForce.set(
            lutGravity=myfe + myfa,
            lutTemperature=np.squeeze(
                tcoef[0] * self.lut["Tr"]
                + tcoef[1] * self.lut["Tx"]
                + tcoef[2] * self.lut["Ty"]
                + tcoef[3] * self.lut["Tu"]
            ),
        )

        tangent_force = (
            self.mirror_weight
            * np.sin(np.radians(self.zenith_angle))
            / 4.0
            / np.cos(np.radians(30.0))
        )

        self.tel_tangentForce.set(
            lutGravity=np.array(
                [
                    0.0,
                    tangent_force,
                    tangent_force,
                    0.0,
                    -tangent_force,
                    -tangent_force,
                ]
            )
        )

        return (
            myfe
            + myf0
            + myfa
            + myff
            + np.squeeze(
                tcoef[0] * self.lut["Tr"]
                + tcoef[1] * self.lut["Tx"]
                + tcoef[2] * self.lut["Ty"]
                + tcoef[3] * self.lut["Tu"]
            ),
        )

    @property
    def mirror_weight(self):
        """Mirror weight (in Newton).
        """
        return 15578.0

    @property
    def max_axial_force(self):
        """Axial force limit (in Newtons).

        According with the vendor documents these limits are hard-coded in
        the real system, so they are also hard-coded in the simulator.
        """
        return 444.82

    @property
    def max_tangent_force(self):
        """Tangent force limit (in Newtons).

        According with the vendor documents these limits are hard-coded in
        the real system, so they are also hard-coded in the simulator.
        """
        return 4893.04

    @property
    def position_limit_radial(self):
        """Mirror radial motion limit (in mm).
        """
        return 7.74

    @property
    def position_limit_z(self):
        """Mirror motion limit in the optical direction (in mm).
        """
        return 7.89

    @classmethod
    async def amain(cls):
        """Make a CSC from command-line arguments and run it.
        """
        parser = argparse.ArgumentParser(f"Run {cls.__name__}")
        parser.add_argument(
            "-v",
            "--verbose",
            action="store_true",
            help="Run in verbose mode?",
            default=False,
        )

        args = parser.parse_args()
        csc = cls(verbose=args.verbose)
        await csc.done_task
