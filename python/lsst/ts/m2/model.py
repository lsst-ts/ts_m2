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

import logging
import copy

import numpy as np
import pandas as pd

from time import sleep

from lsst.ts.idl.enums import MTM2

__all__ = ["Model"]


class Model(object):
    def __init__(self, log=None, telemetry_interval=0.05):
        """Initialize the Model class.

        Parameters
        ----------
        log : `logging.Logger` or None, optional
            A logger. If None, a logger will be instantiated. (the default is
            None)
        telemetry_interval : `float`, optional
            Telemetry interval in second. (the default is 0.05, which means
            20 Hz)

        Attributes
        ----------
        log : `logging.Logger`
            A logger.
        telemetry_interval : `float`
            Telemetry interval in second.
        inclination_source : `MTM2.InclinationTelemetrySource`
            Source of the inclination.
        zenith_angle : `float`
            Zenith angle in degree.
        n_actuators : `int`
            Number of axial actuators.
        n_tangent_actuators : `int`
            Number of tangent actuators.
        mirror_position : `dict`
            Mirror position. The key is the axis (x, y, z, xRot, yRot, zRot) of
            mirror. The units are the micron and arcsec.
        temperature : `dict`
            Temperature of mirror in degree C.
        axial_forces : `dict`
            Forces of the axial actuators in Newton.
        tangent_forces : `dict`
            Forces of the tangent actuators in Newton.
        force_balance : `dict`
            Force balance system contains the information of force and moment.
            The units are the Newton and Newton * meter.
        force_balance_system_status : `bool`
            Force balance system is on or off.
        lut : `dict`
            Look-up table (LUT).
        error_cleared : `bool`
            Error is cleared or not.
        """

        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.telemetry_interval = telemetry_interval

        self.inclination_source = MTM2.InclinationTelemetrySource.ONBOARD
        self.zenith_angle = 0.0

        self.n_actuators = 72
        self.n_tangent_actuators = 6

        self.mirror_position = dict(
            [(axis, 0.0) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
        )

        self.temperature = self._get_default_temperature()

        self.axial_forces = dict(
            [
                (item, np.zeros(self.n_actuators))
                for item in (
                    "lutGravity",
                    "lutTemperature",
                    "applied",
                    "measured",
                    "hardpointCorrection",
                )
            ]
        )
        self.tangent_forces = dict(
            [
                (item, np.zeros(self.n_tangent_actuators))
                for item in (
                    "lutGravity",
                    "lutTemperature",
                    "applied",
                    "measured",
                    "hardpointCorrection",
                )
            ]
        )
        self.force_balance = dict(
            [(axis, 0) for axis in ("fx", "fy", "fz", "mx", "my", "mz")]
        )
        self.force_balance_system_status = False

        self.lut = dict()

        self.error_cleared = True

    def _get_default_temperature(self, init_temperature=11.0):
        """Get the default temperature. The unit is degree C.

        Parameters
        ----------
        init_temperature : `float`, optional
            Initial temperature in degree C. (default is 11.0)

        Returns
        -------
        temperature : `dict`
            Temperature sensor data. The key is sensor's position or reference.
        """

        temperature = dict()
        temperature["ring"] = [init_temperature] * 12
        temperature["intake"] = [init_temperature] * 2
        temperature["exhaust"] = [init_temperature] * 2
        temperature["ref"] = 21.0

        return temperature

    @property
    def mirror_weight(self):
        """Mirror weight (in Newton)."""
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
        """Mirror radial motion limit (in mm)."""
        return 7.74

    @property
    def position_limit_z(self):
        """Mirror motion limit in the optical direction (in mm)."""
        return 7.89

    def configure(self, config_dir, lut_path):
        """Do the configuration.

        Parameters
        ----------
        config_dir : `pathlib.PosixPath`
            Configuration directory.
        lut_path : `str`
            Look-up table (LUT) path.
        """

        # Look-up table (LUT) factors (filename, basically).
        lut_factors = set(
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

        for lut in lut_factors:
            self.log.debug(f"Reading {lut}...")
            name, ext = lut.split(".")
            if ext == "csv":
                data = pd.read_csv(
                    config_dir / lut_path / lut,
                    header="infer" if name.startswith("F") else None,
                )
                self.lut[name] = np.float64(data)
                if name == "F_E":
                    self.lut["lutInAngle"] = np.float64(data.keys())
            else:
                self.lut[name] = np.loadtxt(config_dir / lut_path / lut)

    def apply_forces(self, force_axial, force_tangent):
        """Apply force.

        Parameters
        ----------
        force_axial : `list` or `numpy.ndarray`
            Force of the axial actuators in Newton.
        force_tangent : `list` or `numpy.ndarray`
            Force of the tangent actuators in Newton.
        """

        # Check limits
        self.check_axial_force_limit(apply=force_axial)
        self.check_tangent_force_limit(apply=force_tangent)

        self.axial_forces["applied"] = np.array(force_axial)
        self.tangent_forces["applied"] = np.array(force_tangent)

    def check_axial_force_limit(self, apply=None):
        """Check if axial forces are out of bounds.

        Parameters
        ----------
        apply : `numpy.ndarray` or `None`, optional
            Forces to apply in Newton. If `None` use current setup. (default is
            None.)

        Returns
        -------
        demanded_axial_force : `numpy.ndarray`
            Array with the combined total force per actuator in Newton.

        Raises
        ------
        RuntimeError
            If force limit is out of range.
        """

        demanded_axial_force = (
            (apply if apply is not None else self.axial_forces["applied"])
            + self.axial_forces["lutGravity"]
            + self.axial_forces["lutTemperature"]
            + self.axial_forces["hardpointCorrection"]
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
        apply : `numpy.ndarray` or `None`, optional
            Forces to apply in Newton. If `None` use current setup. (default is
            None.)

        Returns
        -------
        demanded_tanget_force : `numpy.ndarray`
            Array with the combined total force per actuator in Newton.

        Raises
        ------
        RuntimeError
            If force limit is out of range.
        """

        demanded_tanget_force = np.array(
            (apply if apply is not None else self.tangent_forces["applied"])
            + self.tangent_forces["lutGravity"]
            + self.tangent_forces["lutTemperature"]
            + self.tangent_forces["hardpointCorrection"]
        )

        if np.any(demanded_tanget_force > self.max_tangent_force):
            actuators = np.where(demanded_tanget_force > self.max_tangent_force)[0]
            raise RuntimeError(
                f"Maximum axial force limit [{self.max_tangent_force}N] reached in actuators {actuators}."
            )

        return demanded_tanget_force

    def reset_force_offsets(self):
        """Reset the force offsets."""

        self.axial_forces["applied"] = np.zeros(self.n_actuators)
        self.tangent_forces["applied"] = np.zeros(self.n_tangent_actuators)

    def clear_errors(self):
        """Clear the errors."""

        self.error_cleared = True

    def select_inclination_source(self, source):
        """Select the inclination source.

        Parameters
        ----------
        source : `int`
            Inclination source based on the enum:
            MTM2.InclinationTelemetrySource.
        """

        self.inclination_source = MTM2.InclinationTelemetrySource(int(source))

    def set_temperature_offset(self):
        """Set the temperature offset.

        Raises
        ------
        NotImplementedError
            Command not implemented.
        """
        raise NotImplementedError("Command not implemented.")

    def get_telemetry_data(self):
        """Get the telemetry data.

        Returns
        -------
        telemetry_data : `dict`
            Telemetry data.
        in_position : `bool`
            M2 assembly is in position or not.
        """

        telemetry_data = dict()

        # Get the force data
        in_position = self.handle_forces()
        telemetry_data["axialForce"] = self.axial_forces
        telemetry_data["tangentForce"] = self.tangent_forces
        telemetry_data["forceBalance"] = self.force_balance

        # Get the position data
        telemetry_data["position"] = self._simulate_position_mirror()
        telemetry_data["positionIMS"] = self._simulate_position_mirror()

        # Get the temperature data
        self._simulate_temperature_and_update()
        temperature = copy.copy(self.temperature)
        temperature.pop("ref")
        telemetry_data["temperature"] = temperature

        # Get the zenith angle
        telemetry_data["zenithAngle"] = self._simulate_zenith_angle()

        # Get the positions and steps of axial actuators

        # Position to steps (step/mm).
        position_to_steps = 1.0 / 1.99675366010000e-5

        # Coefficients to convert from actuator force to position.
        # position = force_to_position[0] * force + force_to_position[1]
        force_to_position = [-0.0011, 0.224]

        (axial_actuator_position, axial_actuator_steps,) = self._simulate_position_step(
            force_to_position[0],
            force_to_position[1],
            self.axial_forces["measured"],
            position_to_steps,
        )

        telemetry_data["axialEncoderPositions"] = {"position": axial_actuator_position}
        telemetry_data["axialActuatorSteps"] = {"steps": axial_actuator_steps}

        # Get the positions and steps of tangent actuators
        (
            tangent_actuator_position,
            tangent_actuator_steps,
        ) = self._simulate_position_step(
            force_to_position[0],
            force_to_position[1],
            self.tangent_forces["measured"],
            position_to_steps,
        )

        telemetry_data["tangentEncoderPositions"] = {
            "position": tangent_actuator_position
        }
        telemetry_data["tangentActuatorSteps"] = {"steps": tangent_actuator_steps}

        return telemetry_data, in_position

    def handle_forces(self, force_rms=0.5):
        """Handle forces.

        The method will check if the force balance system is activated and,
        if yes, will compute the forces based on the look up tables. If not,
        the LUT forces are set to zero. Then, it makes sure the forces are
        inside range and compute the forces dynamics.

        Parameters
        ----------
        force_rms : `float`, optional
            Force rms variation in Newton. (default is 0.5)

        Returns
        -------
        `bool`
            M2 assembly is in position or not.
        """

        # Update the force data
        self.calc_look_up_forces()

        n_actuators = self.n_actuators
        n_tangent_actuators = self.n_tangent_actuators
        if self.force_balance_system_status:
            self.axial_forces["hardpointCorrection"] = np.random.normal(
                scale=force_rms,
                size=n_actuators,
            )

            self.tangent_forces["hardpointCorrection"] = np.random.normal(
                scale=force_rms,
                size=n_tangent_actuators,
            )

            self.force_balance = dict(
                [
                    (axis, np.random.normal(scale=force_rms))
                    for axis in ("fx", "fy", "fz", "mx", "my", "mz")
                ]
            )

        demanded_axial_force = self.check_axial_force_limit()
        demanded_tangent_force = self.check_tangent_force_limit()

        in_position_axial, final_force_axial = self.force_dynamics(
            demanded_axial_force, self.axial_forces["measured"], force_rms
        )
        self.axial_forces["measured"] = final_force_axial + np.random.normal(
            scale=force_rms,
            size=n_actuators,
        )

        in_position_tangent, final_force_axial_tangent = self.force_dynamics(
            demanded_tangent_force, self.tangent_forces["measured"], force_rms
        )
        self.tangent_forces["measured"] = final_force_axial_tangent + np.random.normal(
            scale=force_rms,
            size=n_tangent_actuators,
        )

        return in_position_axial and in_position_tangent

    def calc_look_up_forces(self):
        """Calculate look-up table (LUT) forces using current system state
        (position and temperature).
        """

        # Calculate the LUT forces of temperature component of axial actuators
        temperature_m2 = np.concatenate(
            (
                self.temperature["ring"],
                self.temperature["intake"],
                self.temperature["exhaust"],
            )
        )

        # Order temperature data based on a12_temperature.ipynb
        temperature_bin = temperature_m2[
            [0, 1, 2, 3, 12, 15, 14, 13, 8, 9, 10, 11, 4, 5, 6, 7]
        ]
        temperature_lut = temperature_bin[[1, 2, 3, 12, 9, 8, 13, 14, 15, 11, 10, 0]]

        tcoef = self.lut["temp_inv"].dot(temperature_lut - self.temperature["ref"])

        self.axial_forces["lutTemperature"] = np.squeeze(
            tcoef[0] * self.lut["Tr"]
            + tcoef[1] * self.lut["Tx"]
            + tcoef[2] * self.lut["Ty"]
            + tcoef[3] * self.lut["Tu"]
        )

        # Calculate the LUT forces of gravity component of axial actuators
        lut_angle = 90.0 - self.zenith_angle

        self.log.debug(
            f"Requested zAngle = {self.zenith_angle}," f"LUT angle = {lut_angle}."
        )

        n_actuators = self.n_actuators
        myfe = np.zeros(n_actuators)
        myf0 = np.zeros(n_actuators)
        myfa = np.zeros(n_actuators)
        myff = np.zeros(n_actuators)
        for i in range(n_actuators):
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

        self.axial_forces["lutGravity"] = myfe + myfa

        # Calculate the LUT forces of gravity component of tangent actuators
        tangent_force = (
            self.mirror_weight
            * np.sin(np.radians(self.zenith_angle))
            / 4.0
            / np.cos(np.radians(30.0))
        )

        self.tangent_forces["lutGravity"] = np.array(
            [0.0, tangent_force, tangent_force, 0.0, -tangent_force, -tangent_force]
        )

    def force_dynamics(self, demand, current, force_rms, force_rate=100.0):
        """Handle force dynamics.

        The method works by comparing the `demand` forces with the `current`.
        forces. For each actuator, if the force differs by less than the
        amount of "force change per cycle" then the current value is set to
        the demand value. If the difference is larger, than the current value
        is modified by the amount of force change per cycle.

        If the difference in force is larger than the force change per
        cycle for any element, the system is considered "not in position".

        Parameters
        ----------
        demand : `numpy.ndarray`
            Demand forces in Newton.
        current : `numpy.ndarray`
            Current forces in Newton.
        force_rms : `float`
            force rms variation in Newton.
        force_rate : `float`, optional
            Force rate to be able to change in each cycle. The unit is N/sec.
            (default is 100.0)

        Returns
        -------
        in_position : `bool`
            Are forces in accepted range?
        final_force : `numpy.ndarray`
            Resulting forces in Newton.
        """

        # Check the mirror is in position or not
        force_diff = demand - current
        if np.std(force_diff) > 2.0 * force_rms:
            in_position = False
        else:
            in_position = True

        # Calculate the final fource

        # how much force per cycle it can apply
        force_per_cycle = force_rate * self.telemetry_interval

        actuators_in_cycle = np.where(force_diff <= force_per_cycle)[0]

        final_force = np.array(current, copy=True)
        final_force[actuators_in_cycle] = demand[actuators_in_cycle]

        actuators_outof_cycle = np.where(force_diff > force_per_cycle)[0]

        for actuator in actuators_outof_cycle:
            final_force[actuator] += (
                force_per_cycle * 1.0 if force_diff[actuator] > 0.0 else -1.0
            )
        return in_position, final_force

    def _simulate_position_mirror(self, position_rms=0.005, angle_rms=0.005):
        """Simulate the position of mirror.

        Parameters
        ----------
        position_rms : `float`, optional
            Position rms variation in microns. (default is 0.005)
        angle_rms : `float`, optional
            Angle rms variation in arcsec. (default is 0.005)

        Returns
        -------
        position : `dict`
            Mirror's position. The key is the axis (x, y, z, xRot, yRot, zRot)
            of mirror. The units are the micron and arcsec.
        """

        position = copy.copy(self.mirror_position)

        for axis, value in position.items():

            if axis in ("x, y, z"):
                scale = position_rms
            else:
                scale = angle_rms

            position[axis] = value + np.random.normal(scale=scale)

        return position

    def _simulate_temperature_and_update(
        self, temperature_rms=0.05, max_temperature=20.0, min_temperature=0.0
    ):
        """Simulate the temperature change and update the internal value.

        Parameters
        ----------
        temperature_rms : `float`, optional
            Temperature rms variation in degree C. (default is 0.05)
        max_temperature : `float`, optional
            Maximum temperature in degree C. (default is 20.0)
        min_temperature : `float`, optional
            Minimum temperature in degree C. (default is 0.0)
        """

        temperature_ring = self.temperature["ring"]

        # Simulate the change of ring temperature
        temperature_change = np.random.normal(
            scale=temperature_rms, size=len(temperature_ring)
        )

        if np.any(np.array(temperature_ring) > max_temperature):
            temperature_change = -abs(temperature_change)

        if np.any(np.array(temperature_ring) < min_temperature):
            temperature_change = abs(temperature_change)

        # Simulate the new intake temperature
        n_intake_temperatures = len(self.temperature["intake"])
        temperature_intake_new = (
            np.mean(np.array(temperature_ring).reshape(-1, 2), axis=0)
            - 2
            + np.random.normal(scale=temperature_rms, size=n_intake_temperatures)
        )

        # Simulate the new exhaust temperature
        n_exhaust_temperatures = len(self.temperature["exhaust"])
        temperature_exhaust_new = (
            np.mean(np.array(temperature_ring).reshape(-1, 2), axis=0)
            + 2
            + np.random.normal(scale=temperature_rms, size=n_exhaust_temperatures)
        )

        # Update the internal data
        self.temperature["ring"] = temperature_ring + temperature_change
        self.temperature["intake"] = temperature_intake_new
        self.temperature["exhaust"] = temperature_exhaust_new

    def _simulate_zenith_angle(self, inclinometer_rms=0.05):
        """Simulate the zenith angle data.

        Parameters
        ----------
        inclinometer_rms : `float`, optional
            Inclinometer rms variation in degree. (default is 0.05)

        Returns
        -------
        zenith_angle : `dict`
            Zenith angle data. The unit is degree.
        """

        inclinometer_value = self.zenith_angle + np.random.normal(
            scale=inclinometer_rms
        )

        if self.inclination_source == MTM2.InclinationTelemetrySource.ONBOARD:
            measured = inclinometer_value
        else:
            measured = self.zenith_angle

        zenith_angle = dict()
        zenith_angle["measured"] = measured
        zenith_angle["inclinometerRaw"] = 1000.0 * inclinometer_value
        zenith_angle["inclinometerProcessed"] = inclinometer_value

        return zenith_angle

    def _simulate_position_step(
        self, force_to_position_slope, force_to_position_const, force, position_to_steps
    ):
        """Simulate the position and step of actuators.

        Parameters
        ----------
        force_to_position_slope : `float`
            Slope of force to position (mm/N).
        force_to_position_const : `float`
            Constant of force to position (mm).
        force : `numpy.ndarray`
            Actuator force in Newton.
        position_to_steps : `float`
            Position to steps (step/mm).

        Returns
        -------
        actuator_position : `numpy.ndarray`
            Position of the actuators.
        actuator_steps : `numpy.ndarray`
            Steps of the actuators.
        """

        actuator_position = force_to_position_slope * force + force_to_position_const
        actuator_steps = np.array(actuator_position * position_to_steps, dtype=int)

        return actuator_position, actuator_steps

    def handle_position_mirror(self, mirror_position_set_point):
        """Handle positining the mirror.

        Parameters
        ----------
        mirror_position_set_point : `dict`
            Dictionary with the same format as `self.mirror_position`.

        Raises
        ------
        RuntimeError
            Requested position outside of radial limits.
        RuntimeError
            Requested position out of limits.
        RuntimeError
            Failed to position the mirror.
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

        mirror_position_speed = dict(
            [(axis, 0.5) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
        )

        time_to_position = dict(
            [
                (axis, abs(mirror_distance[axis]) / mirror_position_speed[axis])
                for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
            ]
        )

        time = max([time_to_position[axis] for axis in time_to_position])

        if time <= self.telemetry_interval:
            sleep(time)
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
                    raise RuntimeError("Failed to position the mirror.")
                beats += 1

                for axis in self.mirror_position:
                    if (
                        abs(mirror_distance[axis]) > 0.0
                        and abs(mirror_distance[axis])
                        < mirror_position_speed[axis] * self.telemetry_interval
                    ):
                        self.mirror_position[axis] = mirror_position_set_point[axis]
                        mirror_distance[axis] = 0.0
                        self.log.debug(f"{axis} in position.")
                    elif (
                        abs(mirror_distance[axis])
                        > mirror_position_speed[axis] * self.telemetry_interval
                    ):
                        delta_pos = (
                            mirror_position_speed[axis]
                            * self.telemetry_interval
                            * (1.0 if mirror_distance[axis] > 0.0 else -1.0)
                        )
                        self.mirror_position[axis] += delta_pos
                        mirror_distance[axis] -= delta_pos
                sleep(self.telemetry_interval)
