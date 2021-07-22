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

import numpy as np

from . import write_json_packet

__all__ = ["MockMessageTelemetry"]


class MockMessageTelemetry:
    """Mock message of telemetry to simulate the message from real hardware.

    Parameters
    ----------
    writer : `asyncio.StreamWriter`
        Writer of the socket.

    Attributes
    ----------
    writer : `asyncio.StreamWriter`
        Writer of the socket.
    """

    def __init__(self, writer):

        self.writer = writer

    async def write_position(self, x, y, z, x_rot, y_rot, z_rot):
        """Write the message: M2 position by the actuator positions.

        Parameters
        ----------
        x : `float`
            Position x in micron.
        y : `float`
            Position y in micron.
        z : `float`
            Position z in micron.
        x_rot : `float`
            Rotation about x in arcsec.
        y_rot : `float`
            Rotation about y in arcsec.
        z_rot : `float`
            Rotation about z in arcsec.
        """

        msg = {
            "id": "position",
            "x": x,
            "y": y,
            "z": z,
            "xRot": x_rot,
            "yRot": y_rot,
            "zRot": z_rot,
        }
        await write_json_packet(self.writer, msg)

    async def write_position_ims(self, x, y, z, x_rot, y_rot, z_rot):
        """Write the message: M2 position by the independent measurement system
        (IMS).

        Parameters
        ----------
        x : `float`
            Position x in micron.
        y : `float`
            Position y in micron.
        z : `float`
            Position z in micron.
        x_rot : `float`
            Rotation about x in arcsec.
        y_rot : `float`
            Rotation about y in arcsec.
        z_rot : `float`
            Rotation about z in arcsec.
        """

        msg = {
            "id": "positionIMS",
            "x": x,
            "y": y,
            "z": z,
            "xRot": x_rot,
            "yRot": y_rot,
            "zRot": z_rot,
        }
        await write_json_packet(self.writer, msg)

    async def write_axial_force(
        self, lut_gravity, lut_temperature, applied, measured, hardpoint_correction
    ):
        """Write the message: axial force in Newton.

        Parameters
        ----------
        lut_gravity : `list`
            Gravity component of look-up table (LUT) force.
        lut_temperature : `list`
            Temperature component of look-up table (LUT) force.
        applied : `list`
            Applied force.
        measured : `list`
            Measured force.
        hardpoint_correction : `list`
            Hardpoint compensation force correction.
        """

        msg = {
            "id": "axialForce",
            "lutGravity": lut_gravity,
            "lutTemperature": lut_temperature,
            "applied": applied,
            "measured": measured,
            "hardpointCorrection": hardpoint_correction,
        }
        await write_json_packet(self.writer, msg)

    async def write_tangent_force(
        self, lut_gravity, lut_temperature, applied, measured, hardpoint_correction
    ):
        """Write the message: tangent force in Newton.

        Parameters
        ----------
        lut_gravity : `list`
            Gravity component of look-up table (LUT) force.
        lut_temperature : `list`
            Temperature component of look-up table (LUT) force.
        applied : `list`
            Applied force.
        measured : `list`
            Measured force.
        hardpoint_correction : `list`
            Hardpoint compensation force correction.
        """

        msg = {
            "id": "tangentForce",
            "lutGravity": lut_gravity,
            "lutTemperature": lut_temperature,
            "applied": applied,
            "measured": measured,
            "hardpointCorrection": hardpoint_correction,
        }
        await write_json_packet(self.writer, msg)

    async def write_temperature(self, ring, intake, exhaust):
        """Write the message: cell temperature in degree C.

        Parameters
        ----------
        ring : `list`
            Offset of ring temperatures.
        intake : `list`
            Offset of intake temperatures.
        exhaust : `list`
            Offset of exhaust temperatures.
        """

        msg = {
            "id": "temperature",
            "ring": ring,
            "intake": intake,
            "exhaust": exhaust,
        }
        await write_json_packet(self.writer, msg)

    async def write_zenith_angle(
        self, measured, inclinometer_raw, inclinometer_processed
    ):
        """Write the message: zenith angle in degree.

        Parameters
        ----------
        measured : `float`
            Measured angle.
        inclinometer_raw : `float`
            Reading raw angle of inclinometer.
        inclinometer_processed : `float`
            Processed angle of inclinometer.
        """

        msg = {
            "id": "zenithAngle",
            "measured": measured,
            "inclinometerRaw": inclinometer_raw,
            "inclinometerProcessed": inclinometer_processed,
        }
        await write_json_packet(self.writer, msg)

    async def write_axial_actuator_steps(self, steps):
        """Write the message: axial actuator steps.

        Parameters
        ----------
        steps : `list`
            Axial actuator steps.
        """

        msg = {
            "id": "axialActuatorSteps",
            "steps": steps,
        }
        await write_json_packet(self.writer, msg)

    async def write_tangent_actuator_steps(self, steps):
        """Write the message: tangent actuator steps.

        Parameters
        ----------
        steps : `list`
            Tangent actuator steps.
        """

        msg = {
            "id": "tangentActuatorSteps",
            "steps": steps,
        }
        await write_json_packet(self.writer, msg)

    async def write_axial_encoder_positions(self, position):
        """Write the message: axial actuator encoder positions in micron.

        Parameters
        ----------
        position : `list`
            Axial actuator encoder position.
        """

        msg = {
            "id": "axialEncoderPositions",
            "position": position,
        }
        await write_json_packet(self.writer, msg)

    async def write_tangent_encoder_positions(self, position):
        """Write the message: tangent actuator encoder positions in micron.

        Parameters
        ----------
        position : `list`
            Tangent actuator encoder position.
        """

        msg = {
            "id": "tangentEncoderPositions",
            "position": position,
        }
        await write_json_packet(self.writer, msg)

    async def write_ilc_data(self, status):
        """Write the message: inner-loop controller (ILC) data.

        Parameters
        ----------
        status : `list`
            ILC status.
        """

        msg = {
            "id": "ilcData",
            "status": status,
        }
        await write_json_packet(self.writer, msg)

    async def write_displacement_sensors(self, theta_z, delta_z):
        """Write the message: raw measurements from displacement sensors.

        Parameters
        ----------
        theta_z : `list`
            Readings of theta-z from displacement sensors in micron.
        delta_z : `list`
            Readings of delta-z from displacement sensors in micron.
        """

        msg = {
            "id": "displacementSensors",
            "thetaZ": theta_z,
            "deltaZ": delta_z,
        }
        await write_json_packet(self.writer, msg)

    async def write_force_balance(self, fx, fy, fz, mx, my, mz):
        """Write the message: net forces and moments as commanded by the force
        balance system.

        Parameters
        ----------
        fx : `float`
            Total x-force in Newton.
        fy : `float`
            Total y-force in Newton.
        fz : `float`
            Total z-force in Newton.
        mx : `float`
            Total x-moment in Newton * meter.
        my : `float`
            Total y-moment in Newton * meter.
        mz : `float`
            Total z-moment in Newton * meter.
        """

        msg = {
            "id": "forceBalance",
            "fx": fx,
            "fy": fy,
            "fz": fz,
            "mx": mx,
            "my": my,
            "mz": mz,
        }
        await write_json_packet(self.writer, msg)

    async def write_net_forces_total(self, fx, fy, fz):
        """Write the message: total actuator net forces in Newton.

        Parameters
        ----------
        fx : `float`
            Net x-force.
        fy : `float`
            Net y-force.
        fz : `float`
            Net z-force.
        """

        msg = {
            "id": "netForcesTotal",
            "fx": fx,
            "fy": fy,
            "fz": fz,
        }
        await write_json_packet(self.writer, msg)

    async def write_net_moments_total(self, mx, my, mz):
        """Write the message: total actuator net moments of force in
        Newton * meter.

        Parameters
        ----------
        mx : `float`
            Net x-moment.
        my : `float`
            Net y-moment.
        mz : `float`
            Net z-moment.
        """

        msg = {
            "id": "netMomentsTotal",
            "mx": mx,
            "my": my,
            "mz": mz,
        }
        await write_json_packet(self.writer, msg)

    async def write_power_status(
        self,
        motor_voltage=23.0,
        motor_current=8.5,
        comm_voltage=23.0,
        comm_current=6.5,
        rms=0.05,
    ):
        """Write the message: power status.

        Parameters
        ----------
        motor_voltage : `float`, optional
            Total motor voltage. (the default is 23.0)
        motor_current : `float`, optional
            Total motor current in Ampere. (the default is 8.5)
        comm_voltage : `float`, optional
            Total communication voltage. (the default is 23.0)
        comm_current : `float`, optional
            Total communication current in Ampere. (the default is 6.5)
        rms : `float`, optional
            RMS variation. (default is 0.5)
        """

        rms_power = np.random.normal(scale=rms, size=4)

        msg = {
            "id": "powerStatus",
            "motorVoltage": motor_voltage + rms_power[0],
            "motorCurrent": motor_current + rms_power[1],
            "commVoltage": comm_voltage + rms_power[2],
            "commCurrent": comm_current + rms_power[3],
        }
        await write_json_packet(self.writer, msg)
