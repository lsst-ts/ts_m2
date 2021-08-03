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

    async def write_position(self, data):
        """Write the message: M2 position by the actuator positions.

        Parameters
        ----------
        data : `dict`
            Data of M2 position.
        """

        msg = {
            "id": "position",
            "x": data["x"],
            "y": data["y"],
            "z": data["z"],
            "xRot": data["xRot"],
            "yRot": data["yRot"],
            "zRot": data["zRot"],
        }
        await write_json_packet(self.writer, msg)

    async def write_position_ims(self, data):
        """Write the message: M2 position by the independent measurement system
        (IMS).

        Parameters
        ----------
        data : `dict`
            Data of M2 position by IMS.
        """

        msg = {
            "id": "positionIMS",
            "x": data["x"],
            "y": data["y"],
            "z": data["z"],
            "xRot": data["xRot"],
            "yRot": data["yRot"],
            "zRot": data["zRot"],
        }
        await write_json_packet(self.writer, msg)

    async def write_axial_force(self, data):
        """Write the message: axial force in Newton.

        Parameters
        ----------
        data : `dict`
            Data of axial force.
        """

        msg = {
            "id": "axialForce",
            "lutGravity": list(data["lutGravity"]),
            "lutTemperature": list(data["lutTemperature"]),
            "applied": list(data["applied"]),
            "measured": list(data["measured"]),
            "hardpointCorrection": list(data["hardpointCorrection"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_tangent_force(self, data):
        """Write the message: tangent force in Newton.

        Parameters
        ----------
        data : `dict`
            Data of axial force.
        """

        msg = {
            "id": "tangentForce",
            "lutGravity": list(data["lutGravity"]),
            "lutTemperature": list(data["lutTemperature"]),
            "applied": list(data["applied"]),
            "measured": list(data["measured"]),
            "hardpointCorrection": list(data["hardpointCorrection"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_temperature(self, data):
        """Write the message: cell temperature in degree C.

        Parameters
        ----------
        data : `dict`
            Data of cell temperature.
        """

        msg = {
            "id": "temperature",
            "ring": list(data["ring"]),
            "intake": list(data["intake"]),
            "exhaust": list(data["exhaust"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_zenith_angle(self, data):
        """Write the message: zenith angle in degree.

        Parameters
        ----------
        data : `dict`
            Data of zenith angle.
        """

        msg = {
            "id": "zenithAngle",
            "measured": data["measured"],
            "inclinometerRaw": data["inclinometerRaw"],
            "inclinometerProcessed": data["inclinometerProcessed"],
        }
        await write_json_packet(self.writer, msg)

    async def write_axial_actuator_steps(self, data):
        """Write the message: axial actuator steps.

        Parameters
        ----------
        data : `dict`
            Data of axial actuator steps.
        """

        msg = {
            "id": "axialActuatorSteps",
            "steps": list(data["steps"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_tangent_actuator_steps(self, data):
        """Write the message: tangent actuator steps.

        Parameters
        ----------
        data : `dict`
            Data of tangent actuator steps.
        """

        msg = {
            "id": "tangentActuatorSteps",
            "steps": list(data["steps"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_axial_encoder_positions(self, data):
        """Write the message: axial actuator encoder positions in micron.

        Parameters
        ----------
        data : `dict`
            Data of axial actuator encoder position.
        """

        msg = {
            "id": "axialEncoderPositions",
            "position": list(data["position"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_tangent_encoder_positions(self, data):
        """Write the message: tangent actuator encoder positions in micron.

        Parameters
        ----------
        data : `dict`
            Data of tangent actuator encoder position.
        """

        msg = {
            "id": "tangentEncoderPositions",
            "position": list(data["position"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_ilc_data(self, data):
        """Write the message: inner-loop controller (ILC) data.

        Parameters
        ----------
        data : `dict`
            Data of ILC status.
        """

        msg = {
            "id": "ilcData",
            "status": list(data["status"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_displacement_sensors(self, data):
        """Write the message: raw measurements from displacement sensors.

        Parameters
        ----------
        data : `dict`
            Data of displacement sensors.
        """

        msg = {
            "id": "displacementSensors",
            "thetaZ": list(data["thetaZ"]),
            "deltaZ": list(data["deltaZ"]),
        }
        await write_json_packet(self.writer, msg)

    async def write_force_balance(self, data):
        """Write the message: net forces and moments as commanded by the force
        balance system.

        Parameters
        ----------
        data : `dict`
            Data of force balance system.
        """

        msg = {
            "id": "forceBalance",
            "fx": data["fx"],
            "fy": data["fy"],
            "fz": data["fz"],
            "mx": data["mx"],
            "my": data["my"],
            "mz": data["mz"],
        }
        await write_json_packet(self.writer, msg)

    async def write_net_forces_total(self, data):
        """Write the message: total actuator net forces in Newton.

        Parameters
        ----------
        data : `dict`
            Data of total actuator net forces.
        """

        msg = {
            "id": "netForcesTotal",
            "fx": data["fx"],
            "fy": data["fy"],
            "fz": data["fz"],
        }
        await write_json_packet(self.writer, msg)

    async def write_net_moments_total(self, data):
        """Write the message: total actuator net moments of force in
        Newton * meter.

        Parameters
        ----------
        data : `dict`
            Data of total actuator net moments of force.
        """

        msg = {
            "id": "netMomentsTotal",
            "mx": data["mx"],
            "my": data["my"],
            "mz": data["mz"],
        }
        await write_json_packet(self.writer, msg)

    async def write_power_status(self, data):
        """Write the message: power status.

        Parameters
        ----------
        data : `dict`
            Data of power status.
        """

        msg = {
            "id": "powerStatus",
            "motorVoltage": data["motorVoltage"],
            "motorCurrent": data["motorCurrent"],
            "commVoltage": data["commVoltage"],
            "commCurrent": data["commCurrent"],
        }
        await write_json_packet(self.writer, msg)
