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
import socket
import json
import asyncio
import numpy as np

from lsst.ts import salobj
from lsst.ts import tcpip
from lsst.ts.idl.enums import MTM2

from . import CommandStatus, DetailedState, write_json_packet


__all__ = ["MockServer"]


class MockServer:
    """Mock server of M2.

    Parameters
    ----------
    host : `str`
        IP address for this server; typically `LOCALHOST` for IP4
        or "::" for IP6.
    port_command : `int`, optional
        IP port for the command server. (the default is 50000)
    port_telemetry : `int`, optional
        IP port for the telemetry server. (the default is 50001)
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    socket_family : `socket.AddressFamily`, optional
        Can be set to `socket.AF_INET` or `socket.AF_INET6` to limit the server
        to IPv4 or IPv6, respectively. If `socket.AF_UNSPEC` (the default)
        the family will be determined from host, and if host is None,
        the server may listen on both IPv4 and IPv6 sockets.

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    server_command : `tcpip.OneClientServer`
        Command server.
    server_telemetry : `tcpip.OneClientServer`
        Telemetry server.
    """

    # 20 Hz (= 0.05 second)
    PERIOD_TELEMETRY_IN_SECOND = 0.05

    TIMEOUT_IN_SECOND = 0.01

    def __init__(
        self,
        host,
        port_command=50000,
        port_telemetry=50001,
        log=None,
        socket_family=socket.AF_UNSPEC,
    ):

        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.server_command = tcpip.OneClientServer(
            "Commands",
            host,
            port_command,
            self.log,
            self._connect_state_changed_callback_command,
            family=socket_family,
        )
        self.server_telemetry = tcpip.OneClientServer(
            "Telemetry",
            host,
            port_telemetry,
            self.log,
            self._connect_state_changed_callback_telemetry,
            family=socket_family,
        )

        # Following two attributes have the type of asyncio.Future
        self._monitor_loop_task_command = salobj.make_done_future()
        self._monitor_loop_task_telemetry = salobj.make_done_future()

        # This is used to send the initial messages when the connection is just
        # on. Check the self._send_welcome_message() for the details.
        self._welcome_message_sent = False

        # System is in the Enabled state or not
        # TODO: This attribute will be removed in the work of DM-30851
        self._system_enabled = False

    def _connect_state_changed_callback_command(self, server_command):
        """Called when the command server connection state changes.

        Parameters
        ----------
        server_command : `tcpip.OneClientServer`
            Command server.
        """
        self._monitor_loop_task_command.cancel()

        if self.server_command.connected:
            self._monitor_loop_task_command = asyncio.create_task(
                self._monitor_message_command()
            )

    async def _monitor_message_command(self):
        """Monitor the message of incoming command."""

        try:
            while self.server_command.connected:

                if not self._welcome_message_sent:
                    await self._send_welcome_message()
                    self._welcome_message_sent = True

                if self.server_command.reader.at_eof():
                    self.log.info("Command reader at eof; stopping monitor loop.")
                    break

                await self._process_message_command()

        except ConnectionError:
            self.log.info("Command reader disconnected.")
            self._monitor_loop_task_command.cancel()

        await self.server_command.close_client()

    async def _send_welcome_message(self):
        """Send the welcome message to describe the system status and
        configuration..

        Most of messages are just the events.
        """

        # TODO (DM-30851):
        # Part of telemetry implementation will be updated in DM-30851
        # to finish. For example, the temp_offset and inclination source

        await self._write_message_tcp_ip_connected(True)
        await self._write_message_commandable_by_dds(True)
        await self._write_message_hardpoint_list([6, 16, 26, 74, 76, 78])
        await self._write_message_interlock(False)
        await self._write_message_inclination_telemetry_source(
            MTM2.InclinationTelemetrySource.ONBOARD
        )

        temp_offset = 21.0
        await self._write_message_temperature_offset(
            [temp_offset] * 12,
            [temp_offset] * 2,
            [temp_offset] * 2,
        )

        # Sleep time is to simulate some internal inspection of
        # real system
        await self._write_message_detailed_state(DetailedState.PublishOnly)
        await asyncio.sleep(0.01)
        await self._write_message_detailed_state(DetailedState.Available)

    async def _process_message_command(self):
        """Process the incoming message of command."""

        try:
            msg_input = await asyncio.wait_for(
                self.server_command.reader.readuntil(separator=tcpip.TERMINATOR),
                self.TIMEOUT_IN_SECOND,
            )

            msg = self._decode_and_deserialize_json_message(msg_input)

            # Process the command
            name = msg["id"]
            if self._is_command(name):

                sequence_id = msg["sequence_id"]

                # Acknowledge the command
                await self._acknowledge_command(sequence_id)

                # Parts of command will wait the DM-30851 to finish
                if name == "cmd_enable":
                    await self._write_message_force_balance_system_status(True)

                    # Simulate the real hardware behavior
                    await asyncio.sleep(5)

                    self._system_enabled = True

                elif name == "cmd_disable":
                    await self._write_message_force_balance_system_status(False)

                    self._system_enabled = False

                elif name == "cmd_exitControl":
                    # Sleep time is to simulate some internal inspection of
                    # real system
                    await asyncio.sleep(0.01)
                    await self._write_message_detailed_state(DetailedState.PublishOnly)
                    await asyncio.sleep(0.01)
                    await self._write_message_detailed_state(DetailedState.Available)

                elif name == "cmd_applyForces":
                    pass

                elif name == "cmd_positionMirror":
                    pass

                elif name == "cmd_resetForceOffsets":
                    pass

                elif name == "cmd_clearErrors":
                    pass

                elif name == "cmd_switchForceBalanceSystem":
                    await self._write_message_force_balance_system_status(msg["status"])

                elif name == "cmd_selectInclinationSource":
                    await self._write_message_inclination_telemetry_source(
                        MTM2.InclinationTelemetrySource(msg["source"])
                    )

                elif name == "cmd_setTemperatureOffset":
                    await self._write_message_temperature_offset(
                        msg["ring"], msg["intake"], msg["exhaust"]
                    )

                # Command result (only reply the success at this moment)
                await self._reply_command(sequence_id)

            # TODO (DM-30851):
            # Handle the event at DM-30851

        except asyncio.TimeoutError:
            await asyncio.sleep(self.TIMEOUT_IN_SECOND)

    def _decode_and_deserialize_json_message(self, msg_encode):
        """Decode the incoming JSON message and return a dictionary.

        Parameters
        ----------
        msg_encode : `bytes` or `None`
            Encoded message.

        Returns
        -------
        `dict`
            Decoded messge. If the msg_encode is None or can not be decoded by
            JSON, return an empty dictionary.
        """

        try:
            return json.loads(msg_encode.decode()) if msg_encode is not None else dict()

        except json.JSONDecodeError:
            self.log.debug(f"Can not decode the message: {msg_encode}.")
            return dict()

    def _is_command(self, message_name):
        """Is the command or not.

        Parameters
        ----------
        message_name : `str`
            Message name in the header.

        Returns
        -------
        `bool`
            True if this is a command.
        """

        return message_name.startswith("cmd_")

    async def _acknowledge_command(self, sequence_id):
        """Acknowledge the command with the sequence ID.

        Parameters
        ----------
        sequence_id : `int`
            Sequence ID that should be >= 0.
        """

        id_ack = CommandStatus.Ack
        msg_ack = {"id": id_ack.name.lower(), "sequence_id": sequence_id}
        await write_json_packet(self.server_command.writer, msg_ack)

    async def _reply_command(self, sequence_id):
        """Reply the command with the sequence ID.

        Assume always success at this moment.

        Parameters
        ----------
        sequence_id : `int`
            Sequence ID that should be >= 0.
        """

        id_success = CommandStatus.Success
        msg_success = {
            "id": id_success.name.lower(),
            "sequence_id": sequence_id,
        }
        await write_json_packet(self.server_command.writer, msg_success)

    def _connect_state_changed_callback_telemetry(self, server_telemetry):
        """Called when the telemetry server connection state changes.

        Parameters
        ----------
        server_telemetry : `tcpip.OneClientServer`
            Telemetry server.
        """
        self._monitor_loop_task_telemetry.cancel()

        if self.server_telemetry.connected:
            self._monitor_loop_task_telemetry = asyncio.create_task(
                self._monitor_message_telemetry()
            )

    async def _monitor_message_telemetry(self):
        """Monitor the message of incoming telemetry."""

        try:
            while self.server_telemetry.connected:

                await self._write_message_telemetry()
                await asyncio.sleep(self.PERIOD_TELEMETRY_IN_SECOND)

                if self.server_telemetry.reader.at_eof():
                    self.log.info("Telemetry reader at eof; stopping monitor loop.")
                    break

                await self._process_message_telemetry()

        except ConnectionError:
            self.log.info("Telemetry reader disconnected.")
            self._monitor_loop_task_telemetry.cancel()

        await self.server_telemetry.close_client()

    async def _write_message_telemetry(self):
        """Write the telemetry message."""

        # TODO(DM-30851):
        # Most of telemetry will be implemented in the DM-30851
        if self._system_enabled:
            await self._write_message_power_status()
        else:
            await self._write_message_power_status(motor_voltage=0.0, motor_current=0.0)

    async def _process_message_telemetry(self):
        """Read and process data from telemetry server."""

        try:
            msg = await asyncio.wait_for(
                self.server_telemetry.reader.readuntil(separator=tcpip.TERMINATOR),
                self.TIMEOUT_IN_SECOND,
            )
            msg_tel = self._decode_and_deserialize_json_message(msg)

            # TODO (DM-30851):
            # Most of telemetry will be implemented in the DM-30851
            name = msg_tel["id"]
            if name == "tel_elevation":
                pass

        except asyncio.TimeoutError:
            await asyncio.sleep(self.TIMEOUT_IN_SECOND)

    def are_servers_connected(self):
        """The command and telemetry sockets are connected or not.

        Returns
        -------
        bool
            True if servers are connected. Else, False.
        """
        return self.server_command.connected and self.server_telemetry.connected

    async def start(self):
        """Start the command and telemetry TCP/IP servers."""
        await asyncio.gather(
            self.server_command.start_task, self.server_telemetry.start_task
        )

    async def close(self):
        """Cancel the tasks and close the connections.

        Note: this function is safe to call even though there is no connection.
        """

        self._monitor_loop_task_command.cancel()
        self._monitor_loop_task_telemetry.cancel()

        await self.server_command.close()
        await self.server_telemetry.close()

        # Reset some attributes
        self._welcome_message_sent = False
        self._system_enabled = False

    async def _write_message_m2_assembly_in_position(self, in_position):
        """Write the message: M2 assembly is in position or not.

        Parameters
        ----------
        in_position : `bool`
            M2 assembly is in position or not.
        """

        msg = {"id": "m2AssemblyInPosition", "inPosition": in_position}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_cell_temperature_high_warning(self, hi_warning):
        """Write the message: cell temperature is high or not.

        Parameters
        ----------
        hi_warning : `bool`
            Cell temperature is high or not.
        """

        msg = {"id": "cellTemperatureHiWarning", "hiWarning": hi_warning}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_detailed_state(self, detailed_state):
        """Write the message: detailed state.

        Parameters
        ----------
        detailed_state : enum `DetailedState`
            M2 detailed state.
        """

        msg = {"id": "detailedState", "detailedState": int(detailed_state)}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_commandable_by_dds(self, state):
        """Write the message: commandable by DDS or not.

        Parameters
        ----------
        state : `bool`
            Commandable by DDS or not.
        """

        msg = {"id": "commandableByDDS", "state": state}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_interlock(self, state):
        """Write the message: interlock.

        Parameters
        ----------
        state : `bool`
            Interlock is on or not.
        """

        msg = {"id": "interlock", "state": state}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_tcp_ip_connected(self, is_connected):
        """Write the message: TCP/IP connection is on or not.

        Parameters
        ----------
        is_connected : `bool`
            TCP/IP connection is on or not.
        """

        msg = {"id": "tcpIpConnected", "isConnected": is_connected}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_hardpoint_list(self, actuators):
        """Write the message: hardpoint list.

        Parameters
        ----------
        actuators : `list`
            Hardpoint list.
        """

        msg = {"id": "hardpointList", "actuators": actuators}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_force_balance_system_status(self, status):
        """Write the message: force balance system is on or not.

        Parameters
        ----------
        status : `bool`
            Force balance system is on or not.
        """

        msg = {"id": "forceBalanceSystemStatus", "status": status}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_inclination_telemetry_source(self, source):
        """Write the message: inclination telemetry source.

        Parameters
        ----------
        source : enum `MTM2.InclinationTelemetrySource`
            Inclination telemetry source.
        """

        msg = {"id": "inclinationTelemetrySource", "source": int(source)}
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_temperature_offset(self, ring, intake, exhaust):
        """Write the message: temperature offset in degree C.

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
            "id": "temperatureOffset",
            "ring": ring,
            "intake": intake,
            "exhaust": exhaust,
        }
        await write_json_packet(self.server_command.writer, msg)

    async def _write_message_position(self, x, y, z, x_rot, y_rot, z_rot):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_position_ims(self, x, y, z, x_rot, y_rot, z_rot):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_axial_force(
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_tangent_force(
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_temperature(self, ring, intake, exhaust):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_zenith_angle(
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_axial_actuator_steps(self, steps):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_tangent_actuator_steps(self, steps):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_axial_encoder_positions(self, position):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_tangent_encoder_positions(self, position):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_ilc_data(self, status):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_displacement_sensors(self, theta_z, delta_z):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_force_balance(self, fx, fy, fz, mx, my, mz):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_net_forces_total(self, fx, fy, fz):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_net_moments_total(self, mx, my, mz):
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
        await write_json_packet(self.server_telemetry.writer, msg)

    async def _write_message_power_status(
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
        await write_json_packet(self.server_telemetry.writer, msg)
