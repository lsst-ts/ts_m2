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

from lsst.ts import salobj, tcpip
from lsst.ts.idl.enums import MTM2
from . import CommandStatus, DetailedState


__all__ = ["MockServer"]


class MockServer:
    """Mock server of M2.

    Parameters
    ----------
    host : `str` or `None`
        IP address for this server; typically `LOCALHOST` for IP4
        or "::" for IP6. If `None` then bind to all network interfaces
        (e.g. listen on an IPv4 socket and an IPv6 socket).
        None can cause trouble with port=0;
        see tcpip.OneClientServer.port in Attributes for more information.
    port_cmd : `int`
        IP port for the command server.
    port_tel : `int`
        IP port for the telemetry server.
    log : `logging.Logger` or None, optional
        A logger. If None, a logger will be instantiated. (the default is
        None)
    family : `socket.AddressFamily`, optional
        Can be set to `socket.AF_INET` or `socket.AF_INET6` to limit the server
        to IPv4 or IPv6, respectively. If `socket.AF_UNSPEC` (the default)
        the family will be determined from host, and if host is None,
        the server may listen on both IPv4 and IPv6 sockets.

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    server_cmd : `tcpip.OneClientServer`
        Command server.
    server_tel : `tcpip.OneClientServer`
        Telemetry server.
    """

    def __init__(self, host, port_cmd, port_tel, log=None, family=socket.AF_UNSPEC):

        # Set the logger
        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        self.server_cmd = tcpip.OneClientServer(
            "Commands",
            host,
            port_cmd,
            self.log,
            self._connect_callback_cmd,
            family=family,
        )
        self.server_tel = tcpip.OneClientServer(
            "Telemetry",
            host,
            port_tel,
            self.log,
            self._connect_callback_tel,
            family=family,
        )

        # Monitor loop task of command server (asyncio.Future)
        self._monitor_loop_task_cmd = salobj.make_done_future()

        # Monitor loop task of telemetry server (asyncio.Future)
        self._monitor_loop_task_tel = salobj.make_done_future()

        # Send the one-time message
        self._send_one_time_msg_done = False

        # System is in the Enabled state or not
        self._is_enabled = False

    def _connect_callback_cmd(self, server_cmd):
        """Called when the command server connection state changes.

        Parameters
        ----------
        server_cmd : `tcpip.OneClientServer`
            Command server.
        """
        self._monitor_loop_task_cmd.cancel()

        if self.server_cmd.connected:
            self._monitor_loop_task_cmd = asyncio.create_task(self._monitor_msg_cmd())

    async def _monitor_msg_cmd(self, period_check=0.001):
        """Monitor the message of incoming command.

        Parameters
        ----------
        period_check : `float`, optional
            Period of checking the incoming messages in second. (the default is
            0.001)
        """

        try:
            while True:

                if not self._send_one_time_msg_done:
                    await self._send_one_time_msg()
                    self._send_one_time_msg_done = True

                if self.server_cmd.reader.at_eof():
                    self.log.info("Command reader at eof; closing client")
                    break

                await self._process_msg_cmd(period_check)

        except ConnectionError:
            self.log.info("Command reader disconnected; closing client")
            self._monitor_loop_task_cmd.cancel()

        await self.server_cmd.close_client()

    async def _send_one_time_msg(self):
        """Send the one-time message."""

        # Part of telemetry implementation will be updated in DM-30851
        # to finish. For example, the temp_offset and inclination source

        await self._write_msg_tcp_ip_connected(True)
        await self._write_msg_commandable_by_dds(True)
        await self._write_msg_hardpoint_list([6, 16, 26, 74, 76, 78])
        await self._write_msg_interlock(False)
        await self._write_msg_inclination_telemetry_source(
            MTM2.InclinationTelemetrySource.ONBOARD
        )

        temp_offset = 21.0
        await self._write_msg_temperature_offset(
            [temp_offset] * 12,
            [temp_offset] * 2,
            [temp_offset] * 2,
        )

        # Sleep time is to simulate some internal inspection of
        # real system
        await self._write_msg_detailed_state(DetailedState.PublishOnly)
        await asyncio.sleep(0.01)
        await self._write_msg_detailed_state(DetailedState.Available)

    async def _process_msg_cmd(self, period_check):
        """Process the incoming message of command.

        Parameters
        ----------
        period_check : `float`
            Period of checking the incoming messages in second.
        """

        msg_input = await self.server_cmd.reader.read(n=1000)
        msg = self._decode_incoming_msg(msg_input)

        # Acknowledge the command
        name = msg["id"]
        sequence_id = msg["sequence_id"]
        if self._is_cmd(name):
            id_ack = CommandStatus.Ack
            msg_ack = {"id": id_ack.name.lower(), "sequence_id": sequence_id}
            await self._write_json_packet(self.server_cmd.writer, msg_ack)

        # Process the command
        # Parts of command will wait the DM-30851 to finish
        if name == "cmd_enable":
            await self._write_msg_force_balance_system_status(True)

            # Simulate the real hardware behavior
            await asyncio.sleep(5)

            self._is_enabled = True

        elif name == "cmd_disable":
            await self._write_msg_force_balance_system_status(False)

            self._is_enabled = False

        elif name == "cmd_exitControl":
            # Sleep time is to simulate some internal inspection of real system
            await asyncio.sleep(0.01)
            await self._write_msg_detailed_state(DetailedState.PublishOnly)
            await asyncio.sleep(0.01)
            await self._write_msg_detailed_state(DetailedState.Available)

        elif name == "cmd_applyForces":
            pass

        elif name == "cmd_positionMirror":
            pass

        elif name == "cmd_resetForceOffsets":
            pass

        elif name == "cmd_clearErrors":
            pass

        elif name == "cmd_switchForceBalanceSystem":
            await self._write_msg_force_balance_system_status(msg["status"])

        elif name == "cmd_selectInclinationSource":
            await self._write_msg_inclination_telemetry_source(
                MTM2.InclinationTelemetrySource(msg["source"])
            )

        elif name == "cmd_setTemperatureOffset":
            await self._write_msg_temperature_offset(
                msg["ring"], msg["intake"], msg["exhaust"]
            )

        # Command result
        if self._is_cmd(name):
            id_success = CommandStatus.Success
            msg_success = {"id": id_success.name.lower(), "sequence_id": sequence_id}
            await self._write_json_packet(self.server_cmd.writer, msg_success)

        await asyncio.sleep(period_check)

    def _decode_incoming_msg(self, msg_encode):
        """Decode the incoming message.

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

        # Decode the message
        if msg_encode is None:
            return dict()
        else:
            msg_decode = msg_encode.decode()

        # Decode by JSON
        try:
            return json.loads(msg_decode)

        except json.JSONDecodeError:
            self.log.debug(f"Can not decode the message: {msg_decode}.")
            return dict()

    def _is_cmd(self, id_value):
        """Is the command or not.

        Parameters
        ----------
        id_value : `str`
            ID value.

        Returns
        -------
        `bool`
            True if this is a command.
        """

        return "cmd_" in id_value

    def _connect_callback_tel(self, server_tel):
        """Called when the telemetry server connection state changes.

        Parameters
        ----------
        server_tel : `tcpip.OneClientServer`
            Telemetry server.
        """
        self._monitor_loop_task_tel.cancel()

        if self.server_tel.connected:
            self._monitor_loop_task_tel = asyncio.create_task(self._monitor_msg_tel())

    async def _monitor_msg_tel(self, period_tel=0.05, period_check=0.001):
        """Monitor the message of incoming telemetry.

        Parameters
        ----------
        period_tel : `float`, optional
            Telemetry period in second. The frequency is 20 Hz usually. (the
            default is 0.05)
        period_check : `float`, optional
            Period of checking the incoming messages in second. (the default is
            0.001)
        """

        try:
            while True:

                await self._write_msg_tel(period_tel)

                if self.server_tel.reader.at_eof():
                    self.log.info("Telemetry reader at eof; closing client")
                    break

                await self._process_msg_tel(period_check)

        except ConnectionError:
            self.log.info("Telemetry reader disconnected; closing client")
            self._monitor_loop_task_tel.cancel()

        await self.server_tel.close_client()

    async def _write_msg_tel(self, period):
        """Write the telemetry message.

        Parameters
        ----------
        period : `float`
            Period of telemetry in second.
        """

        # Most of telemetry will wait the DM-30851 to finish
        if self._is_enabled:
            await self._write_msg_power_status()
        else:
            await self._write_msg_power_status(motor_voltage=0.0, motor_current=0.0)

        await asyncio.sleep(period)

    async def _process_msg_tel(self, period_check):
        """Process the incoming message of telemetry.

        Parameters
        ----------
        period_check : `float`
            Period of checking the incoming messages in second.
        """

        try:
            msg = await asyncio.wait_for(
                self.server_tel.reader.read(n=1000), period_check
            )
            msg_tel = self._decode_incoming_msg(msg)

            # Most of telemetry will wait the DM-30851 to finish
            name = msg_tel["id"]
            if name == "tel_elevation":
                pass

        except asyncio.TimeoutError:
            pass

    def is_connected(self):
        """The command and telemetry sockets are connected or not.

        Returns
        -------
        bool
            True if is connected. Else, False.
        """
        return self.server_cmd.connected and self.server_tel.connected

    async def start(self):
        """Start the command and telemetry TCP/IP servers."""
        await asyncio.gather(self.server_cmd.start_task, self.server_tel.start_task)

    async def close(self):
        """Cancel the tasks and close the connections.

        Note: this function is safe to call even though there is no connection.
        """

        # Cancel the tasks
        self._monitor_loop_task_cmd.cancel()
        self._monitor_loop_task_tel.cancel()

        # Close the servers
        await self.server_cmd.close()
        await self.server_tel.close()

        self._send_one_time_msg_done = False
        self._is_enabled = False

    async def _write_json_packet(self, writer, msg_input, sleep_time=0.001):
        """Write the json packet.

        Parameters
        ----------
        writer : `asyncio.StreamWriter`
            Writer of the socket.
        msg_input : `dict`
            Input message.
        sleep_time : `float`, optional
            Sleep time after writing the message. This value should be >=
            period_check in TcpClient._monitor_msg(). (the default is 0.001)
        """

        # Transfer to json string and do the encode
        msg = json.dumps(msg_input, indent=4).encode() + tcpip.TERMINATOR

        writer.write(msg)
        await writer.drain()

        await asyncio.sleep(sleep_time)

    async def _write_msg_m2_assembly_in_position(self, in_position):
        """Write the message: M2 assembly is in position or not.

        Parameters
        ----------
        in_position : `bool`
            M2 assembly is in position or not.
        """

        msg = {"id": "m2AssemblyInPosition", "inPosition": in_position}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_cell_temperature_high_warning(self, hi_warning):
        """Write the message: cell temperature is high or not.

        Parameters
        ----------
        hi_warning : `bool`
            Cell temperature is high or not.
        """

        msg = {"id": "cellTemperatureHiWarning", "hiWarning": hi_warning}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_detailed_state(self, detailed_state):
        """Write the message: detailed state.

        Parameters
        ----------
        detailed_state : enum `DetailedState`
            M2 detailed state.
        """

        msg = {"id": "detailedState", "detailedState": int(detailed_state)}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_commandable_by_dds(self, state):
        """Write the message: commandable by DDS or not.

        Parameters
        ----------
        state : `bool`
            Commandable by DDS or not.
        """

        msg = {"id": "commandableByDDS", "state": state}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_interlock(self, state):
        """Write the message: interlock.

        Parameters
        ----------
        state : `bool`
            Interlock is on or not.
        """

        msg = {"id": "interlock", "state": state}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_tcp_ip_connected(self, is_connected):
        """Write the message: TCP/IP connection is on or not.

        Parameters
        ----------
        is_connected : `bool`
            TCP/IP connection is on or not.
        """

        msg = {"id": "tcpIpConnected", "isConnected": is_connected}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_hardpoint_list(self, actuators):
        """Write the message: hardpoint list.

        Parameters
        ----------
        actuators : `list`
            Hardpoint list.
        """

        msg = {"id": "hardpointList", "actuators": actuators}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_force_balance_system_status(self, status):
        """Write the message: force balance system is on or not.

        Parameters
        ----------
        status : `bool`
            Force balance system is on or not.
        """

        msg = {"id": "forceBalanceSystemStatus", "status": status}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_inclination_telemetry_source(self, source):
        """Write the message: inclination telemetry source.

        Parameters
        ----------
        source : enum `MTM2.InclinationTelemetrySource`
            Inclination telemetry source.
        """

        msg = {"id": "inclinationTelemetrySource", "source": int(source)}
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_temperature_offset(self, ring, intake, exhaust):
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
        await self._write_json_packet(self.server_cmd.writer, msg)

    async def _write_msg_position(self, x, y, z, x_rot, y_rot, z_rot):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_position_ims(self, x, y, z, x_rot, y_rot, z_rot):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_axial_force(
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_tangent_force(
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_temperature(self, ring, intake, exhaust):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_zenith_angle(
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_axial_actuator_steps(self, steps):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_tangent_actuator_steps(self, steps):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_axial_encoder_positions(self, position):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_tangent_encoder_positions(self, position):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_ilc_data(self, status):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_displacement_sensors(self, theta_z, delta_z):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_force_balance(self, fx, fy, fz, mx, my, mz):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_net_forces_total(self, fx, fy, fz):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_net_moments_total(self, mx, my, mz):
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
        await self._write_json_packet(self.server_tel.writer, msg)

    async def _write_msg_power_status(
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
        await self._write_json_packet(self.server_tel.writer, msg)
