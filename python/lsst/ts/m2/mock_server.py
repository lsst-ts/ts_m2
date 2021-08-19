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

from lsst.ts import salobj
from lsst.ts import tcpip

from . import (
    MockModel,
    CommandStatus,
    DetailedState,
    write_json_packet,
    MockMessageTelemetry,
    MockMessageEvent,
    MockCommand,
)


__all__ = ["MockServer"]


class MockServer:
    """Mock server of M2.

    Parameters
    ----------
    config_dir : `pathlib.PosixPath`
        Configuration directory.
    lut_path : `str`
        Look-up table (LUT) path.
    host : `str`
        IP address for this server; typically `LOCALHOST` for IP4
        or "::" for IP6.
    port_command : `int`, optional
        IP port for the command server. (the default is 50000)
    port_telemetry : `int`, optional
        IP port for the telemetry server. (the default is 50001)
    timeout_in_second : `float`, optional
        Read timeout in second. (the default is 0.05)
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
    model : `MockModel`
        Mock model to simulate the M2 hardware behavior.
    server_command : `tcpip.OneClientServer`
        Command server.
    server_telemetry : `tcpip.OneClientServer`
        Telemetry server.
    timeout_in_second : `float`
        Read timeout in second.
    """

    # 20 Hz (= 0.05 second)
    PERIOD_TELEMETRY_IN_SECOND = 0.05

    FAKE_ERROR_CODE = 99

    def __init__(
        self,
        config_dir,
        lut_path,
        host,
        port_command=50000,
        port_telemetry=50001,
        timeout_in_second=0.05,
        log=None,
        socket_family=socket.AF_UNSPEC,
    ):

        if log is None:
            self.log = logging.getLogger(type(self).__name__)
        else:
            self.log = log.getChild(type(self).__name__)

        # Instantiate the MockModel class and do the configuration
        self.model = MockModel(
            log=self.log, telemetry_interval=self.PERIOD_TELEMETRY_IN_SECOND
        )
        self.model.configure(config_dir, lut_path)

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

        self.timeout_in_second = timeout_in_second

        # Following two attributes have the type of asyncio.Future
        self._monitor_loop_task_command = salobj.make_done_future()
        self._monitor_loop_task_telemetry = salobj.make_done_future()

        # This is used to send the initial messages when the connection is just
        # on. Check the self._send_welcome_message() for the details.
        self._welcome_message_sent = False

        # Simulate the messages
        self._message_event = None
        self._message_telemetry = None

        self._command = MockCommand()
        self._command_response = {
            "cmd_enable": self._command.enable,
            "cmd_disable": self._command.disable,
            "cmd_standby": self._command.standby,
            "cmd_start": self._command.start,
            "cmd_enterControl": self._command.enter_control,
            "cmd_exitControl": self._command.exit_control,
            "cmd_applyForces": self._command.apply_forces,
            "cmd_positionMirror": self._command.position_mirror,
            "cmd_resetForceOffsets": self._command.reset_force_offsets,
            "cmd_clearErrors": self._command.clear_errors,
            "cmd_switchForceBalanceSystem": self._command.switch_force_balance_system,
            "cmd_selectInclinationSource": self._command.select_inclination_source,
            "cmd_setTemperatureOffset": self._command.set_temperature_offset,
        }

    def _connect_state_changed_callback_command(self, server_command):
        """Called when the command server connection state changes.

        Parameters
        ----------
        server_command : `tcpip.OneClientServer`
            Command server.
        """
        self._monitor_loop_task_command.cancel()

        if self.server_command.connected:
            self._message_event = MockMessageEvent(self.server_command.writer)

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

                if self.model.is_cell_temperature_high():
                    await self._message_event.write_cell_temperature_high_warning(True)

                if self.model.in_position:
                    await self._message_event.write_m2_assembly_in_position(True)

                if not self.model.error_cleared:
                    await self._message_event.write_summary_state(salobj.State.FAULT)
                    await self._message_event.write_error_code(self.FAKE_ERROR_CODE)
                    await self._message_event.write_force_balance_system_status(
                        self.model.force_balance_system_status
                    )

                await asyncio.sleep(self.PERIOD_TELEMETRY_IN_SECOND)

                if self.server_command.reader.at_eof():
                    self.log.info("Command reader at eof; stopping monitor loop.")
                    break

                await self._process_message_command()

        except ConnectionError:
            self.log.info("Command reader disconnected.")
            self._monitor_loop_task_command.cancel()

        except asyncio.IncompleteReadError:
            self.log.info("EOF is reached.")

        await self.server_command.close_client()

    async def _send_welcome_message(self):
        """Send the welcome message to describe the system status and
        configuration..

        Most of messages are just the events.
        """

        await self._message_event.write_tcp_ip_connected(True)
        await self._message_event.write_commandable_by_dds(True)
        await self._message_event.write_hardpoint_list([6, 16, 26, 74, 76, 78])
        await self._message_event.write_interlock(False)
        await self._message_event.write_inclination_telemetry_source(
            self.model.inclination_source
        )

        temp_offset = self.model.temperature["ref"]
        await self._message_event.write_temperature_offset(
            [temp_offset] * 12,
            [temp_offset] * 2,
            [temp_offset] * 2,
        )

        # Sleep time is to simulate some internal inspection of
        # real system
        await self._message_event.write_detailed_state(DetailedState.PublishOnly)
        await asyncio.sleep(0.01)
        await self._message_event.write_detailed_state(DetailedState.Available)

        await self._message_event.write_summary_state(salobj.State.OFFLINE)

    async def _process_message_command(self):
        """Process the incoming message of command."""

        try:
            msg_input = await asyncio.wait_for(
                self.server_command.reader.readuntil(separator=tcpip.TERMINATOR),
                self.timeout_in_second,
            )

            msg = self._decode_and_deserialize_json_message(msg_input)

            # Process the command
            name = msg["id"]
            if self._is_command(name):

                # Acknowledge the command
                sequence_id = msg["sequence_id"]
                command_name = msg["id"]
                if command_name in self._command_response.keys():
                    await self._acknowledge_command(sequence_id)

                command_status = await self._process_command(msg)

                # Command result
                await self._reply_command(sequence_id, command_status)

        except asyncio.TimeoutError:
            await asyncio.sleep(self.timeout_in_second)

        except asyncio.IncompleteReadError:
            raise

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

    async def _process_command(self, message):
        """Process the command.

        Parameters
        ----------
        message : `dict`
            Command message.

        Returns
        -------
        `CommandStatus`
            Status of command execution.
        """

        command_name = message["id"]
        available_commands = list(self._command_response.keys())
        if command_name in available_commands:
            self.model, command_status = await self._command_response[command_name](
                message, self.model, self._message_event
            )
            return command_status

        else:
            self.log.debug(
                f"Unrecognized command: {command_name}. Must be one of {available_commands}."
            )
            return CommandStatus.NoAck

    async def _reply_command(self, sequence_id, command_status):
        """Reply the command with the sequence ID.

        Parameters
        ----------
        sequence_id : `int`
            Sequence ID that should be >= 0.
        command_status : `CommandStatus`
            Status of command execution.
        """

        msg_command_status = {
            "id": command_status.name.lower(),
            "sequence_id": sequence_id,
        }
        await write_json_packet(self.server_command.writer, msg_command_status)

    def _connect_state_changed_callback_telemetry(self, server_telemetry):
        """Called when the telemetry server connection state changes.

        Parameters
        ----------
        server_telemetry : `tcpip.OneClientServer`
            Telemetry server.
        """
        self._monitor_loop_task_telemetry.cancel()

        if self.server_telemetry.connected:
            self._message_telemetry = MockMessageTelemetry(self.server_telemetry.writer)

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

        except asyncio.IncompleteReadError:
            self.log.exception("EOF is reached.")

        await self.server_telemetry.close_client()

    async def _write_message_telemetry(self):
        """Write the telemetry message."""

        telemetry_data = self.model.get_telemetry_data()

        await self._message_telemetry.write_power_status(telemetry_data["powerStatus"])
        await self._message_telemetry.write_displacement_sensors(
            telemetry_data["displacementSensors"]
        )

        if self.model.actuator_power_on:
            await self._message_telemetry.write_ilc_data(telemetry_data["ilcData"])
            await self._message_telemetry.write_net_forces_total(
                telemetry_data["netForcesTotal"]
            )
            await self._message_telemetry.write_net_moments_total(
                telemetry_data["netMomentsTotal"]
            )
            await self._message_telemetry.write_axial_force(
                telemetry_data["axialForce"]
            )
            await self._message_telemetry.write_tangent_force(
                telemetry_data["tangentForce"]
            )
            await self._message_telemetry.write_force_balance(
                telemetry_data["forceBalance"]
            )
            await self._message_telemetry.write_position(telemetry_data["position"])
            await self._message_telemetry.write_position_ims(
                telemetry_data["positionIMS"]
            )
            await self._message_telemetry.write_temperature(
                telemetry_data["temperature"]
            )
            await self._message_telemetry.write_zenith_angle(
                telemetry_data["zenithAngle"]
            )
            await self._message_telemetry.write_axial_encoder_positions(
                telemetry_data["axialEncoderPositions"]
            )
            await self._message_telemetry.write_tangent_encoder_positions(
                telemetry_data["tangentEncoderPositions"]
            )
            await self._message_telemetry.write_axial_actuator_steps(
                telemetry_data["axialActuatorSteps"]
            )
            await self._message_telemetry.write_tangent_actuator_steps(
                telemetry_data["tangentActuatorSteps"]
            )

    async def _process_message_telemetry(self):
        """Read and process data from telemetry server."""

        try:
            msg = await asyncio.wait_for(
                self.server_telemetry.reader.readuntil(separator=tcpip.TERMINATOR),
                self.timeout_in_second,
            )
            msg_tel = self._decode_and_deserialize_json_message(msg)

            # In the real M2 LabVIEW code, we will compare the string with the
            # lower case
            name = msg_tel["id"].lower()
            component = msg_tel["compName"].lower()
            if name == "tel_elevation" and component == "mtmount":
                self.model.zenith_angle = 90.0 - msg_tel["actualPosition"]

        except asyncio.TimeoutError:
            await asyncio.sleep(self.timeout_in_second)

        except asyncio.IncompleteReadError:
            raise

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

        self._message_event = None
        self._message_telemetry = None

        self.model.force_balance_system_status = False
        self.model.actuator_power_on = False
        self.model.error_cleared = True
