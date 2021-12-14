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

import sys
import asyncio
import logging
import time

from lsst.ts import tcpip
from lsst.ts import salobj
from lsst.ts.utils import make_done_future

from .config_schema import CONFIG_SCHEMA
from . import MsgType, ErrorCode, Model, MockServer, Translator
from . import __version__

__all__ = ["M2"]


class M2(salobj.ConfigurableCsc):
    """M2 commandable SAL component (CSC) class.

    Parameters
    ----------
    host : `str`, optional
        IP address of the TCP/IP interface. (the default is
        "m2-control.cp.lsst.org", which is the IP of M2 server on summit.)
    port_command : `int`, optional
        Command port number of the TCP/IP interface. (the default is 50000)
    port_telemetry : `int`, optional
        Telemetry port number of the TCP/IP interface. (the default is 50001)
    timeout_in_second : `float`, optional
        Time limit for reading data from the TCP/IP interface (sec). (the
        default is 0.05)
    config_dir : `str` or None, optional
        Directory of configuration files, or None for the standard
        configuration directory (obtained from `_get_default_config_dir`).
        This is provided for unit testing.
    initial_state : `lsst.ts.salobj.State` or `int`, optional
        The initial state of the CSC. (the default is salobj.State.STANDBY)
    simulation_mode : `int`, optional
        Simulation mode. The default is 0: do not simulate.
    verbose : `bool`, optional
        Print the debug message to standard output or not. (the default is
        False)

    Attributes
    ----------
    log : `logging.Logger`
        A logger.
    timeout_in_second : `float`
        Time limit for reading data from the TCP/IP interface (sec).
    model : `Model`
        Model to do the TCP/IP communication with the servers.
    stop_loop_timeout : `float`
        Timeout of stoping loop in second.
    config : `types.SimpleNamespace` or None
        Namespace with configuration values.
    mtmount : `lsst.ts.salobj.Remote`
        Remote object of MTMount CSC.
    """

    # Class attributes comes from the upstream BaseCsc class
    valid_simulation_modes = (0, 1)
    version = __version__

    # Command timeout in second
    COMMAND_TIMEOUT = 10

    def __init__(
        self,
        host="m2-control.cp.lsst.org",
        port_command=50000,
        port_telemetry=50001,
        timeout_in_second=0.05,
        config_dir=None,
        initial_state=salobj.State.STANDBY,
        simulation_mode=0,
        verbose=False,
    ):
        super().__init__(
            "MTM2",
            index=0,
            config_schema=CONFIG_SCHEMA,
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

        if verbose:
            stream_handler = logging.StreamHandler(sys.stdout)
            # self.log is the attribute from the uptream: Controller
            self.log.addHandler(stream_handler)
            self.log.setLevel(logging.DEBUG)

        # IP address of the TCP/IP interface
        self._host = host

        # Sequence generator
        self._sequence_generator = salobj.index_generator()

        # Command port number of the TCP/IP interface
        self._port_command = port_command

        # Telemetry port number of the TCP/IP interface
        self._port_telemetry = port_telemetry

        self.timeout_in_second = timeout_in_second
        self.model = Model(log=self.log, timeout_in_second=self.timeout_in_second)

        # Translator to translate the message from component for the SAL topic
        # to use
        self._translator = Translator()

        # Mock server that is only needed in the simualtion mode
        self._mock_server = None

        self.stop_loop_timeout = 5.0

        self.config = None

        # Run the loops of telemetry and event or not
        self._run_loops = False

        # Task of the telemetry loop from component (asyncio.Future)
        self._task_telemetry_loop = make_done_future()

        # Task of the event loop from component (asyncio.Future)
        self._task_event_loop = make_done_future()

        # Remote to listen to MTMount position
        self.mtmount = salobj.Remote(
            self.domain, "MTMount", include=["elevation", "elevationInPosition"]
        )
        self.mtmount.tel_elevation.callback = self.set_mount_elevation_callback
        self.mtmount.evt_elevationInPosition.callback = (
            self.set_mount_elevation_in_position_callback
        )

    async def set_mount_elevation_callback(self, data):
        """Callback function to set the mount elevation.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        if self.model.are_clients_connected():
            await self.model.client_telemetry.write(
                MsgType.Telemetry,
                "elevation",
                msg_details=dict(actualPosition=data.actualPosition),
                comp_name="MTMount",
            )

    async def set_mount_elevation_in_position_callback(self, data):
        """Callback function to notify the mount elevation in position.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        if self.model.are_clients_connected():
            await self.model.client_command.write(
                MsgType.Event,
                "mountInPosition",
                msg_details=dict(inPosition=data.inPosition),
                comp_name="MTMount",
            )

    async def configure(self, config):
        """Configure CSC.

        Parameters
        ----------
        config : `types.SimpleNamespace`
            Namespace with configuration values.
        """

        self.config = config
        self.log.debug(f"LUT directory: {self.config.lut_path}.")

    async def close_tasks(self):

        await self.model.close()

        if self._mock_server is not None:
            await self._mock_server.close()
            self._mock_server = None

        try:
            await self.stop_loops()
        except Exception:
            self.log.exception("Exception while stopping the loops. Ignoring...")

        await super().close_tasks()

    async def stop_loops(self):
        """Stop the loops."""

        self._run_loops = False

        try:
            await asyncio.wait_for(
                self._task_telemetry_loop, timeout=self.stop_loop_timeout
            )
            await asyncio.wait_for(
                self._task_event_loop, timeout=self.stop_loop_timeout
            )

        except asyncio.TimeoutError:
            self.log.debug("Timed out waiting for the loops to finish. Canceling.")

            self._task_telemetry_loop.cancel()
            self._task_event_loop.cancel()

    async def handle_summary_state(self):
        """Handle summary state changes."""

        self.log.debug(f"Handle summary state: {self.summary_state}.")

        # Run the mock server in the simulation mode
        # self.simulation_mode is the attribute from upstream: BaseCsc
        if (self.summary_state == salobj.State.STANDBY) and (
            self._mock_server is not None
        ):
            await self._mock_server.close()
            self._mock_server = None

        if (self.simulation_mode == 1) and (self._mock_server is None):

            self._mock_server = MockServer(
                tcpip.LOCAL_HOST,
                port_command=0,
                port_telemetry=0,
                log=self.log,
            )
            # This is a hacking to get the configuration files in the STANDBY
            # state
            self._mock_server.model.configure(self.config_dir, "harrisLUT")
            await self._mock_server.start()

        # Run the event and telemetry loops
        if self._run_loops is False:

            self._run_loops = True

            self.log.debug("Starting the event loop task.")
            self._task_event_loop = asyncio.create_task(self._event_loop())

            self.log.debug("Starting the telemetry loop task.")
            self._task_telemetry_loop = asyncio.create_task(self._telemetry_loop())

    async def _event_loop(self):
        """Update and output event information from component."""

        self.log.debug("Begin to run the event loop from component.")

        while self._run_loops:

            message = (
                self.model.queue_event.get_nowait()
                if not self.model.queue_event.empty()
                else await self.model.queue_event.get()
            )

            # Publish the SAL event
            self._publish_message_by_sal("evt_", message)

            # Fault the CSC if the controller is in Fault
            if (self.model.controller_state == salobj.State.FAULT) and (
                self.summary_state != salobj.State.FAULT
            ):
                self.fault(
                    code=ErrorCode.ControllerInFault,
                    report="Controller's state is Fault.",
                )

            else:
                await asyncio.sleep(self.timeout_in_second)

        self.log.debug("Stop the running of event loop from component.")

    def _publish_message_by_sal(self, prefix_sal_topic, message):
        """Publish the message from component by SAL.

        Parameters
        ----------
        prefix_sal_topic : `str`
            Prefix of the SAL topic.
        message : `dict`
            Message from the component.
        """

        message_payload = self._translator.translate(message)

        message_name = message_payload["id"]
        sal_topic_name = prefix_sal_topic + message_name

        if hasattr(self, sal_topic_name):
            message_payload.pop("id")
            getattr(self, sal_topic_name).set_put(**message_payload)
        else:
            message_name_original = message["id"]
            self.log.warning(
                f"Unspecified message: {message_name_original}, ignoring..."
            )

    async def _telemetry_loop(self):
        """Update and output telemetry information from component."""

        self.log.debug("Starting telemetry loop from component.")

        messages_consumed = 0
        messages_consumed_log_timer = asyncio.create_task(
            asyncio.sleep(self.heartbeat_interval)
        )
        while self._run_loops:

            if self.model.are_clients_connected():
                message = (
                    self.model.client_telemetry.queue.get_nowait()
                    if not self.model.client_telemetry.queue.empty()
                    else await self.model.client_telemetry.queue.get()
                )
                self._publish_message_by_sal("tel_", message)
                messages_consumed += 1
                if messages_consumed_log_timer.done():
                    self.log.debug(
                        f"Consumed {messages_consumed/self.heartbeat_interval} messages/s."
                    )
                    messages_consumed = 0
                    messages_consumed_log_timer = asyncio.create_task(
                        asyncio.sleep(self.heartbeat_interval)
                    )

            else:
                self.log.debug(
                    f"Clients not connected. Waiting {self.timeout_in_second}s..."
                )
                await asyncio.sleep(self.timeout_in_second)

        self.log.debug("Telemetry loop from component closed.")

    async def begin_start(self, data: salobj.type_hints.BaseDdsDataType) -> None:

        self.cmd_start.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        return await super().begin_start(data)

    async def do_start(self, data):
        await self._connect_server(self.COMMAND_TIMEOUT)

        await super().do_start(data)

    async def _connect_server(self, timeout):
        """Connect the TCP/IP server.

        Parameters
        ----------
        timeout : `float`
            Connection timeout in second.

        Raises
        ------
        RuntimeError
            If timeout in connection.
        """

        # self.simulation_mode is the attribute from upstream: BaseCsc
        if self.simulation_mode == 0:
            host = self._host
            port_command = self._port_command
            port_telemetry = self._port_telemetry

        else:
            host = tcpip.LOCAL_HOST
            port_command = self._mock_server.server_command.port
            port_telemetry = self._mock_server.server_telemetry.port

        self.model.start(
            host,
            port_command,
            port_telemetry,
            sequence_generator=self._sequence_generator,
            timeout=timeout,
        )

        time_start = time.monotonic()
        connection_pooling_time = 0.1
        while not self.model.are_clients_connected() and (
            (time.monotonic() - time_start) < timeout
        ):
            await asyncio.sleep(connection_pooling_time)

        if not self.model.are_clients_connected():
            raise RuntimeError(
                f"Timeount in connection. Host: {host}, ports: {port_command} and {port_telemetry}"
            )

    async def begin_standby(self, data: salobj.type_hints.BaseDdsDataType) -> None:
        self.cmd_standby.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        return await super().begin_standby(data)

    async def do_standby(self, data):

        # Try to transition the controller's state to OFFLINE state before
        # closing the connection
        if self.model.are_clients_connected():

            timeout = self.COMMAND_TIMEOUT

            # Try to clear the error if any
            if self.model.controller_state == salobj.State.FAULT:
                await self._clear_controller_errors()

            await self._transition_controller_state(
                salobj.State.DISABLED, "standby", timeout
            )
            await self._transition_controller_state(
                salobj.State.STANDBY, "exitControl", timeout
            )

        # Disconnect from the server
        await self.model.close()

        await self.stop_loops()

        await super().do_standby(data)

    async def begin_enable(self, data: salobj.type_hints.BaseDdsDataType) -> None:
        self.cmd_enable.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        return await super().begin_enable(data)

    async def do_enable(self, data):

        timeout = self.COMMAND_TIMEOUT

        await self._transition_controller_state(
            salobj.State.OFFLINE, "enterControl", timeout
        )

        await self._transition_controller_state(salobj.State.STANDBY, "start", timeout)
        await self._transition_controller_state(
            salobj.State.DISABLED, "enable", timeout
        )

        await super().do_enable(data)

    async def begin_disable(self, data: salobj.type_hints.BaseDdsDataType) -> None:
        # multiply timeout by 3 as this is the number of commands executed with
        # this timeout.
        self.cmd_disable.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT * 3)

        return await super().begin_disable(data)

    async def do_disable(self, data):

        timeout = self.COMMAND_TIMEOUT

        await self._transition_controller_state(
            salobj.State.ENABLED, "disable", timeout
        )

        await self._transition_controller_state(
            salobj.State.DISABLED, "standby", timeout
        )
        await self._transition_controller_state(
            salobj.State.STANDBY, "exitControl", timeout
        )

        await super().do_disable(data)

    async def _transition_controller_state(self, state_original, message_name, timeout):
        """Transition the controller's state if possible.

        This function will only do the transition if the controller'state right
        now equals the state_original. Otherwise, nothing will happen.

        Parameters
        ----------
        state_original : `lsst.ts.salobj.State`
            Original controller's state.
        message_name : `str`
            Message name to do the state transition.
        timeout : `float`
            Connection timeout in second.

        Raises
        ------
        ValueError
            If the command (message_name) is not supported.
        """

        if message_name == "enterControl":
            state_target = salobj.State.STANDBY
        elif message_name == "start":
            state_target = salobj.State.DISABLED
        elif message_name == "enable":
            state_target = salobj.State.ENABLED
        elif message_name == "disable":
            state_target = salobj.State.DISABLED
        elif message_name == "standby":
            state_target = salobj.State.STANDBY
        elif message_name == "exitControl":
            state_target = salobj.State.OFFLINE
        else:
            raise ValueError(f"{message_name} command is not supported.")

        try:
            if self.model.controller_state == state_original:
                await self.model.write_command_to_server(
                    message_name,
                    timeout=timeout,
                    controller_state_expected=state_target,
                )

        except OSError:
            await self.model.close()

            self.fault(
                code=ErrorCode.NoConnection,
                report="Lost the TCP/IP connection.",
            )

    async def do_applyForces(self, data):
        """Apply force.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.cmd_applyForces.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        message_name = "applyForces"
        self._assert_enabled_csc_and_controller(message_name, [salobj.State.ENABLED])

        message_details = dict(axial=data.axial, tangent=data.tangent)
        await self._write_command_to_server(
            message_name,
            self.COMMAND_TIMEOUT,
            message_details=message_details,
        )

    async def do_positionMirror(self, data):
        """Position Mirror.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.cmd_positionMirror.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        message_name = "positionMirror"
        self._assert_enabled_csc_and_controller(message_name, [salobj.State.ENABLED])

        message_details = dict(
            x=data.x, y=data.y, z=data.z, xRot=data.xRot, yRot=data.yRot, zRot=data.zRot
        )
        await self._write_command_to_server(
            message_name,
            self.COMMAND_TIMEOUT,
            message_details=message_details,
        )

    async def do_resetForceOffsets(self, data):
        """Resets user defined forces to zeros.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.cmd_resetForceOffsets.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        message_name = "resetForceOffsets"
        self._assert_enabled_csc_and_controller(message_name, [salobj.State.ENABLED])

        await self._write_command_to_server(
            message_name,
            self.COMMAND_TIMEOUT,
        )

    async def do_clearErrors(self, data):
        """Emulate clearError command.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        await self._clear_controller_errors()

    async def _clear_controller_errors(self):
        """Clear the controller errors.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        try:
            await self.model.clear_errors()

        except OSError:
            await self.model.close()

            self.fault(
                code=ErrorCode.NoConnection,
                report="Lost the TCP/IP connection.",
            )

    async def do_selectInclinationSource(self, data):
        """Command to select source of inclination data.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.cmd_selectInclinationSource.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        message_name = "selectInclinationSource"
        self._assert_enabled_csc_and_controller(message_name, [salobj.State.ENABLED])

        message_details = dict(source=data.source)
        await self._write_command_to_server(
            message_name,
            self.COMMAND_TIMEOUT,
            message_details=message_details,
        )

    async def do_setTemperatureOffset(self, data):
        """Command to set temperature offset for the LUT temperature
        correction.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.cmd_setTemperatureOffset.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        message_name = "setTemperatureOffset"
        self._assert_enabled_csc_and_controller(message_name, [salobj.State.ENABLED])

        message_details = dict(ring=data.ring, intake=data.intake, exhaust=data.exhaust)
        await self._write_command_to_server(
            message_name,
            self.COMMAND_TIMEOUT,
            message_details=message_details,
        )

    async def do_switchForceBalanceSystem(self, data):
        """Command to switch force balance system on and off.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.cmd_switchForceBalanceSystem.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        message_name = "switchForceBalanceSystem"
        self._assert_enabled_csc_and_controller(message_name, [salobj.State.ENABLED])

        message_details = dict(status=data.status)
        await self._write_command_to_server(
            message_name,
            self.COMMAND_TIMEOUT,
            message_details=message_details,
        )

    def _assert_enabled_csc_and_controller(self, message_name, allowed_curr_states):
        """Assert the CSC and controller are in ENABLED state.

        Parameters
        ----------
        command_name : `str`
            Command name.
        allowed_curr_states : `list [lsst.ts.salobj.State]`
            Allowed current states.
        """
        self.model.assert_controller_state(message_name, allowed_curr_states)
        self.assert_enabled()

    async def _write_command_to_server(
        self, message_name, timeout, message_details=None
    ):
        """Write the command to server.

        Parameters
        ----------
        message_name : `str`
            Message name to server.
        timeout : `float`
            Timeout of command in second.
        message_details : `dict` or None, optional
            Message details. (the default is None)
        """

        try:
            await self.model.write_command_to_server(
                message_name,
                message_details=message_details,
                timeout=timeout,
            )

        except OSError:
            await self.model.close()

            self.fault(
                code=ErrorCode.NoConnection,
                report="Lost the TCP/IP connection.",
            )

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    @classmethod
    def add_arguments(cls, parser):
        super(M2, cls).add_arguments(parser)

        parser.add_argument(
            "--host",
            type=str,
            default="m2-control.cp.lsst.org",
            help="""
                 IP address of the TCP/IP interface. The default is
                 'm2-control.cp.lsst.org', which is the IP of M2 server on
                 summit. Do not use this in the simulation mode.
                 """,
        )

        parser.add_argument(
            "--ports",
            type=int,
            nargs=2,
            default=[50000, 50001],
            help="""
                 Ports: [port_command, port_telemetry] of the TCP/IP interface.
                 The default is [50000, 50001]. Do not use this in the
                 simulation mode.
                 """,
        )

        parser.add_argument(
            "-v",
            "--verbose",
            action="store_true",
            help="Run in verbose mode?",
            default=False,
        )

    @classmethod
    def add_kwargs_from_args(cls, args, kwargs):
        super(M2, cls).add_kwargs_from_args(args, kwargs)

        kwargs["host"] = args.host
        kwargs["port_command"] = args.ports[0]
        kwargs["port_telemetry"] = args.ports[1]
        kwargs["verbose"] = args.verbose
