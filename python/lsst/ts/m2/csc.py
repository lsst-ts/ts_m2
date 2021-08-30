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

from .config_schema import CONFIG_SCHEMA
from . import MsgType, Model, MockServer, CommandStatus, Translator
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
        The initial state of the CSC.
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
    default_initial_state = salobj.State.OFFLINE
    version = __version__

    COMMAND_TIME_OUTOUT_IN_SECOND = 10

    def __init__(
        self,
        host="m2-control.cp.lsst.org",
        port_command=50000,
        port_telemetry=50001,
        timeout_in_second=0.05,
        config_dir=None,
        initial_state=salobj.State.OFFLINE,
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
        self._task_telemetry_loop = salobj.make_done_future()

        # Task of the event loop from component (asyncio.Future)
        self._task_event_loop = salobj.make_done_future()

        # Remote to listen to MTMount position
        self.mtmount = salobj.Remote(self.domain, "MTMount", include=["elevation"])
        self.mtmount.tel_elevation.callback = self.set_mount_elevation_callback

    async def set_mount_elevation_callback(self, data):
        """Callback function to set the mount elevation.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        await self.model.client_telemetry.write(
            MsgType.Telemetry,
            "elevation",
            msg_details=dict(actualPosition=data.actualPosition),
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

        try:
            await self.stop_loops()

        except Exception:
            self.log.exception("Exception while stopping the loops. Ignoring...")

        finally:
            await self.model.close()

            if self._mock_server is not None:
                await self._mock_server.close()
                self._mock_server = None

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
        if self.simulation_mode == 1 and self._mock_server is None:

            self._mock_server = MockServer(
                tcpip.LOCAL_HOST,
                port_command=0,
                port_telemetry=0,
                log=self.log,
            )
            # This is a hacking to get the configuration files in the OFFLINE
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

            await self.connect_server()

    async def _event_loop(self):
        """Update and output event information from component."""

        self.log.debug("Begin to run the event loop from component.")

        while self._run_loops:

            if not self.model.queue_event.empty():
                message = self.model.queue_event.get_nowait()

                # Update the internal attribute: self._summary_state, which
                # comes from the upstream class: BaseCsc
                message_name = message["id"]
                if message_name == "summaryState":
                    self._summary_state = salobj.State(message["summaryState"])

                self._publish_message_by_sal("evt_", message)

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

        message_name = message["id"]
        sal_topic_name = prefix_sal_topic + message_name

        if hasattr(self, sal_topic_name):
            message_payload = self._translator.translate(message)
            getattr(self, sal_topic_name).set_put(**message_payload)
        else:
            self.log.warning(f"Unspecified message: {message_name}, ignoring...")

    async def _telemetry_loop(self):
        """Update and output telemetry information from component."""

        self.log.debug("Starting telemetry loop from component.")

        while self._run_loops:

            if not self.model.client_telemetry.queue.empty():
                message = self.model.client_telemetry.queue.get_nowait()
                self._publish_message_by_sal("tel_", message)

            else:
                await asyncio.sleep(self.timeout_in_second)

        self.log.debug("Telemetry loop from component closed.")

    async def connect_server(self):
        """Connect the TCP/IP server."""

        # self.simulation_mode is the attribute from upstream: BaseCsc
        if self.simulation_mode == 0:
            host = self._host
            port_command = self._port_command
            port_telemetry = self._port_telemetry

        else:
            host = tcpip.LOCAL_HOST
            port_command = self._mock_server.server_command.port
            port_telemetry = self._mock_server.server_telemetry.port

        self.model.start(host, port_command, port_telemetry)

    async def do_enterControl(self, data):
        """Go from OFFLINE state, Available offline substate to STANDBY.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_state("enterControl", [salobj.State.OFFLINE])

        await self._write_command_to_server("enterControl", data)

    async def do_start(self, data):
        self._assert_state("start", [salobj.State.STANDBY])

        await self._write_command_to_server("start", data)

    async def do_enable(self, data):
        self._assert_state("enable", [salobj.State.DISABLED])

        await self._write_command_to_server("enable", data)

    async def do_disable(self, data):
        self._assert_state("disable", [salobj.State.ENABLED])

        await self._write_command_to_server("disable", data)

    async def do_standby(self, data):
        self._assert_state("standby", [salobj.State.DISABLED])

        await self._write_command_to_server("standby", data)

    async def do_exitControl(self, data):
        self._assert_state("exitControl", [salobj.State.STANDBY])

        await self._write_command_to_server("exitControl", data)

    async def do_applyForces(self, data):
        """Apply force.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.assert_enabled()

        message_details = dict(axial=data.axial, tangent=data.tangent)
        await self._write_command_to_server(
            "applyForces", data, message_details=message_details
        )

    async def do_positionMirror(self, data):
        """Position Mirror.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.assert_enabled()

        message_details = dict(
            x=data.x, y=data.y, z=data.z, xRot=data.xRot, yRot=data.yRot, zRot=data.zRot
        )
        await self._write_command_to_server(
            "positionMirror", data, message_details=message_details
        )

    async def do_resetForceOffsets(self, data):
        """Resets user defined forces to zeros.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.assert_enabled()

        await self._write_command_to_server("resetForceOffsets", data)

    async def do_clearErrors(self, data):
        """Emulate clearError command.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        await self._write_command_to_server("clearErrors", data)

    async def do_selectInclinationSource(self, data):
        """Command to select source of inclination data.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self._assert_state("selectInclinationSource", [salobj.State.DISABLED])

        await self._write_command_to_server(
            "selectInclinationSource", data, message_details=dict(source=data.source)
        )

    async def do_setTemperatureOffset(self, data):
        """Command to set temperature offset for the LUT temperature
        correction.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self._assert_state("setTemperatureOffset", [salobj.State.DISABLED])

        message_details = dict(ring=data.ring, intake=data.intake, exhaust=data.exhaust)
        await self._write_command_to_server(
            "setTemperatureOffset", data, message_details=message_details
        )

    async def do_switchForceBalanceSystem(self, data):
        """Command to switch force balance system on and off.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        self.assert_enabled()

        await self._write_command_to_server(
            "switchForceBalanceSystem", data, message_details=dict(status=data.status)
        )

    def _assert_state(self, command_name, allowed_curr_states):
        """Assert the current summary state is allowed to do the command or
        not.

        Parameters
        ----------
        command_name : `str`
            Command name.
        allowed_curr_states : `list`
            Allowed current states.

        Raises
        ------
        lsst.ts.salobj.ExpectedError
            When the command is not allowed in current state.
        """

        # Make sure the data type of allowed_curr_states is list
        if not isinstance(allowed_curr_states, list):
            allowed_curr_states = [allowed_curr_states]

        curr_state = self.summary_state
        if curr_state not in allowed_curr_states:
            raise salobj.ExpectedError(
                f"{command_name} command is not allowed in state {curr_state!r}."
            )

    async def _write_command_to_server(
        self, message_name, data, command_name=None, message_details=None
    ):
        """Write the command to server.

        Parameters
        ----------
        message_name : `str`
            Message name to server.
        data : `object`
            Data of the SAL message.
        command_name : `str` or None, optional
            Command name of SAL. This might be different from the message_name
            when communicating with the M2 cell instead of M2 server. If None,
            use the message_name instead. (the default is None)
        message_details : `dict` or None, optional
            Message details. (the default is None)

        Raises
        ------
        lsst.ts.salobj.ExpectedError
            When no command acknowledgement for command from server.
        """

        # Send the command
        self.model.last_command_status = CommandStatus.Unknown
        await self.model.client_command.write(
            MsgType.Command, message_name, msg_details=message_details
        )

        # Decide the command name of SAL
        _command_name = command_name if command_name is not None else message_name

        # Track the command status
        send_ack = await self._handle_command_acknowledgement(
            _command_name, data, self.COMMAND_TIME_OUTOUT_IN_SECOND
        )

        if send_ack is False:
            raise salobj.ExpectedError(
                f"No command acknowledgement for {_command_name} from server."
            )

    async def _handle_command_acknowledgement(self, command_name, data, timeout):
        """Handle the command acknowledgement for the controller.

        Parameters
        ----------
        command_name : `str`
            Command name of SAL.
        data : `object`
            Data of the SAL message.
        timeout : `float`
           Timeout of command acknowledgement in second.

        Returns
        -------
        send_ack : `bool`
            True if send the acknowledgement. Else, False.

        Raises
        ------
        lsst.ts.salobj.ExpectedError
            When the command is failed.
        """

        send_ack = False

        time_start = time.monotonic()
        while time.monotonic() - time_start < timeout:

            last_command_status = self.model.last_command_status
            if last_command_status == CommandStatus.Success:
                # Wait one second to let the event and telemetry loops have
                # the time to publish the messages
                await asyncio.sleep(1)
                send_ack = True
                break

            elif last_command_status == CommandStatus.Fail:
                raise salobj.ExpectedError(f"{command_name} is failed.")

            elif (last_command_status == CommandStatus.Ack) and (send_ack is False):
                send_ack = True
                # Call self.cmd_{_command_name}.ack_in_progress() dynamically
                getattr(self, f"cmd_{command_name}").ack_in_progress(
                    data, timeout=timeout
                )

            await asyncio.sleep(10 * self.timeout_in_second)

        return send_ack

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
