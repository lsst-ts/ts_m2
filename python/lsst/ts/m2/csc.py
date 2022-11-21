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

import asyncio
import logging
import sys

import numpy as np
from lsst.ts import salobj
from lsst.ts.m2com import ControllerCell, MsgType
from lsst.ts.m2com import __version__ as __m2com_version__

from . import ErrorCode, Translator, __version__
from .config_schema import CONFIG_SCHEMA

__all__ = ["M2", "run_mtm2"]


class M2(salobj.ConfigurableCsc):
    """M2 commandable SAL component (CSC) class.

    Parameters
    ----------
    host : `str` or `None`, optional
        IP address of the TCP/IP interface. (the default is None and the value
        in ts_config_mttcs configuration files will be applied.)
    port_command : `int` or `None`, optional
        Command port number of the TCP/IP interface. (the default is None and
        the value in ts_config_mttcs configuration files will be applied.)
    port_telemetry : `int` or `None`, optional
        Telemetry port number of the TCP/IP interface. (the default is None and
        the value in ts_config_mttcs configuration files will be applied.)
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
    controller_cell : `lsst.ts.m2com.ControllerCell`
        Controller to do the TCP/IP communication with the servers of M2 cell.
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

    # Limits of force in Newton
    LIMIT_FORCE_AXIAL = 444.82  # 100 lbf
    LIMIT_FORCE_TANGENT = 4893.04  # 1100 lbf

    def __init__(
        self,
        host=None,
        port_command=None,
        port_telemetry=None,
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

        self.controller_cell = ControllerCell(
            log=self.log,
            timeout_in_second=timeout_in_second,
            is_csc=True,
            host=host,
            port_command=port_command,
            port_telemetry=port_telemetry,
        )

        # Translator to translate the message from component for the SAL topic
        # to use
        self._translator = Translator()

        self.config = None

        # Remote to listen to MTMount position
        self.mtmount = salobj.Remote(
            self.domain, "MTMount", include=["elevation", "elevationInPosition"]
        )
        self.mtmount.tel_elevation.callback = self.set_mount_elevation_callback
        self.mtmount.evt_elevationInPosition.callback = (
            self.set_mount_elevation_in_position_callback
        )

        # Software version of the M2 common module
        self.evt_softwareVersions.set(subsystemVersions=f"ts-m2com={__m2com_version__}")

    async def set_mount_elevation_callback(self, data):
        """Callback function to set the mount elevation.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        if self.controller_cell.are_clients_connected():
            await self.controller_cell.client_telemetry.write(
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

        if self.controller_cell.are_clients_connected():
            await self.controller_cell.client_command.write(
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
        self.log.debug(f"LUT directory in ts_config_mttcs: {self.config.lut_path}.")
        self.log.debug(f"Host in ts_config_mttcs: {self.config.host}.")
        self.log.debug(f"Command port in ts_config_mttcs: {self.config.port_command}.")
        self.log.debug(
            f"Telemetry port in ts_config_mttcs: {self.config.port_telemetry}."
        )

    async def close_tasks(self):

        await self.controller_cell.close_tasks()

        await super().close_tasks()

    async def handle_summary_state(self):
        """Handle summary state changes."""

        self.log.debug(f"Handle summary state: {self.summary_state}.")

        # Run the mock server in the simulation mode
        # self.simulation_mode is the attribute from upstream: BaseCsc
        if (self.summary_state == salobj.State.STANDBY) and (
            self.controller_cell.mock_server is not None
        ):
            await self.controller_cell.mock_server.close()
            self.controller_cell.mock_server = None

        if (self.simulation_mode == 1) and (self.controller_cell.mock_server is None):
            await self.controller_cell.run_mock_server(self.config_dir, "harrisLUT")

        # Run the event and telemetry loops
        if self.controller_cell.run_loops is False:

            self.controller_cell.run_loops = True

            self.log.debug(
                "Starting event, telemetry and connection monitor loop tasks."
            )

            self.controller_cell.start_task_event_loop(self._process_event)
            self.controller_cell.start_task_telemetry_loop(self._process_telemetry)
            self.controller_cell.start_task_connection_monitor_loop(
                self._process_lost_connection
            )

    async def _process_event(self, message=None):
        """Process the events from the M2 controller.

        Parameters
        ----------
        message : `dict` or None, optional
            Message from the M2 controller. (the default is None)
        """

        # Publish the SAL event
        if isinstance(message, dict):
            await self._publish_message_by_sal("evt_", message)

        # Fault the CSC if the controller is in Fault
        if (self.controller_cell.controller_state == salobj.State.FAULT) and (
            self.summary_state != salobj.State.FAULT
        ):
            await self.fault(
                code=ErrorCode.ControllerInFault,
                report="Controller's state is Fault.",
            )

    async def _publish_message_by_sal(self, prefix_sal_topic, message):
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
            await getattr(self, sal_topic_name).set_write(**message_payload)
        else:
            message_name_original = message["id"]
            self.log.warning(
                f"Unspecified message: {message_name_original}, ignoring..."
            )

    async def _process_telemetry(self, message=None):
        """Process the telemetry from the M2 controller.

        Parameters
        ----------
        message : `dict` or None, optional
            Message from the M2 controller. (the default is None)
        """

        # Publish the SAL telemetry
        if isinstance(message, dict):
            await self._publish_message_by_sal("tel_", message)

    async def _process_lost_connection(self):
        """Process the lost of connection."""

        if self.disabled_or_enabled:
            await self.fault(
                code=ErrorCode.NoConnection,
                report="Lost the TCP/IP connection.",
            )

    async def begin_start(self, data: salobj.type_hints.BaseDdsDataType) -> None:

        await self.cmd_start.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        return await super().begin_start(data)

    async def do_start(self, data):

        await super().do_start(data)

        await self._connect_server(self.COMMAND_TIMEOUT)

    async def _connect_server(self, timeout):
        """Connect the TCP/IP server.

        Parameters
        ----------
        timeout : `float`
            Connection timeout in second.
        """

        # Overwrite the connection information if needed
        if self.controller_cell.host is None:
            self.controller_cell.host = self.config.host

        if self.controller_cell.port_command is None:
            self.controller_cell.port_command = self.config.port_command

        if self.controller_cell.port_telemetry is None:
            self.controller_cell.port_telemetry = self.config.port_telemetry

        await self.controller_cell.connect_server()

    async def begin_standby(self, data: salobj.type_hints.BaseDdsDataType) -> None:
        await self.cmd_standby.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        return await super().begin_standby(data)

    async def do_standby(self, data):

        # Try to transition the controller's state to OFFLINE state before
        # closing the connection
        if self.controller_cell.are_clients_connected():

            timeout = self.COMMAND_TIMEOUT

            # Try to clear the error if any
            if self.controller_cell.controller_state == salobj.State.FAULT:
                await self._clear_controller_errors()

            await self._transition_controller_state(
                salobj.State.DISABLED, "standby", timeout
            )
            await self._transition_controller_state(
                salobj.State.STANDBY, "exitControl", timeout
            )

        # Disconnect from the server
        await self.controller_cell.close()

        await self.controller_cell.stop_loops()

        await super().do_standby(data)

    async def begin_enable(self, data: salobj.type_hints.BaseDdsDataType) -> None:
        await self.cmd_enable.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

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
        await self.cmd_disable.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT * 3)

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
        `ValueError`
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
            if self.controller_cell.controller_state == state_original:
                await self.controller_cell.write_command_to_server(
                    message_name,
                    timeout=timeout,
                    controller_state_expected=state_target,
                )

        except OSError:
            await self.controller_cell.close()

            await self.fault(
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
        await self.cmd_applyForces.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        message_name = "applyForces"
        self._assert_enabled_csc_and_controller(message_name, [salobj.State.ENABLED])

        force_axial = data.axial
        force_tangent = data.tangent
        self._check_applied_forces_in_range(force_axial, force_tangent)

        message_details = dict(axial=force_axial, tangent=force_tangent)
        await self._write_command_to_server(
            message_name,
            self.COMMAND_TIMEOUT,
            message_details=message_details,
        )

    def _check_applied_forces_in_range(
        self, applied_force_axial, applied_force_tangent
    ):
        """Check the applied forces are in the range or not at the moment.

        Parameters
        ----------
        applied_force_axial : `list`
            Applied axial forces in Newton.
        applied_force_tangent : `list`
            Applied tangent forces in Newton.

        Raises
        ------
        `ValueError`
            If the maximum axial force is out of range.
        `ValueError`
            If the maximum tangent force is out of range.
        """

        total_force_axial = np.array(applied_force_axial)
        total_force_tangent = np.array(applied_force_tangent)

        if self.tel_axialForce.has_data:
            total_force_axial = total_force_axial + np.array(
                self.tel_axialForce.data.measured
            )

        if self.tel_tangentForce.has_data:
            total_force_tangent = total_force_tangent + np.array(
                self.tel_tangentForce.data.measured
            )

        max_force_axial = np.max(np.abs(total_force_axial))
        max_force_tangent = np.max(np.abs(total_force_tangent))

        if max_force_axial >= self.LIMIT_FORCE_AXIAL:
            raise ValueError(
                f"Max axial force ({max_force_axial:.2f} N) >= {self.LIMIT_FORCE_AXIAL} N."
            )

        if max_force_tangent >= self.LIMIT_FORCE_TANGENT:
            raise ValueError(
                f"Max tangent force ({max_force_tangent:.2f} N) >= {self.LIMIT_FORCE_TANGENT} N."
            )

    async def do_positionMirror(self, data):
        """Position Mirror.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        await self.cmd_positionMirror.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

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
        await self.cmd_resetForceOffsets.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

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
            await self.controller_cell.clear_errors()

        except OSError:
            await self.controller_cell.close()

            await self.fault(
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
        await self.cmd_selectInclinationSource.ack_in_progress(
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
        await self.cmd_setTemperatureOffset.ack_in_progress(
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
        await self.cmd_switchForceBalanceSystem.ack_in_progress(
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
        self.controller_cell.assert_controller_state(message_name, allowed_curr_states)
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
            await self.controller_cell.write_command_to_server(
                message_name,
                message_details=message_details,
                timeout=timeout,
            )

        except OSError:
            await self.controller_cell.close()

            await self.fault(
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
            default=None,
            help="""
                 IP address of the TCP/IP interface. The default is None and
                 the value in ts_config_mttcs configuration files will be
                 applied. Do not use this in the simulation mode.
                 """,
        )

        parser.add_argument(
            "--ports",
            type=int,
            nargs=2,
            default=[None, None],
            help="""
                 Ports: [port_command, port_telemetry] of the TCP/IP interface.
                 The default is [None, None] and the value in ts_config_mttcs
                 configuration files will be applied. Do not use this in the
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


def run_mtm2():
    """Run the MTM2 CSC."""
    asyncio.run(M2.amain(0))
