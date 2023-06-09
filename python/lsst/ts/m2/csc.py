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

import argparse
import asyncio
import logging
import sys
import types
import typing

import numpy as np
from lsst.ts import salobj
from lsst.ts.idl.enums import MTM2
from lsst.ts.m2com import (
    LIMIT_FORCE_AXIAL_CLOSED_LOOP,
    LIMIT_FORCE_TANGENT_CLOSED_LOOP,
    ControllerCell,
    MsgType,
)
from lsst.ts.m2com import __version__ as __m2com_version__

from . import __version__
from .config_schema import CONFIG_SCHEMA
from .enum import ErrorCode
from .translator import Translator

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
    config_dir : `str`, `pathlib.Path`, or None, optional
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
    COMMAND_TIMEOUT_LONG = 60

    def __init__(
        self,
        host: str | None = None,
        port_command: int | None = None,
        port_telemetry: int | None = None,
        timeout_in_second: float = 0.05,
        config_dir: salobj.PathType | None = None,
        initial_state: salobj.State = salobj.State.STANDBY,
        simulation_mode: int = 0,
        verbose: bool = False,
    ) -> None:
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

        self.config: types.SimpleNamespace | None = None

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

    async def set_mount_elevation_callback(self, data: salobj.BaseMsgType) -> None:
        """Callback function to set the mount elevation.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        if self.controller_cell.are_clients_connected():
            await self.controller_cell.set_external_elevation_angle(data.actualPosition)

    async def set_mount_elevation_in_position_callback(
        self, data: salobj.BaseMsgType
    ) -> None:
        """Callback function to notify the mount elevation in position.

        Note. This function will be removed after the CSC communicates with
        the cRIO directly.

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

    async def configure(self, config: types.SimpleNamespace) -> None:
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

    async def close_tasks(self) -> None:
        await self.controller_cell.close_tasks()

        await super().close_tasks()

    async def handle_summary_state(self) -> None:
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

    async def _process_event(self, message: dict | None = None) -> None:
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

    async def _publish_message_by_sal(
        self, prefix_sal_topic: str, message: dict
    ) -> None:
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

            try:
                await getattr(self, sal_topic_name).set_write(**message_payload)
            except Exception as error:
                self.log.debug(f"Error in publishing data: {error!r}.")

        else:
            message_name_original = message["id"]
            self.log.warning(
                f"Unspecified message: {message_name_original}, ignoring..."
            )

    async def _process_telemetry(self, message: dict | None = None) -> None:
        """Process the telemetry from the M2 controller.

        Parameters
        ----------
        message : `dict` or None, optional
            Message from the M2 controller. (the default is None)
        """

        # Publish the SAL telemetry
        if isinstance(message, dict):
            await self._publish_message_by_sal("tel_", message)

    async def _process_lost_connection(self) -> None:
        """Process the lost of connection."""

        if self.disabled_or_enabled:
            await self.fault(
                code=ErrorCode.NoConnection,
                report="Lost the TCP/IP connection.",
            )

    async def begin_start(self, data: salobj.BaseMsgType) -> None:
        await self.cmd_start.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        await super().begin_start(data)

    async def do_start(self, data: salobj.BaseMsgType) -> None:
        await super().do_start(data)

        await self._connect_server(self.COMMAND_TIMEOUT)

    async def _connect_server(self, timeout: float) -> None:
        """Connect the TCP/IP server.

        Parameters
        ----------
        timeout : `float`
            Connection timeout in second.
        """

        # Workaround of the mypy checking
        assert self.config is not None

        # Overwrite the connection information if needed
        if self.controller_cell.host is None:
            self.controller_cell.host = self.config.host

        if self.controller_cell.port_command is None:
            self.controller_cell.port_command = self.config.port_command

        if self.controller_cell.port_telemetry is None:
            self.controller_cell.port_telemetry = self.config.port_telemetry

        await self.controller_cell.connect_server()

    async def begin_standby(self, data: salobj.BaseMsgType) -> None:
        await self.cmd_standby.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        await super().begin_standby(data)

    async def do_standby(self, data: salobj.BaseMsgType) -> None:
        # Try to clear the error if any
        if self.controller_cell.controller_state == salobj.State.FAULT:
            try:
                await self._clear_controller_errors()

            except Exception as error:
                self.log.warning(
                    f"Ignoring the error when transitions to STANDBY state: {error}."
                )

        # Disconnect from the server
        await self.controller_cell.close()

        await self.controller_cell.stop_loops()

        await super().do_standby(data)

    async def do_enable(self, data: salobj.BaseMsgType) -> None:
        await self.cmd_enable.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT_LONG)

        timeout = self.COMMAND_TIMEOUT

        await self._transition_controller_state(
            salobj.State.OFFLINE, "enterControl", timeout
        )

        await self._transition_controller_state(salobj.State.STANDBY, "start", timeout)
        await self._transition_controller_state(
            salobj.State.DISABLED, "enable", timeout
        )

        await super().do_enable(data)

    async def begin_disable(self, data: salobj.BaseMsgType) -> None:
        # multiply timeout by 3 as this is the number of commands executed with
        # this timeout.
        await self.cmd_disable.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT * 3)

        await super().begin_disable(data)

    async def do_disable(self, data: salobj.BaseMsgType) -> None:
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

    async def _transition_controller_state(
        self, state_original: salobj.State, message_name: str, timeout: float
    ) -> None:
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
            Timeout of command in second.

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

    async def do_applyForces(self, data: salobj.BaseMsgType) -> None:
        """Apply force.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        await self.cmd_applyForces.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        self._assert_enabled_csc_and_controller("applyForces", [salobj.State.ENABLED])

        force_axial = data.axial
        force_tangent = data.tangent
        self._check_applied_forces_in_range(force_axial, force_tangent)

        await self._execute_command_to_server(
            self.controller_cell.apply_forces,
            force_axial,
            force_tangent,
        )

    def _check_applied_forces_in_range(
        self,
        applied_force_axial: typing.List[float],
        applied_force_tangent: typing.List[float],
    ) -> None:
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

        if max_force_axial >= LIMIT_FORCE_AXIAL_CLOSED_LOOP:
            raise ValueError(
                f"Max axial force ({max_force_axial:.2f} N) >= {LIMIT_FORCE_AXIAL_CLOSED_LOOP} N."
            )

        if max_force_tangent >= LIMIT_FORCE_TANGENT_CLOSED_LOOP:
            raise ValueError(
                f"Max tangent force ({max_force_tangent:.2f} N) >= {LIMIT_FORCE_TANGENT_CLOSED_LOOP} N."
            )

    async def do_positionMirror(self, data: salobj.BaseMsgType) -> None:
        """Position Mirror.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        await self.cmd_positionMirror.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        self._assert_enabled_csc_and_controller(
            "positionMirror", [salobj.State.ENABLED]
        )

        await self._execute_command_to_server(
            self.controller_cell.position_mirror,
            data.x,
            data.y,
            data.z,
            data.xRot,
            data.yRot,
            data.zRot,
        )

    async def do_resetForceOffsets(self, data: salobj.BaseMsgType) -> None:
        """Resets user defined forces to zeros.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        await self.cmd_resetForceOffsets.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        self._assert_enabled_csc_and_controller(
            "resetForceOffsets", [salobj.State.ENABLED]
        )

        await self._execute_command_to_server(
            self.controller_cell.reset_force_offsets,
        )

    async def do_clearErrors(self, data: salobj.BaseMsgType) -> None:
        """Emulate clearError command.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        await self._clear_controller_errors()

    async def _clear_controller_errors(self) -> None:
        """Clear the controller errors."""

        try:
            await self.controller_cell.clear_errors()

        except OSError:
            await self.controller_cell.close()

            if self.disabled_or_enabled:
                await self.fault(
                    code=ErrorCode.NoConnection,
                    report="Lost the TCP/IP connection.",
                )

    async def do_selectInclinationSource(self, data: salobj.BaseMsgType) -> None:
        """Command to select source of inclination data.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        await self.cmd_selectInclinationSource.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        self._assert_enabled_csc_and_controller(
            "selectInclinationSource", [salobj.State.ENABLED]
        )

        source = MTM2.InclinationTelemetrySource(data.source)
        use_mtmount = source == MTM2.InclinationTelemetrySource.MTMOUNT

        self.controller_cell.select_inclination_source(
            use_external_elevation_angle=use_mtmount,
            enable_angle_comparison=use_mtmount,
        )

        await self._execute_command_to_server(
            self.controller_cell.set_control_parameters
        )

    async def do_setTemperatureOffset(self, data: salobj.BaseMsgType) -> None:
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

        self._assert_enabled_csc_and_controller(
            "setTemperatureOffset", [salobj.State.ENABLED]
        )

        await self._execute_command_to_server(
            self.controller_cell.set_temperature_offset,
            data.ring,
            data.intake,
            data.exhaust,
        )

    async def do_switchForceBalanceSystem(self, data: salobj.BaseMsgType) -> None:
        """Command to switch force balance system on and off.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """
        await self.cmd_switchForceBalanceSystem.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        self._assert_enabled_csc_and_controller(
            "switchForceBalanceSystem", [salobj.State.ENABLED]
        )

        await self._execute_command_to_server(
            self.controller_cell.switch_force_balance_system,
            data.status,
        )

    def _assert_enabled_csc_and_controller(
        self, message_name: str, allowed_curr_states: typing.List[salobj.State]
    ) -> None:
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

    async def _execute_command_to_server(
        self,
        command: typing.Coroutine,
        *args: typing.Any,
        timeout: float | None = None,
        **kwargs: dict[str, typing.Any],
    ) -> None:
        """Execute the command to server.

        Parameters
        ----------
        command : `coroutine`
            Command.
        *args : `args`
            Arguments of the command.
        timeout : `float` or None, optional
            Timeout of command in second. If None, the default value is used.
            (the default is None)
        **kwargs : `dict`, optional
            Additional keyword arguments to run the command.
        """

        timeout = self.COMMAND_TIMEOUT if (timeout is None) else timeout

        try:
            await command(*args, timeout=timeout, **kwargs)  # type: ignore[operator]

        except OSError:
            await self.controller_cell.close()

            await self.fault(
                code=ErrorCode.NoConnection,
                report="Lost the TCP/IP connection.",
            )

    @staticmethod
    def get_config_pkg() -> str:
        return "ts_config_mttcs"

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
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
    def add_kwargs_from_args(
        cls, args: argparse.Namespace, kwargs: typing.Dict[str, typing.Any]
    ) -> None:
        super(M2, cls).add_kwargs_from_args(args, kwargs)

        kwargs["host"] = args.host
        kwargs["port_command"] = args.ports[0]
        kwargs["port_telemetry"] = args.ports[1]
        kwargs["verbose"] = args.verbose


def run_mtm2() -> None:
    """Run the MTM2 CSC."""
    asyncio.run(M2.amain(0))
