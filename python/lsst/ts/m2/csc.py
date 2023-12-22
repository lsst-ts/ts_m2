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
from lsst.ts.m2com import (
    DEFAULT_ENABLED_FAULTS_MASK,
    LIMIT_FORCE_AXIAL_CLOSED_LOOP,
    LIMIT_FORCE_TANGENT_CLOSED_LOOP,
    NUM_ACTUATOR,
    NUM_HARDPOINTS_AXIAL,
    NUM_TANGENT_LINK,
    ActuatorDisplacementUnit,
    CommandActuator,
    ControllerCell,
    DigitalOutputStatus,
)
from lsst.ts.m2com import __version__ as __m2com_version__
from lsst.ts.m2com import check_hardpoints, read_yaml_file
from lsst.ts.utils import make_done_future
from lsst.ts.xml.component_info import ComponentInfo
from lsst.ts.xml.enums import MTM2

from . import __version__
from .config_schema import CONFIG_SCHEMA
from .enum import BumpTest, ErrorCode
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
    ilc_retry_times : `int`
        Retry times to transition the inner-loop controller (ILC) state.
    ilc_timeout : `float`
        Timeout in second for the transition of ILC state.
    system_is_ready : `bool`
        System is ready or not.
    """

    # Class attributes comes from the upstream BaseCsc class
    valid_simulation_modes = (0, 1)
    version = __version__

    # Command timeout in second
    COMMAND_TIMEOUT = 10
    COMMAND_TIMEOUT_LONG = 60
    COMMAND_TIMEOUT_LONG_ENABLE = 400

    # Short sleep time in second
    SLEEP_TIME_SHORT = 3.0
    SLEEP_TIME_MEDIUM = 5.0

    def __init__(
        self,
        host: str | None = None,
        port_command: int | None = None,
        port_telemetry: int | None = None,
        config_dir: salobj.PathType | None = None,
        initial_state: salobj.State = salobj.State.STANDBY,
        simulation_mode: int = 0,
        verbose: bool = False,
    ) -> None:
        # This is to keep the backward compatibility of ts_xml v20.0.0 that
        # does not have the 'killActuatorBumpTest' command defined in xml.
        # TODO: Remove this after ts_xml v20.1.0.
        component_info = ComponentInfo("MTM2", "sal")
        if "cmd_killActuatorBumpTest" in component_info.topics:
            setattr(self, "do_killActuatorBumpTest", self._do_killActuatorBumpTest)

        if "cmd_setHardpointList" in component_info.topics:
            setattr(self, "do_setHardpointList", self._do_setHardpointList)

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

        # Setup the controller
        self.controller_cell = ControllerCell(
            log=self.log,
            is_csc=True,
            host=host,
            port_command=port_command,
            port_telemetry=port_telemetry,
        )
        self.controller_cell.set_callback_process_event(self._process_event)
        self.controller_cell.set_callback_process_telemetry(self._process_telemetry)
        self.controller_cell.set_callback_process_lost_connection(
            self._process_lost_connection
        )

        # Translator to translate the message from component for the SAL topic
        # to use
        self._translator = Translator()

        self.config: types.SimpleNamespace | None = None

        # Remote to listen to MTMount position
        self.mtmount = salobj.Remote(self.domain, "MTMount", include=["elevation"])
        self.mtmount.tel_elevation.callback = self.set_mount_elevation_callback

        self.ilc_retry_times = 3
        self.ilc_timeout = 20.0

        self.system_is_ready = False

        # Bypassed error codes
        self._error_codes_bypass: set[int] = set()

        # Software version of the M2 common module
        self.evt_softwareVersions.set(subsystemVersions=f"ts-m2com={__m2com_version__}")

        # Task of the bump test
        self._task_bump_test = make_done_future()

    async def set_mount_elevation_callback(self, data: salobj.BaseMsgType) -> None:
        """Callback function to set the mount elevation.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        if self.controller_cell.are_clients_connected():
            await self.controller_cell.set_external_elevation_angle(data.actualPosition)

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
        await self.controller_cell.close_controller_and_mock_server()

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

    async def _process_event(self, message: dict | None = None) -> None:
        """Process the events from the M2 controller.

        Parameters
        ----------
        message : `dict` or None, optional
            Message from the M2 controller. (the default is None)
        """

        is_interlock_engaged = False

        # Publish the SAL event
        if isinstance(message, dict):
            if message["id"] == "limitSwitchStatus":
                await self._publish_limit_switch_status(
                    message["retract"], message["extend"]
                )
            else:
                await self._publish_message_by_sal("evt_", message)

            # Check the interlock status
            if (message["id"] == "interlock") and (message["state"] is True):
                is_interlock_engaged = True

        # Fault the CSC when needed
        if self.system_is_ready and (self.summary_state != salobj.State.FAULT):
            if self._exists_error_in_controller():
                await self.fault(
                    code=ErrorCode.ControllerInFault,
                    report="Controller's state is Fault.",
                )

            if is_interlock_engaged:
                await self.fault(
                    code=ErrorCode.InterlockEngaged,
                    report="Interlock is engaged.",
                )

    async def _publish_limit_switch_status(
        self, limit_switch_retract: list[int], limit_switch_extend: list[int]
    ) -> None:
        """Publish the limit switch status.

        Parameters
        ----------
        limit_switch_retract : `list`
            Triggered retract limit switch.
        limit_switch_extend : `list`
            Triggered extend limit switch.
        """

        for limit_switch in limit_switch_retract:
            await self._publish_message_by_sal(
                "evt_", dict(id="limitSwitchRetract", actuatorId=limit_switch)
            )

        for limit_switch in limit_switch_extend:
            await self._publish_message_by_sal(
                "evt_", dict(id="limitSwitchExtend", actuatorId=limit_switch)
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

        # Wait for some time to process the welcome messages
        await asyncio.sleep(self.SLEEP_TIME_SHORT)

        # Clear all the existed error if any
        if self._exists_error_in_controller():
            await self._clear_controller_errors()

        # Sleep for some time to process the messages
        await asyncio.sleep(self.SLEEP_TIME_SHORT)

    def _exists_error_in_controller(self) -> bool:
        """Exists the error in the controller or not.

        Returns
        -------
        `bool`
            True if there is the error. Otherwise, False.
        """
        return self.controller_cell.error_handler.exists_error()

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
        # By doing this, we can avoid the new error code put the system into
        # the Fault state again, which is annoying.
        self.system_is_ready = False
        await asyncio.sleep(self.SLEEP_TIME_SHORT)

        # Try to clear the error if any
        if self._exists_error_in_controller():
            try:
                await self._clear_controller_errors()

            except Exception as error:
                self.log.warning(
                    f"Ignoring the error when clearing the controller's errors: {error}."
                )

        # Cleaning up
        if self.controller_cell.are_clients_connected():
            try:
                await self._execute_command(
                    self.controller_cell.power,
                    MTM2.PowerType.Communication,
                    False,
                    timeout=self.COMMAND_TIMEOUT_LONG,
                )
                await self._execute_command(
                    self.controller_cell.set_closed_loop_control_mode,
                    MTM2.ClosedLoopControlMode.Idle,
                    timeout=self.COMMAND_TIMEOUT_LONG,
                )

            except Exception as error:
                self.log.warning(
                    f"Ignoring the error when transitions to STANDBY state: {error}."
                )

        # Disconnect from the server
        await self.controller_cell.close()

        # Clear the internal data
        self._error_codes_bypass.clear()

        await super().do_standby(data)

    async def do_enable(self, data: salobj.BaseMsgType) -> None:
        await self.cmd_enable.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT_LONG_ENABLE
        )

        self.log.info("Bypass the error codes.")
        await self._bypass_error_codes()

        # Check there is any existed error or not
        if self._exists_error_in_controller():
            raise RuntimeError(
                "The M2 controller has active errors that cannot be cleared."
                " Please, check the error table in the GUI and clear the "
                "errors before proceeding. You may also have to check the "
                "GIS for any active interlocks. See documentation for "
                "additional troubleshooting information."
            )

        # Reset motor and communication power breakers bits and cRIO interlock
        # bit. Based on the original developer in ts_mtm2, this is required to
        # make the power system works correctly.

        # TODO: Check with electrical engineer that I need to reset the cRIO
        # interlock or not in a latter time.
        self.log.info(
            "Reset the motor and communication power breakers and cRIO interlock bits."
        )
        for idx in range(2, 5):
            await self._execute_command(
                self.controller_cell.set_bit_digital_status,
                idx,
                DigitalOutputStatus.BinaryHighLevel,
            )

        # I don't understand why I need to put the CLC mode to be Idle twice.
        # This is translated from the ts_mtm2 and I need this to make the M2
        # cRIO simulator to work.
        self.log.info("Set closed loop control mode to idle.")
        await self._execute_command(
            self.controller_cell.set_closed_loop_control_mode,
            MTM2.ClosedLoopControlMode.Idle,
        )

        self.log.info("Load the configuration.")
        await self._execute_command(
            self.controller_cell.load_configuration,
        )

        self.log.info("Set control parameters.")
        await self._execute_command(
            self.controller_cell.set_control_parameters,
        )

        self.log.info("Set closed loop control mode to idle.")
        await self._execute_command(
            self.controller_cell.set_closed_loop_control_mode,
            MTM2.ClosedLoopControlMode.Idle,
        )

        self.log.info("Reset the force offsets and actuator steps.")
        await self._execute_command(
            self.controller_cell.reset_force_offsets,
        )
        await self._execute_command(
            self.controller_cell.reset_actuator_steps,
        )

        # Power on the system and enable the ILCs
        self.log.info("Power-on the communication.")
        await self._execute_command(
            self.controller_cell.power,
            MTM2.PowerType.Communication,
            True,
            timeout=self.COMMAND_TIMEOUT_LONG,
        )

        self.log.info("Power-on the motor.")
        try:
            await self._execute_command(
                self.controller_cell.power,
                MTM2.PowerType.Motor,
                True,
                timeout=self.COMMAND_TIMEOUT_LONG,
            )

        except RuntimeError as error:
            error.add_note(
                "Failed to power up motors, please check/reset the interlock or power system."
            )
            raise

        self.log.info("Enable the ILCs.")
        if not self.controller_cell.are_ilc_modes_enabled():
            try:
                await self._execute_command(
                    self.controller_cell.set_ilc_to_enabled,
                    timeout=self.ilc_timeout,
                    retry_times=self.ilc_retry_times,  # type: ignore[arg-type]
                )

            except RuntimeError as error:
                await self._basic_cleanup_and_power_off_motor()

                error.add_note(
                    "Failed to enable ILCs. Powering off motors. Please try"
                    "to run the enable command again. If it still fails, use"
                    " the GUI to debug the ILCs. It worths to try the"
                    " power-cycle as well."
                )
                raise

        self.log.info("Set closed loop control mode to open-loop control.")
        try:
            await self._execute_command(
                self.controller_cell.set_closed_loop_control_mode,
                MTM2.ClosedLoopControlMode.OpenLoop,
                timeout=self.COMMAND_TIMEOUT_LONG,
            )

        except RuntimeError as error:
            if self._exists_error_in_controller():
                error.add_note(
                    "Cannot transition to open-loop control due to existing"
                    " errors. This implies the M2 might have the bad"
                    "force/moment distribution. Use the M2 GUI to fix the"
                    " possible issues."
                )
            raise

        # Wait for some time before transitioning to the closed-loop control
        await asyncio.sleep(self.SLEEP_TIME_MEDIUM)

        self.log.info("Switch on the force balance system.")
        await self._switch_force_balance_system(True)

        # Wait for some time to stabilize the system
        await asyncio.sleep(self.SLEEP_TIME_MEDIUM)

        # System is ready now. If there is any error, the system will
        # transition to the Fault state from now on.
        self.system_is_ready = True

        await super().do_enable(data)

    async def _bypass_error_codes(self) -> None:
        """Bypass the error codes related to the monitoring inner-loop
        controller (ILC) read error before the fix and other error codes.

        TODO: Remove the ILC read error after the ILC is fixed.
        """

        # Note the union() will return a new set object
        ilc_codes_to_bypass = {1000, 1001, 6052}
        codes_to_bypass = self._error_codes_bypass.union(ilc_codes_to_bypass)

        (
            enabled_faults_mask,
            bits,
        ) = self.controller_cell.error_handler.calc_enabled_faults_mask(
            codes_to_bypass, DEFAULT_ENABLED_FAULTS_MASK
        )
        self.log.info(f"Bypass the error codes: {codes_to_bypass}. Bits: {bits}.")

        await self._execute_command(
            self.controller_cell.set_enabled_faults_mask,
            enabled_faults_mask,
        )

    async def _basic_cleanup_and_power_off_motor(self) -> None:
        """Basic cleanup and power off the motor."""

        try:
            await self._execute_command(
                self.controller_cell.reset_force_offsets,
            )
            await self._execute_command(
                self.controller_cell.reset_actuator_steps,
            )

            await self._execute_command(
                self.controller_cell.set_closed_loop_control_mode,
                MTM2.ClosedLoopControlMode.TelemetryOnly,
                timeout=self.COMMAND_TIMEOUT_LONG,
            )

            await self._execute_command(
                self.controller_cell.power,
                MTM2.PowerType.Motor,
                False,
                timeout=self.COMMAND_TIMEOUT_LONG,
            )

        except Exception:
            self.log.exception(
                "Error when doing the basic cleanup and power off the motor."
            )

    async def _switch_force_balance_system(self, status: bool) -> None:
        """Switch the force balance system.

        Parameters
        ----------
        status : `bool`
            True if turn on the force balance system. Otherwise, False.
        """

        # Do not allow the open-loop max limit in the closed-loop control
        if status is True:
            await self._execute_command(
                self.controller_cell.enable_open_loop_max_limit, False
            )

        await self._execute_command(
            self.controller_cell.switch_force_balance_system, status
        )

    async def begin_disable(self, data: salobj.BaseMsgType) -> None:
        # multiply timeout by 3 as this is the number of commands executed with
        # this timeout.
        await self.cmd_disable.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT * 3)

        await super().begin_disable(data)

    async def do_disable(self, data: salobj.BaseMsgType) -> None:
        # Cancel the bump test if exists
        if not self._is_bump_test_done():
            self._task_bump_test.cancel()

        # By doing this, we can avoid the new error code put the system into
        # the Fault state again, which is annoying.
        self.system_is_ready = False
        await asyncio.sleep(self.SLEEP_TIME_SHORT)

        await self._switch_force_balance_system(False)
        await self._basic_cleanup_and_power_off_motor()

        await super().do_disable(data)

    def _is_bump_test_done(self) -> bool:
        """The bump test is done or not.

        Returns
        -------
        `bool`
            True if the bump test is done. Otherwise, False.
        """
        return self._task_bump_test.done()

    async def do_applyForces(self, data: salobj.BaseMsgType) -> None:
        """Apply force.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_closed_loop_control_and_allow_motion()

        await self.cmd_applyForces.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        force_axial = data.axial
        force_tangent = data.tangent
        self._check_applied_forces_in_range(force_axial, force_tangent)

        await self._execute_command(
            self.controller_cell.apply_forces,
            force_axial,
            force_tangent,
        )

    def _assert_closed_loop_control_and_allow_motion(self) -> None:
        """Assert the system is under the closed-loop control and allow the
        motion or not.

        Raises
        -------
        `RuntimeError`
            If the system is not under the closed-loop control.
        `RuntimeError`
            If the system is doing the bump test.
        """

        self.assert_enabled()

        closed_loop = MTM2.ClosedLoopControlMode.ClosedLoop
        if self.controller_cell.closed_loop_control_mode != closed_loop:
            raise RuntimeError(f"System needs to be under {closed_loop!r}.")

        if not self._is_bump_test_done():
            raise RuntimeError("System is still doing the bump test.")

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

        self._assert_closed_loop_control_and_allow_motion()

        await self.cmd_positionMirror.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        await self._execute_command(
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

        self._assert_closed_loop_control_and_allow_motion()

        await self.cmd_resetForceOffsets.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        await self._execute_command(
            self.controller_cell.reset_force_offsets,
        )

    async def do_resetActuatorSteps(self, data: salobj.BaseMsgType) -> None:
        """Resets user defined actuator steps to zeros.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_enabled_and_open_loop_control()

        await self.cmd_resetActuatorSteps.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        await self._execute_command(
            self.controller_cell.command_actuator,
            CommandActuator.Stop,
        )
        await self._execute_command(
            self.controller_cell.reset_actuator_steps,
        )

    def _assert_enabled_and_open_loop_control(self) -> None:
        """Assert the system is in the Enabled state and under the open-loop
        control or not.

        Raises
        -------
        `RuntimeError`
            If the system is not under the open-loop control.
        """

        self.assert_enabled()

        open_loop = MTM2.ClosedLoopControlMode.OpenLoop
        if self.controller_cell.closed_loop_control_mode != open_loop:
            raise RuntimeError(f"System needs to be under {open_loop!r}.")

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

        await self._execute_command(self.controller_cell.clear_errors)

    async def do_selectInclinationSource(self, data: salobj.BaseMsgType) -> None:
        """Command to select source of inclination data.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_disabled()

        await self.cmd_selectInclinationSource.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        source = MTM2.InclinationTelemetrySource(data.source)
        use_mtmount = source == MTM2.InclinationTelemetrySource.MTMOUNT

        max_angle_difference = (
            None if (data.maxDifference == 0.0) else data.maxDifference
        )

        enable_angle_comparison = True if use_mtmount else data.enableComparison

        self.controller_cell.select_inclination_source(
            use_external_elevation_angle=use_mtmount,
            max_angle_difference=max_angle_difference,
            enable_angle_comparison=enable_angle_comparison,
        )

        await self._execute_command(self.controller_cell.set_control_parameters)

    def _assert_disabled(self) -> None:
        """Assert that an action that requires DISABLED state can be run.

        Raises
        ------
        `RuntimeError`
            When the system is not in DISABLED state.
        """

        disabled_state = salobj.State.DISABLED
        if self.summary_state != disabled_state:
            raise RuntimeError(f"Only allowed in the {disabled_state!r}")

    async def do_setTemperatureOffset(self, data: salobj.BaseMsgType) -> None:
        """Command to set temperature offset for the LUT temperature
        correction.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_disabled()

        await self.cmd_setTemperatureOffset.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        await self._execute_command(
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

        self.assert_enabled()

        await self.cmd_switchForceBalanceSystem.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        await self._switch_force_balance_system(data.status)

    async def do_bypassErrorCode(self, data: salobj.BaseMsgType) -> None:
        """Bypass the error code in control loop.

        Notes
        -----
        This command might break the system.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.

        Raises
        ------
        `ValueError`
            When the error code is unrecognized.
        """

        self._assert_disabled()

        await self.cmd_bypassErrorCode.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        error_code = int(data.code)
        available_error_codes = self.controller_cell.error_handler.list_code_total
        if error_code not in available_error_codes:
            raise ValueError(
                f"Unrecognized error code: {error_code}. Available codes are: {available_error_codes}."
            )

        self._error_codes_bypass.add(error_code)

    async def do_resetEnabledFaultsMask(self, data: salobj.BaseMsgType) -> None:
        """Reset the enabled faults mask to default. This will remove all the
        bypassed error codes in control loop.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_disabled()

        await self.cmd_resetEnabledFaultsMask.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        self._error_codes_bypass.clear()

    async def do_setConfigurationFile(self, data: salobj.BaseMsgType) -> None:
        """Set the system configuration file.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.

        Raises
        ------
        `RuntimeError`
            When no available configuration files.
        `ValueError`
            When the configuration file is not allowed.
        """

        self._assert_disabled()

        await self.cmd_setConfigurationFile.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        # Get the current available files in controller
        configuration_files = list()
        error_no_file = "No available configuration files."
        if self.evt_configurationFiles.has_data:
            configuration_files = self.evt_configurationFiles.data.files.split(",")
        else:
            raise RuntimeError(error_no_file)

        if len(configuration_files) <= 1:
            raise RuntimeError(error_no_file)

        # Check the file is available or not
        configuration_file = data.file
        if configuration_file not in configuration_files:
            raise ValueError(
                f"Only the following files are allowed: {configuration_files}."
            )

        await self._execute_command(
            self.controller_cell.set_configuration_file,
            configuration_file,
        )

    async def do_enableOpenLoopMaxLimit(self, data: salobj.BaseMsgType) -> None:
        """Enable the maximum force limit in open-loop control.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_enabled_and_open_loop_control()

        await self.cmd_enableOpenLoopMaxLimit.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        await self._execute_command(
            self.controller_cell.enable_open_loop_max_limit, data.status
        )

    async def do_moveActuator(self, data: salobj.BaseMsgType) -> None:
        """Move the actuator in open-loop control.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.

        Raises
        ------
        `ValueError`
            When both of the displacement and step are not zero.
        """

        self._assert_enabled_and_open_loop_control()

        await self.cmd_moveActuator.ack_in_progress(data, timeout=self.COMMAND_TIMEOUT)

        # micron to mm
        displacement = data.displacement * 1e-3
        step = int(data.step)
        if (displacement != 0.0) and (step != 0):
            raise ValueError(
                "You can only move the displacement or step in a single time."
            )

        actuators = [int(data.actuator)]
        if displacement != 0.0:
            await self._execute_command(
                self.controller_cell.command_actuator,
                CommandActuator.Start,
                actuators=actuators,  # type: ignore[arg-type]
                target_displacement=displacement,
                unit=ActuatorDisplacementUnit.Millimeter,
            )
        elif step != 0:
            await self._execute_command(
                self.controller_cell.command_actuator,
                CommandActuator.Start,
                actuators=actuators,  # type: ignore[arg-type]
                target_displacement=step,  # type: ignore[arg-type]
                unit=ActuatorDisplacementUnit.Step,
            )

    async def do_actuatorBumpTest(self, data: salobj.BaseMsgType) -> None:
        """Exercise the actuator bump test in the closed-loop control, which is
        performed at +/push and -/pull directions.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_closed_loop_control_and_allow_motion()

        await self.cmd_actuatorBumpTest.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        self._task_bump_test = asyncio.create_task(
            self._exercise_actuator_bump_test(
                int(data.actuator), data.force, data.period
            )
        )

    async def _exercise_actuator_bump_test(
        self, actuator: int, force: float, period: float
    ) -> None:
        """Exercise the actuator bump test in the closed-loop control, which is
        performed at +/push and -/pull directions.

        Parameters
        ----------
        actuator : `int`
            0-based actuator ID.
        force : `float`
            Force to apply in Newton.
        period : `float`
            Time period in seconds to hold when the actuator reaches the
            applied force.
        """

        try:
            # Move the +/push direction
            await self._bump_actuator(actuator, force, period)

            # Move the -/pull direction
            await self._bump_actuator(actuator, -force, period)

            # Publish the event that the bump test passes
            await self._publish_status_bump_test(actuator, BumpTest.PASSED)

        except (Exception, asyncio.CancelledError):
            # Publish the event that the bump test fails
            await self._publish_status_bump_test(actuator, BumpTest.FAILED)

            self.log.debug("Bump test task is failed or cancelled.")

    async def _bump_actuator(self, actuator: int, force: float, period: float) -> None:
        """Bump the actuator.

        Parameters
        ----------
        actuator : `int`
            0-based actuator ID.
        force : `float`
            Force to apply in Newton.
        period : `float`
            Time period in seconds to hold when the actuator reaches the
            applied force.

        Raises
        ------
        `RuntimeError`
            When the controller's applied force does not match the request.
        """

        # Check this actuator is axial or tangent, and modify the actuator
        # index if needed
        num_axial_actuator = NUM_ACTUATOR - NUM_TANGENT_LINK
        is_axial = actuator < num_axial_actuator
        actuator_bump = actuator if is_axial else (actuator - num_axial_actuator)

        # Check the forces are in range or not
        force_axial = [0.0] * num_axial_actuator
        force_tangent = [0.0] * NUM_TANGENT_LINK
        if is_axial:
            force_axial[actuator_bump] = force
        else:
            force_tangent[actuator_bump] = force

        self._check_applied_forces_in_range(force_axial, force_tangent)

        # Apply the force and publish the related event
        is_positive_force = force >= 0.0
        if is_positive_force:
            await self._publish_status_bump_test(actuator, BumpTest.TESTINGPOSITIVE)
        else:
            await self._publish_status_bump_test(actuator, BumpTest.TESTINGNEGATIVE)
        await self._execute_command(
            self.controller_cell.apply_forces,
            force_axial,
            force_tangent,
        )

        # Wait for some time and publish the related event
        if is_positive_force:
            await self._publish_status_bump_test(actuator, BumpTest.TESTINGPOSITIVEWAIT)
        else:
            await self._publish_status_bump_test(actuator, BumpTest.TESTINGNEGATIVEWAIT)
        await asyncio.sleep(period)

        # Check the applied force with the first digit accuracy
        data_force_applied = (
            self.tel_axialForce.data.applied
            if is_axial
            else self.tel_tangentForce.data.applied
        )
        is_matched = round(data_force_applied[actuator_bump], 1) == round(force, 1)

        # Reset the force and wait for some time if success, otherwise, abort.
        await self._execute_command(
            self.controller_cell.reset_force_offsets,
        )

        if not is_matched:
            raise RuntimeError(
                "Controller's applied force does not match the request. Abort..."
            )

        await asyncio.sleep(period)

    async def _publish_status_bump_test(self, actuator: int, status: BumpTest) -> None:
        """Publish the status of actuator bump test.

        Parameters
        ----------
        actuator : `int`
            0-based actuator ID.
        status : enum `MTM2.BumpTest`
            Status of the actuator bump test. The current annotation of
            enum "BumpTest" is to maintain the backward compatibility.
        """

        # This is to keep the backward compatibility of ts_xml v20.0.0 that
        # does not have the 'actuatorBumpTestStatus' event defined in xml.
        # TODO: Remove this after ts_xml v20.1.0.
        if hasattr(self, "evt_actuatorBumpTestStatus"):
            await self.evt_actuatorBumpTestStatus.set_write(
                actuator=actuator, status=status.value
            )

    async def _do_killActuatorBumpTest(self, data: salobj.BaseMsgType) -> None:
        """Kill the running actuator bump test in the closed-loop control.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self.assert_enabled()

        if self._is_bump_test_done():
            return

        await self.cmd_killActuatorBumpTest.ack_in_progress(
            data, timeout=self.COMMAND_TIMEOUT
        )

        self.log.info("Killing bump test.")

        self._task_bump_test.cancel()
        await self._task_bump_test

    async def _do_setHardpointList(self, data: salobj.BaseMsgType) -> None:
        """Set the hardpoint list.

        Parameters
        ----------
        data : `object`
            Data of the SAL message.
        """

        self._assert_disabled()

        # Check the hardpoints
        yaml_file = self.config_dir / "harrisLUT" / "cell_geom.yaml"
        cell_geom = read_yaml_file(yaml_file)

        hardpoints = data.actuators
        hardpoints.sort()

        check_hardpoints(
            cell_geom["locAct_axial"],
            hardpoints[:NUM_HARDPOINTS_AXIAL],
            hardpoints[NUM_HARDPOINTS_AXIAL:],
        )

        if self.simulation_mode == 1:
            await self.cmd_setHardpointList.ack_in_progress(
                data, timeout=self.COMMAND_TIMEOUT
            )

            await self._execute_command(
                self.controller_cell.set_hardpoint_list,
                hardpoints,
            )

        else:
            # TODO: Support this after the update of M2 cell LabVIEW code is
            # done.
            raise NotImplementedError("Command is not supported yet.")

    async def _execute_command(
        self,
        command: typing.Coroutine,
        *args: typing.Any,
        timeout: float | None = None,
        **kwargs: dict[str, typing.Any],
    ) -> None:
        """Execute the command to controller.

        Notes
        -----
        This function captures the OSError and sends the CSC to fault.

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
            self.log.exception("Connection error executing command.")
            await self.controller_cell.close()

            if self.disabled_or_enabled:
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
