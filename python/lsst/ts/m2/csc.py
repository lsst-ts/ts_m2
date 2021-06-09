# This file is part of ts_m2.
#
# Developed for the LSST Data Management System.
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
import concurrent
import pathlib
import logging
import argparse
import traceback

from lsst.ts import salobj
from lsst.ts.idl.enums import MTM2

from . import Model

__all__ = ["M2"]

# Error code for when telemetry loop dies while in ENABLE or DISABLE state.
TELEMETRY_LOOP_DIED = 101

# Error code for when it fails to reposition the position.
POSITION_MIRROR_ERROR = 102


class M2(salobj.ConfigurableCsc):
    """This is a test CSC for the M2 component with salobj."""

    def __init__(
        self,
        config_dir=None,
        initial_state=salobj.State.STANDBY,
        simulation_mode=0,
        verbose=False,
    ):
        schema_path = (
            pathlib.Path(__file__).resolve().parents[4].joinpath("schema", "m2.yaml")
        )
        super().__init__(
            "MTM2",
            index=0,
            schema_path=schema_path,
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

        if verbose:
            stream_handler = logging.StreamHandler(sys.stdout)
            self.log.addHandler(stream_handler)
            self.log.setLevel(logging.DEBUG)

        # Model class to do the simulation
        self.model = Model(log=self.log, telemetry_interval=self.heartbeat_interval)

        self.telemetry_loop_task = None
        self.run_telemetry_loop = False
        self.stop_loop_timeout = 5.0

        self.config = None

        # Remote to listen to MTMount position
        self.mtmount = salobj.Remote(self.domain, "MTMount", include=["elevation"])

        self.mtmount.tel_elevation.callback = self.set_mount_elevation_callback

        self.evt_inclinationTelemetrySource.set(source=self.model.inclination_source)

        self.evt_forceBalanceSystemStatus.set(
            status=self.model.force_balance_system_status
        )

        self.evt_tcpIpConnected.set(isConnected=True)
        self.evt_interlock.set(state=False)
        self.evt_commandableByDDS.set(state=True)
        self.evt_cellTemperatureHiWarning.set(hiWarning=False)

        self.tel_zenithAngle.set(
            measured=3.5,
            inclinometerRaw=3.5,
            inclinometerProcessed=3.5,
        )

        self.tel_temperature.set(
            ring=self.model.temperature["ring"],
            intake=self.model.temperature["intake"],
            exhaust=self.model.temperature["exhaust"],
        )

    def set_mount_elevation_callback(self, data):
        """Callback function to set the mount elevation.

        Parameters
        ----------
        data : `object`
            Data for the elevation telemetry of mount.
        """

        # The actualPosition is the elevation angle in degree
        self.model.zenith_angle = 90.0 - data.actualPosition

    async def handle_summary_state(self):
        """Handle summary state changes."""

        self.log.debug(f"Handle summary state: {self.disabled_or_enabled}")

        if not self.model.error_cleared:
            raise RuntimeError(
                "Error not cleared. Send clearError command before sending component to standby."
            )
        elif self.disabled_or_enabled and self.telemetry_loop_task is None:
            self.log.debug("Starting telemetry loop task.")
            self.telemetry_loop_task = asyncio.create_task(self.telemetry_loop())
        elif not self.disabled_or_enabled and self.telemetry_loop_task is not None:
            self.log.debug("Closing telemetry loop task.")
            await self.stop_telemetry_loop()

    async def telemetry_loop(self):
        """Update and output telemetry information from component."""

        self.run_telemetry_loop = True

        self.evt_inclinationTelemetrySource.put()
        self.evt_forceBalanceSystemStatus.put()
        self.evt_tcpIpConnected.put()
        self.evt_interlock.put()
        self.evt_commandableByDDS.put()
        self.evt_cellTemperatureHiWarning.put()

        self.log.debug("Starting telemetry loop.")

        while self.run_telemetry_loop:

            try:

                self.tel_displacementSensors.put()
                self.tel_ilcData.put()

                self.tel_netForcesTotal.put()
                self.tel_netMomentsTotal.put()

                self.tel_powerStatus.put()

                # Get the telemetry data from model
                telemetry_data, in_position = self.model.get_telemetry_data()
                self.tel_axialForce.set_put(**telemetry_data["axialForce"])
                self.tel_tangentForce.set_put(**telemetry_data["tangentForce"])
                self.tel_forceBalance.set_put(**telemetry_data["forceBalance"])

                self.tel_position.set_put(**telemetry_data["position"])
                self.tel_positionIMS.set_put(**telemetry_data["positionIMS"])

                self.tel_temperature.set_put(**telemetry_data["temperature"])

                self.tel_zenithAngle.set_put(**telemetry_data["zenithAngle"])

                self.tel_axialActuatorSteps.set_put(
                    **telemetry_data["axialActuatorSteps"]
                )
                self.tel_axialEncoderPositions.set_put(
                    **telemetry_data["axialEncoderPositions"]
                )

                self.tel_tangentEncoderPositions.set_put(
                    **telemetry_data["tangentEncoderPositions"]
                )
                self.tel_tangentActuatorSteps.set_put(
                    **telemetry_data["tangentActuatorSteps"]
                )

                self.evt_m2AssemblyInPosition.set_put(inPosition=in_position)

                await asyncio.sleep(self.model.telemetry_interval)
            except Exception:
                self.log.exception("Exception in telemetry loop.")
                self.fault(
                    code=TELEMETRY_LOOP_DIED,
                    report="Exception in telemetry loop.",
                    traceback=traceback.format_exc(),
                )

        self.log.debug(f"Telemetry loop closed. {self.run_telemetry_loop}")

    async def do_applyForces(self, data):
        """Apply force."""
        self.assert_enabled()

        self.model.apply_forces(data.axial, data.tangent)

        self.evt_m2AssemblyInPosition.set_put(inPosition=False)

    async def do_positionMirror(self, data):
        """Position Mirror."""
        self.assert_enabled()

        self.evt_m2AssemblyInPosition.set_put(inPosition=False)

        try:
            mirror_position_set_point = dict(
                [
                    (axis, getattr(data, axis))
                    for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
                ]
            )
            await self._run_task_in_new_event_loop(
                self.model.handle_position_mirror, mirror_position_set_point
            )

            self.evt_m2AssemblyInPosition.set_put(inPosition=True)

        except RuntimeError:
            self.fault(
                code=POSITION_MIRROR_ERROR,
                report="Failed to position the mirror.",
                traceback="",
            )

    async def _run_task_in_new_event_loop(self, func, *args):
        """Run the task in the new event loop.

        Parameters
        ----------
        func : object
            Function to put in the new event loop.
        *args : any
            Arguments needed in function.
        """

        executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(executor, func, *args)

    async def do_resetForceOffsets(self, data):
        """Resets user defined forces to zeros."""
        self.assert_enabled()

        self.model.reset_force_offsets()

        self.evt_m2AssemblyInPosition.set_put(inPosition=False)

    async def do_clearErrors(self, data):
        """Emulate clearError command.

        Raises
        ------
        RuntimeError
            clearErrors command only valid in FAULT state.
        """

        if self.summary_state != salobj.State.FAULT:
            raise RuntimeError("clearErrors command only valid in FAULT state.")

        self.model.clear_errors()

        await asyncio.sleep(self.heartbeat_interval)

    async def do_selectInclinationSource(self, data):
        """Command to select source of inclination data."""
        try:
            self.model.select_inclination_source(data.source)

            self.evt_inclinationTelemetrySource.set_put(
                source=self.model.inclination_source
            )
        except ValueError:
            raise RuntimeError(
                f"Command rejected. Invalid inclination source {data.source}. "
                f"Must be one of {[value for value in MTM2.InclinationTelemetrySource]}"
            )

    async def do_setTemperatureOffset(self, data):
        """Command to set temperature offset for the LUT temperature
        correction.
        """

        self.model.set_temperature_offset()

    async def do_switchForceBalanceSystem(self, data):
        """Command to switch force balance system on and off."""

        if self.model.force_balance_system_status == data.status:
            raise RuntimeError(
                f"Command rejected. Force balance system status already {data.status}."
            )

        self.model.force_balance_system_status = data.status

        self.evt_forceBalanceSystemStatus.set_put(status=data.status)

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def configure(self, config):
        """Configure CSC.

        Parameters
        ----------
        config : `types.SimpleNamespace`
            Namespace with configuration values.
        """

        self.config = config

        lut_path = self.config.lut_path
        self.log.debug(f"LUT directory: {lut_path}.")

        self.model.configure(self.config_dir, lut_path)

    async def stop_telemetry_loop(self):

        self.run_telemetry_loop = False
        try:
            await asyncio.wait_for(
                self.telemetry_loop_task, timeout=self.stop_loop_timeout
            )
        except asyncio.TimeoutError:
            self.log.debug("Timed out waiting for telemetry loop to finish. Canceling.")
            self.telemetry_loop_task.cancel()
            try:
                await self.telemetry_loop_task
            except asyncio.CancelledError:
                self.log.debug("Telemetry loop cancelled.")
        finally:
            self.telemetry_loop_task = None

    async def close_tasks(self):
        """Stop telemetry loop."""

        try:
            await self.stop_telemetry_loop()
        except Exception:
            self.log.exception("Exception while stopping telemetry loop.")
        finally:
            await super().close_tasks()

    def fault(self, code=None, report="", traceback=""):
        """Enter the fault state and output the ``errorCode`` event.

        Override parent method to stop telemetry loop before going to FAULT.

        Parameters
        ----------
        code : `int` (optional)
            Error code for the ``errorCode`` event.
            If `None` then ``errorCode`` is not output and you should
            output it yourself. Specifying `None` is deprecated;
            please always specify an integer error code.
        report : `str` (optional)
            Description of the error.
        traceback : `str` (optional)
            Description of the traceback, if any.
        """
        self.model.error_cleared = False
        self.run_telemetry_loop = False

        self.model.force_balance_system_status = False
        self.evt_forceBalanceSystemStatus.set_put(status=False)

        # Resets data
        self.tel_axialForce.data = self.tel_axialForce.DataType()
        self.tel_tangentForce.data = self.tel_tangentForce.DataType()

        super().fault(code=code, report=report, traceback=traceback)

    @classmethod
    async def amain(cls):
        """Make a CSC from command-line arguments and run it."""
        parser = argparse.ArgumentParser(f"Run {cls.__name__}")
        parser.add_argument(
            "-v",
            "--verbose",
            action="store_true",
            help="Run in verbose mode?",
            default=False,
        )

        args = parser.parse_args()
        csc = cls(verbose=args.verbose)
        await csc.done_task
