import asyncio
import pathlib
from lsst.ts import salobj
import numpy as np

__all__ = ["M2"]


class M2(salobj.ConfigurableCsc):
    """This is a test CSC for the M2 component with salobj.
    """

    def __init__(
        self, config_dir=None, initial_state=salobj.State.STANDBY, simulation_mode=0
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

        self.telemetry_loop_task = None
        self.run_telemetry_loop = False
        self.stop_loop_timeout = 5.0

        self.n_actuators = 72

        self.config = None

    async def handle_summary_state(self):
        """Handle summary state changes.
        """

        if self.disabled_or_enabled and self.telemetry_loop_task is None:
            self.telemetry_loop_task = asyncio.ensure_future(self.telemetry_loop())
        elif self.telemetry_loop_task is not None:
            await self.stop_telemetry_loop()

    async def telemetry_loop(self):

        self.run_telemetry_loop = True

        while self.run_telemetry_loop:
            self.tel_mirrorPositionMeasured.put()
            self.tel_axialForceData.put()
            self.tel_tangentForceData.put()
            self.tel_temperaturesMeasured.put()
            self.tel_zenithAngleData.put()
            self.tel_axialActuatorSteps.put()
            self.tel_axialEncoderPositions.put()
            self.tel_tangentEncoderPositions.put()
            self.tel_ilcData.put()
            self.tel_displacementSensors.put()
            self.tel_netForcesTotal.put()
            self.tel_netMomentsTotal.put()

            await asyncio.sleep(self.heartbeat_interval)

    async def do_applyForces(self, data):
        """Apply force.
        """
        self.assert_enabled()
        self.evt_m2AssemblyInPosition.set_put(inPosition=False)

        self.tel_axialForceData.set(
            axialForcesApplied=data.axialForceSetPoints,
            axialForcesMeasured=data.axialForceSetPoints,
        )

        self.tel_tangentForceData.set(
            tangentForcesApplied=data.tangentForceSetPoints,
            tangentForcesMeasured=data.tangentForceSetPoints,
        )

        await asyncio.sleep(self.heartbeat_interval)
        self.evt_m2AssemblyInPosition.set_put(inPosition=True)

    async def do_positionMirror(self, data):
        """Position Mirror.
        """
        self.assert_enabled()
        self.evt_m2AssemblyInPosition.set_put(inPosition=False)

        self.tel_mirrorPositionMeasured.set(
            **dict(
                [
                    (axis, getattr(data, axis))
                    for axis in ("x", "y", "z", "xRot", "yRot", "zRot")
                ]
            )
        )
        await asyncio.sleep(self.heartbeat_interval)
        self.evt_m2AssemblyInPosition.set_put(inPosition=True)

    async def do_resetForceOffsets(self, data):
        self.assert_enabled()
        self.evt_m2AssemblyInPosition.set_put(inPosition=False)

        self.tel_axialForceData.set(
            axialForcesApplied=np.zeros_like(
                self.tel_axialForceData.data.axialForcesApplied
            ),
            axialForcesMeasured=np.zeros_like(
                self.tel_axialForceData.data.axialForceSetPoints
            ),
        )

        self.tel_tangentForceData.set(
            tangentForcesApplied=np.zeros_like(
                self.tel_tangentForceData.data.tangentForcesApplied
            ),
            tangentForcesMeasured=np.zeros_like(
                self.tel_tangentForceData.data.tangentForcesMeasured
            ),
        )

        await asyncio.sleep(self.heartbeat_interval)
        self.evt_m2AssemblyInPosition.set_put(inPosition=True)

    async def do_clearErrors(self, data):
        raise NotImplementedError("Command not implemented.")

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def configure(self, config):
        self.config = config

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
        """Disconnect from the TCP/IP controller, if connected, and stop
        the mock controller, if running.
        """
        try:
            await self.stop_telemetry_loop()
        except Exception:
            pass
        finally:
            await super().close_tasks()
