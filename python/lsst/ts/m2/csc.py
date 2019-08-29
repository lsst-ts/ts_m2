
import pathlib
from lsst.ts import salobj


class M2(salobj.ConfigurableCsc):
    """This is a test CSC for the M2 component with salobj 4.
    """

    def __init__(self, config_dir=None, initial_state=salobj.State.STANDBY,
                 initial_simulation_mode=0):
        schema_path = pathlib.Path(__file__).resolve().parents[4].joinpath("schema", "m2.yaml")
        super().__init__("MTM2", index=0, schema_path=schema_path, config_dir=config_dir,
                         initial_state=initial_state,
                         initial_simulation_mode=initial_simulation_mode)

        self.config = None

    async def do_applyBendingMode(self, id_data):
        """Apply bending mode.
        """
        pass

    async def do_applyForce(self, id_data):
        """Apply force.
        """
        self.config.max_force
        pass

    async def do_setCorrectionMode(self, id_data):
        """Set correction Mode.
        """
        pass

    async def do_positionMirror(self, id_data):
        """Position Mirror.
        """
        pass

    async def do_moveAxialActuator(self, id_data):
        """Move axial actuator."""
        pass

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def configure(self, config):
        self.config = config
