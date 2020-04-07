
import pathlib
import unittest
import asynctest

from lsst.ts import salobj
from lsst.ts.m2 import M2

TEST_CONFIG_DIR = pathlib.Path(__file__).resolve().parent.joinpath("data", "config")


class TestM2CSC(salobj.BaseCscTestCase, asynctest.TestCase):

    def basic_make_csc(self, initial_state, config_dir, simulation_mode):
        return M2(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode
        )

    async def test_standard_state_transitions(self):
        async with self.make_csc(
                initial_state=salobj.State.STANDBY,
                config_dir=None,
                simulation_mode=0
        ):
            await self.check_standard_state_transitions(
                enabled_commands=(
                    "applyForces",
                    "positionMirror",
                    "resetForceOffsets",
                    "clearErrors"
                )
            )


if __name__ == "__main__":
    unittest.main()
