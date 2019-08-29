
import unittest
import pathlib
import asyncio

from lsst.ts import salobj
from lsst.ts.m2 import M2

TEST_CONFIG_DIR = pathlib.Path(__file__).resolve().parent.joinpath("data", "config")


class Harness:
    def __init__(self, initial_state=salobj.State.STANDBY,
                 config_dir=None,
                 initial_simulation_mode=0):

        salobj.test_utils.set_random_lsst_dds_domain()

        self.csc = M2(config_dir=config_dir,
                      initial_state=initial_state,
                      initial_simulation_mode=initial_simulation_mode)

        self.remote = salobj.Remote(self.csc.domain, "MTM2")

    async def __aenter__(self):
        await asyncio.gather(self.csc.start_task,
                             self.remote.start_task)
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await asyncio.gather(self.csc.close(),
                             self.remote.close())


class TestM2CSC(unittest.TestCase):

    def test_basic_state_transition(self):

        async def doit():
            async with Harness(config_dir=TEST_CONFIG_DIR) as harness:

                evt_timeout = 5.

                state = await harness.remote.evt_summaryState.next(flush=False,
                                                                   timeout=evt_timeout)

                self.assertEqual(salobj.State(state.summaryState),
                                 salobj.State.STANDBY)

        asyncio.get_event_loop().run_until_complete(doit())


if __name__ == "__main__":
    unittest.main()
