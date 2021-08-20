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

import logging
import unittest
import asyncio

from lsst.ts.m2 import check_queue_size


class TestUtility(unittest.IsolatedAsyncioTestCase):
    """Test the functions in utility."""

    def test_check_queue_size(self):

        # No information is logged
        queue = asyncio.Queue(maxsize=3)
        log = logging.getLogger()
        self.assertFalse(check_queue_size(queue, log))

        queue.put_nowait(1)
        self.assertFalse(check_queue_size(queue, log))

        # Information is logged
        queue.put_nowait(2)
        self.assertTrue(check_queue_size(queue, log))


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
