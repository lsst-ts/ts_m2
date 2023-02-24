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

import unittest

from lsst.ts.m2 import Translator


class TestTranslator(unittest.TestCase):
    def setUp(self):
        self.translator = Translator()

    def test_handle_tangent_force(self):
        message = dict(id="tangentForce", lutTemperature=[])

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["lutTemperature"], [0] * 6)

    def test_handle_summary_state(self):
        message = dict(id="summaryState", summaryState=3)

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["id"], "controllerState")
        self.assertEqual(message_payload["controllerState"], 3)


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
