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
    def setUp(self) -> None:
        self.translator = Translator()

    def test_handle_default(self) -> None:
        message = dict(id="default", value=1)

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload, message)

    def test_handle_tangent_force(self) -> None:
        message = dict(id="tangentForce", lutTemperature=[])

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["lutTemperature"], [0] * 6)

    def test_handle_config(self) -> None:
        message = dict(
            id="config",
            timeoutSal=1.0,
            timeoutCrio=1.0,
            timeoutIlc=1.0,
            inclinometerDelta=1.0,
            inclinometerDiffEnabled=False,
        )

        message_payload = self.translator.translate(message)

        self.assertEqual(len(message_payload.keys()), 1)

    def test_handle_configuration_files(self) -> None:
        # Multiple files
        message = dict(id="configurationFiles", files=["a", "b"])

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["files"], "a,b")

        # Single file
        message = dict(id="configurationFiles", files=["a"])

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["files"], "a")

        # No file
        message = dict(id="configurationFiles", files=[])

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["files"], "")

    def test_handle_digital_input(self) -> None:
        message = dict(id="digitalInput", value=16)

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["value"], "0x10")

    def test_handle_summary_faults_status(self) -> None:
        message = dict(id="summaryFaultsStatus", status=16)

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["status"], "0x10")

    def test_handle_enabled_faults_mask(self) -> None:
        message = dict(id="enabledFaultsMask", mask=16)

        message_payload = self.translator.translate(message)

        self.assertEqual(message_payload["mask"], "0x10")


if __name__ == "__main__":
    # Do the unit test
    unittest.main()
