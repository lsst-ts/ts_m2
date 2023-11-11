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

from enum import IntEnum, auto

__all__ = ["ErrorCode", "BumpTest"]


class ErrorCode(IntEnum):
    ControllerInFault = 1
    NoConnection = auto()
    InterlockEngaged = auto()


# This is to keep the backward compatibility of ts_xml v20.0.0 that
# does not have the 'actuatorBumpTestStatus' event defined in xml.
# TODO: Remove this after ts_xml v20.1.0.
class BumpTest(IntEnum):
    NOTTESTED = 1
    TESTINGPOSITIVE = 2
    TESTINGPOSITIVEWAIT = 3
    TESTINGNEGATIVE = 4
    TESTINGNEGATIVEWAIT = 5
    PASSED = 6
    FAILED = 7
