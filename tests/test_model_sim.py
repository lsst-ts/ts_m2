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

import pathlib
import numpy as np
import unittest

from lsst.ts.idl.enums import MTM2

from lsst.ts.m2 import ModelSim


class TestModelSim(unittest.TestCase):
    """Test the Model Simulation class."""

    def setUp(self):

        self.model = ModelSim()

        config_dir = pathlib.Path(__file__).parents[0]

        self.model.configure(config_dir, "harrisLUT")

    def test_configure(self):

        self.assertEqual(len(self.model.lut.keys()), 10)

    def test_apply_forces(self):

        force_axial, force_tangent = self._apply_forces()

        np.testing.assert_array_equal(self.model.axial_forces["applied"], force_axial)
        np.testing.assert_array_equal(
            self.model.tangent_forces["applied"], force_tangent
        )

    def _apply_forces(self):

        force_axial = [1] * self.model.n_actuators
        force_tangent = [2] * self.model.n_tangent_actuators
        self.model.apply_forces(force_axial, force_tangent)

        return force_axial, force_tangent

    def test_check_axial_force_limit(self):

        force_axial = self._apply_forces()[0]
        demanded_axial_force = self.model.check_axial_force_limit()

        np.testing.assert_array_equal(demanded_axial_force, force_axial)

    def test_check_axial_force_limit_error(self):

        force_axial = [0] * self.model.n_actuators
        force_axial[2] = 999

        self.assertRaises(RuntimeError, self.model.check_axial_force_limit, force_axial)

    def test_check_tangent_force_limit(self):

        force_tangent = self._apply_forces()[1]
        demanded_tanget_force = self.model.check_tangent_force_limit()

        np.testing.assert_array_equal(demanded_tanget_force, force_tangent)

    def test_check_tangent_force_limit_error(self):

        force_tangent = [0] * self.model.n_tangent_actuators
        force_tangent[2] = 9999

        self.assertRaises(
            RuntimeError, self.model.check_tangent_force_limit, force_tangent
        )

    def test_reset_force_offsets(self):

        self._apply_forces()

        self.model.reset_force_offsets()

        np.testing.assert_array_equal(
            self.model.axial_forces["applied"], [0] * self.model.n_actuators
        )
        np.testing.assert_array_equal(
            self.model.tangent_forces["applied"], [0] * self.model.n_tangent_actuators
        )

    def test_clear_errors(self):

        self.model.error_cleared = False
        self.model.clear_errors()

        self.assertTrue(self.model.error_cleared)

    def test_select_inclination_source(self):

        self.assertEqual(
            self.model.inclination_source, MTM2.InclinationTelemetrySource.ONBOARD
        )

        self.model.select_inclination_source(2)

        self.assertEqual(
            self.model.inclination_source, MTM2.InclinationTelemetrySource.MTMOUNT
        )

    def test_set_temperature_offset(self):

        self.assertRaises(NotImplementedError, self.model.set_temperature_offset)

    def test_get_telemetry_data(self):

        self.model.force_balance_system_status = True
        force_axial, force_tangent = self._apply_forces()

        telemetry_data, in_position = self.model.get_telemetry_data()

        self.assertFalse(in_position)
        self.assertEqual(len(telemetry_data), 11)

    def test_handle_forces(self):

        self.model.force_balance_system_status = True
        force_axial, force_tangent = self._apply_forces()

        force_rms = 0.5
        in_position = self.model.handle_forces(force_rms=0.5)

        self.assertFalse(in_position)

        # Check the axial forces
        self.assertNotEqual(
            np.sum(np.abs(self.model.axial_forces["hardpointCorrection"])), 0
        )
        self.assertLess(
            np.std(self.model.axial_forces["hardpointCorrection"]), 3 * force_rms
        )
        self.assertNotEqual(np.sum(np.abs(self.model.axial_forces["measured"])), 0)
        self.assertLess(np.std(self.model.axial_forces["measured"]), 3 * force_rms)

        # Check the tangent forces
        self.assertNotEqual(
            np.sum(np.abs(self.model.tangent_forces["hardpointCorrection"])), 0
        )
        self.assertLess(
            np.std(self.model.tangent_forces["hardpointCorrection"]), 3 * force_rms
        )
        self.assertNotEqual(np.sum(np.abs(self.model.tangent_forces["measured"])), 0)
        self.assertLess(np.std(self.model.tangent_forces["measured"]), 3 * force_rms)

    def test_calc_look_up_forces(self):

        self.model.zenith_angle = 10
        self.model.calc_look_up_forces()

        self.assertAlmostEqual(self.model.axial_forces["lutTemperature"][0], -9.683872)
        self.assertAlmostEqual(self.model.axial_forces["lutGravity"][0], 319.58224)

        self.assertAlmostEqual(self.model.tangent_forces["lutGravity"][0], 0)
        self.assertAlmostEqual(self.model.tangent_forces["lutGravity"][1], 780.89259849)

    def test_force_dynamics_in_position(self):

        demand = np.array([1, 2])
        current = demand.copy()
        force_rms = 0.5
        in_position, final_force = self.model.force_dynamics(
            demand, current, force_rms, force_rate=100.0
        )

        self.assertTrue(in_position)
        np.testing.assert_array_equal(final_force, demand)

    def test_force_dynamics_not_in_position(self):

        demand = np.array([1, 2])
        current = np.array([5, -5])
        force_rms = 0.5
        in_position, final_force = self.model.force_dynamics(
            demand, current, force_rms, force_rate=100.0
        )

        self.assertFalse(in_position)
        np.testing.assert_array_equal(final_force, [1, 0])

    def test_handle_position_mirror(self):

        mirror_position_set_point = dict(
            [(axis, 1.0) for axis in ("x", "y", "z", "xRot", "yRot", "zRot")]
        )

        self.model.handle_position_mirror(mirror_position_set_point)

        self.assertEqual(self.model.mirror_position, mirror_position_set_point)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
