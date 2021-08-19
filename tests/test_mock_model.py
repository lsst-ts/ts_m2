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

import numpy as np
import unittest

from lsst.ts.idl.enums import MTM2

from lsst.ts.m2 import MockModel, get_module_path


class TestMockModel(unittest.TestCase):
    """Test the Mock Model class."""

    def setUp(self):

        self.model = MockModel()

        config_dir = get_module_path() / "tests"
        self.model.configure(config_dir, "harrisLUT")

    def test_configure(self):

        self.assertEqual(len(self.model.lut.keys()), 10)

    def test_is_cell_temperature_high(self):

        # Temperature is normal
        self.assertFalse(self.model.is_cell_temperature_high())

        # Temperature is high
        self.model.temperature["exhaust"] = [99, 99]

        self.assertTrue(self.model.is_cell_temperature_high())

    def test_fault_actuator_power_not_on(self):

        self.model.fault()
        self.assertFalse(self.model.error_cleared)
        self.assertFalse(self.model.force_balance_system_status)

    def test_fault_actuator_power_on(self):

        # Turn on the power and the switch should work
        self.model.actuator_power_on = True
        self.assertTrue(self.model.switch_force_balance_system(True))
        self.assertTrue(self.model.force_balance_system_status)

        # Fault the model
        self.model.fault()
        self.assertFalse(self.model.error_cleared)
        self.assertFalse(self.model.force_balance_system_status)

    def test_switch_force_balance_system(self):

        # This should fail
        self.assertFalse(self.model.switch_force_balance_system(True))
        self.assertFalse(self.model.force_balance_system_status)

        # Turn on the power and the switch should work
        self.model.actuator_power_on = True
        self.assertTrue(self.model.switch_force_balance_system(True))
        self.assertTrue(self.model.force_balance_system_status)

        # Switch off the force balance system
        self.assertTrue(self.model.switch_force_balance_system(False))
        self.assertFalse(self.model.force_balance_system_status)

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

    def test_get_telemetry_data(self):

        # No power
        telemetry_data = self.model.get_telemetry_data()

        self.assertFalse(self.model.in_position)
        self.assertEqual(len(telemetry_data), 2)

        # With power
        self.model.actuator_power_on = True
        self.model.switch_force_balance_system(True)
        force_axial, force_tangent = self._apply_forces()

        telemetry_data = self.model.get_telemetry_data()

        self.assertFalse(self.model.in_position)
        self.assertEqual(len(telemetry_data), 16)

    def test_get_ilc_data(self):

        for idx in range(30):
            ilc_status = self.model._get_ilc_data()["status"]
            self.assertEqual(ilc_status[0], idx % 16)

    def test_get_displacement_sensors(self):

        # Zero position
        displacement_ims = self.model._get_displacement_sensors()

        position = self._calc_position_ims(displacement_ims)

        np.testing.assert_array_almost_equal(
            position, list(self.model.mirror_position.values())
        )

        # Non-zero position
        mirror_position = {
            "x": 10,
            "y": 20,
            "z": 30,
            "xRot": 40,
            "yRot": 50,
            "zRot": 60,
        }
        displacement_ims = self.model._get_displacement_sensors(
            mirror_position=mirror_position
        )

        position = self._calc_position_ims(displacement_ims)

        np.testing.assert_array_almost_equal(position, list(mirror_position.values()))

    def _calc_position_ims(self, displacement_ims):
        """Calculate the position based on the reading of IMS.

        Parameters
        ----------
        displacement_ims : `dict`
            Data of displacement sensors.

        Returns
        -------
        `numpy.ndarray`
            Calculated mirror position based on IMS.
        """

        # Get the position with the same order of matrix and offset
        # The unit needs to change to mm from um
        theta_z = np.array(displacement_ims["thetaZ"])
        delta_z = np.array(displacement_ims["deltaZ"])
        position_ims = np.append(theta_z, delta_z) * 1e-3

        index_array = np.array([8, 10, 4, 6, 0, 2, 9, 11, 5, 7, 1, 3])
        position_ims_update = position_ims[index_array.argsort()]

        matrix = np.array(self.model._disp_ims["matrix"])
        offset = np.array(self.model._disp_ims["offset"])

        return matrix.dot((position_ims_update - offset).reshape(-1, 1)).ravel()

    def test_get_net_forces_total(self):

        self.model.axial_forces["measured"][1:3] = np.array([1, 2])
        self.model.tangent_forces["measured"] = np.array([1, 2, 3, 4, 5, 6])

        net_forces_total = self.model._get_net_forces_total()

        self.assertAlmostEqual(net_forces_total["fx"], -3)
        self.assertAlmostEqual(net_forces_total["fy"], -5.1961524)
        self.assertAlmostEqual(net_forces_total["fz"], 3)

    def test_get_net_moments_total(self):

        self.model.axial_forces["measured"][1:4] = np.array([1, 2, 3])
        self.model.tangent_forces["measured"] = np.array([1, 2, 3, 4, 5, 6])

        net_moments_total = self.model._get_net_moments_total()

        self.assertAlmostEqual(net_moments_total["mx"], 8.37691)
        self.assertAlmostEqual(net_moments_total["my"], 4.45836999)
        self.assertAlmostEqual(net_moments_total["mz"], 37.3839844)

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

        # This should fail
        self.assertFalse(self.model.handle_position_mirror(mirror_position_set_point))

        # This should fail again
        self.model.actuator_power_on = True
        self.assertFalse(self.model.handle_position_mirror(mirror_position_set_point))

        # This should succeed in the final
        self.model.switch_force_balance_system(True)
        result = self.model.handle_position_mirror(mirror_position_set_point)

        self.assertTrue(result)
        self.assertEqual(self.model.mirror_position, mirror_position_set_point)


if __name__ == "__main__":

    # Do the unit test
    unittest.main()
