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

__all__ = ["CONFIG_SCHEMA"]

import yaml

CONFIG_SCHEMA = yaml.safe_load(
    """
$schema: http://json-schema.org/draft-07/schema#
$id: https://github.com/lsst-ts/ts_m2/blob/main/python/lsst/ts/m2/config_schema.py
# title must end with one or more spaces followed by the schema version, which must begin with "v"
title: M2 v2
description: Schema for M2 configuration files
type: object
properties:
  lut_path:
    description: >-
      Relative location of the Look-Up Tables, with respect to configuration
      package.
    type: string
  host:
    description: >-
      IP address of the TCP/IP interface.
    type: string
    format: hostname
  port_command:
    description: >-
      Command port number of the TCP/IP interface.
    type: integer
  port_telemetry:
    description: >-
      Telemetry port number of the TCP/IP interface.
    type: integer
  configuration_file:
    description: >-
      System configuration file. This should be in the "configurationFiles"
      event defined in the ts_xml.
    type: string
  hardpoints:
    description: >-
      0-based hardpoints. The index should be from low to high.
    type: array
    items:
      type: integer
      minItems: 6
      maxItems: 6
  inclination_source:
    description: >-
        Inclination source used in the look-up table calculation:
        1: OnBoard. 2: MTMount. See the enum 'MTM2.InclinationTelemetrySource'
        in ts_xml.
        If the MTMount is used, the angles between the onboard and MTMount will
        always be compared.
    type: integer
  inclination_enable_comparison:
    description: >-
        True if enable the comparison between the onboard and MTMount angles.
        Otherwise, False. Under the comparison, if the difference is greater
        than the "inclination_max_difference" or default threshold, the system
        will fault itself to protect the mirror if you do not bypass the
        related error code. If the source is MTMount, this value should be
        True.
    type: boolean
  inclination_max_difference:
    description: >-
        Maximum angle difference between the onboard and MTMount angles. This
        value should be greater than 0. If 0 is used, the default value will be
        applied.
    type: number
"""
)
