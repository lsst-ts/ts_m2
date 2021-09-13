#######################
M2 Configuration
#######################

M2 high-level configuration is handled using the yaml file located in the `ts_config_mttcs <https://github.com/lsst-ts/ts_config_mttcs/tree/develop/MTM2>`_.
Of particular interests are the look-up table (LUT), cell geomoetry, and independent measurement system (IMS) settings, which should be changed to match the observatory configuration.
The values of LUT will affect the closed-loop control directly.

More configuration parameters can be found in a *schema* available at `ts_m2 <https://github.com/lsst-ts/ts_m2/blob/develop/python/lsst/ts/m2/config_schema.py>`_.
For example, you could modify the path to LUT to test the closed-loop control under different parameter space.
This is useful in the commissioning stage to optimize the performance with other equipments such as the M1M3, camera hexapod, M2 hexapod, etc. with the calibration of real data.
The detailed analysis and test of M2 can follow: `M2_FEA <https://github.com/lsst-sitcom/M2_FEA>`_ and `SCTR-21 <https://github.com/lsst-sitcom/SCTR-21>`_.
