#####
ts_m2
#####

This package provide a simple emulator for the M2 control system. It is
intended to be used for software-only integration and tests.

Installation instructions
=========================

This is a pure ``eups`` package. To install, first clone and setup the
configuration repo ``ts_config_mttcs``;

``````
git clone https://github.com/lsst-ts/ts_config_mttcs.git
cd ts_config_mttcs/
eups declare -r . -t $USER
``````

Then do the same with ``ts_m2``.
``````
git clone https://github.com/lsst-ts/ts_m2.git
cd ts_m2/
eups declare -r . -t $USER
setup ts_m2 -t $USER
scons
``````

The final line (``scons``) will run the build and installation process.
