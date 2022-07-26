[![docs](https://img.shields.io/badge/docs-ts--m2.lsst.io-brightgreen)](https://ts-m2.lsst.io/)

# M2 Commandable SAL Component (CSC)

This package is the commandable SAL component (CSC) and provides a simple emulator for the M2 control system.

## Installation instructions

This is a pure `eups` package.
To install, first clone and setup the configuration repo `ts_config_mttcs`:

```bash
git clone https://github.com/lsst-ts/ts_config_mttcs.git
cd ts_config_mttcs/
eups declare -r . -t $USER
```

Then do the same with ``ts_m2``.

```bash
git clone https://github.com/lsst-ts/ts_m2.git
cd ts_m2/
eups declare -r . -t $USER
setup ts_m2 -t $USER
scons
```

The final line (`scons`) will run the build and installation process.
