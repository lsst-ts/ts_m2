#!/usr/bin/env bash

# Source this file when starting the container to set it up

echo "#"
echo "# Loading LSST Stack"
. /opt/lsst/software/stack/loadLSST.bash
setup lsst_distrib
echo "#"
echo "# Loading sal environment"
. /home/saluser/repos/ts_sal/setup.env
# Work around for setting LSST_DDS_IP working on most systems
export LSST_DDS_IP=`ip route get 1 | awk '{print $7;exit}'`
echo "#"
echo "# Setting up salobj"

setup ts_m2 -t current

echo "# Starting M2 CSC"
run_mtm2.py
echo "# CSC exited."