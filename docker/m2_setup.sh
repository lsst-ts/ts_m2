#!/usr/bin/env bash

source ~/miniconda3/bin/activate
source $OSPL_HOME/release.com
source /home/saluser/.bashrc

echo "# Starting M2 CSC"
run_mtm2.py
echo "# CSC exited."