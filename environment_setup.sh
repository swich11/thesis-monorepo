#!/bin/bash

# grab the venv
source .venv/bin/activate


# add isaacsim packages to python path for linting
SCRIPT_DIR="~/isaacsim"
export PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/../../../$PYTHONPATH:$SCRIPT_DIR/kit/python/lib/python3.10:$SCRIPT_DIR/kit/python/lib/python3.10/site-packages:$SCRIPT_DIR/python_packages:$SCRIPT_DIR/exts/isaacsim.simulation_app:$SCRIPT_DIR/extsDeprecated/omni.isaac.kit:$SCRIPT_DIR/kit/kernel/py:$SCRIPT_DIR/kit/plugins/bindings-python:$SCRIPT_DIR/exts/isaacsim.robot_motion.lula/pip_prebundle:$SCRIPT_DIR/exts/isaacsim.asset.exporter.urdf/pip_prebundle:$SCRIPT_DIR/extscache/omni.kit.pip_archive-0.0.0+d02c707b.lx64.cp310/pip_prebundle:$SCRIPT_DIR/exts/omni.isaac.core_archive/pip_prebundle:$SCRIPT_DIR/exts/omni.isaac.ml_archive/pip_prebundle:$SCRIPT_DIR/exts/omni.pip.compute/pip_prebundle:$SCRIPT_DIR/exts/omni.pip.cloud/pip_prebundle