#!/bin/bash
# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2022-2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

CRNT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_DIR="$(dirname "$CRNT_DIR")"
YAKUT_COMPILE_OUTPUT=$REPO_DIR/compile_output

cd $REPO_DIR

DSDL_PATH=$REPO_DIR/Libs/cyphal_application/Libs/public_regulated_data_types
yakut compile $DSDL_PATH/uavcan $DSDL_PATH/reg -O $YAKUT_COMPILE_OUTPUT

ZUBAX_DSDL_PATH=$REPO_DIR/Libs/cyphal_application/Libs/zubax_dsdl
git clone https://github.com/Zubax/zubax_dsdl.git ${ZUBAX_DSDL_PATH}
cd ${ZUBAX_DSDL_PATH} && git checkout b14f6c8 && cd ${REPO_DIR}
yakut compile $DSDL_PATH/uavcan $ZUBAX_DSDL_PATH/zubax -O $YAKUT_COMPILE_OUTPUT
