#!/bin/bash
cd "$(dirname "$0")"

export YAKUT_COMPILE_OUTPUT=compile_output
REG_DATA_TYPE_PATH_UAVCAN=public_regulated_data_types/uavcan
REG_DATA_TYPE_PATH_REG=public_regulated_data_types/reg

export YAKUT_PATH="$YAKUT_COMPILE_OUTPUT"
export REG_DATA_TYPE_PATH="$REG_DATA_TYPE_PATH_UAVCAN $REG_DATA_TYPE_PATH_REG"

git clone https://github.com/OpenCyphal/public_regulated_data_types.git
yakut compile $REG_DATA_TYPE_PATH -O $YAKUT_COMPILE_OUTPUT
