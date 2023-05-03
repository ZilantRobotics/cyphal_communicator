#!/bin/bash
REPO_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DSDL_PATH=$REPO_DIR/public_regulated_data_types
ZUBAX_DSDL_PATH=$REPO_DIR/zubax_dsdl
YAKUT_COMPILE_OUTPUT=$REPO_DIR/compile_output

cd $REPO_DIR

git clone https://github.com/OpenCyphal/public_regulated_data_types.git
cd public_regulated_data_types && git checkout 935973b && cd ..
yakut compile $DSDL_PATH/uavcan $DSDL_PATH/reg -O $YAKUT_COMPILE_OUTPUT

git clone https://github.com/Zubax/zubax_dsdl.git
cd zubax_dsdl && git checkout b14f6c8 && cd ..
yakut compile $DSDL_PATH/uavcan $ZUBAX_DSDL_PATH/zubax -O $YAKUT_COMPILE_OUTPUT
