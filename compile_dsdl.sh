#!/bin/bash
cd "$(dirname "$0")"
source scripts/config.sh
git clone https://github.com/OpenCyphal/public_regulated_data_types.git
cd public_regulated_data_types && git checkout 1565632 && cd ..

yakut compile $REG_DATA_TYPE_PATH -O $YAKUT_COMPILE_OUTPUT
