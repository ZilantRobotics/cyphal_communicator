#!/bin/bash
# This software is distributed under the terms of the GPL v3 License.
# Copyright (c) 2022-2023 Dmitry Ponomarev.
# Author: Dmitry Ponomarev <ponomarevda96@gmail.com>

CRNT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_DIR="$(dirname "$CRNT_DIR")"
OUTPUT_DIR=${REPO_DIR}/src/nunavut_out

$REPO_DIR/Libs/cyphal_application/scripts/nnvg_generate_c_headers.sh ${OUTPUT_DIR}
