#!/bin/bash

scenarios=$(ls homework_05/data/)
# add non-existed file scenario
scenarios="$scenarios non_existed_file.txt"

GREEN='\033[0;32m'
NC='\033[0m'

for scenario in $scenarios; do
    echo -e "${GREEN}Running scenario: $scenario${NC}"
    ./build/debug/homework_05/telemetry_check ./homework_05/data/$scenario
done
