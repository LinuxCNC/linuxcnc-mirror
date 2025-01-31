#!/bin/bash

cd "$(dirname "$0")" || { echo "E: Could not change directory to '$(dirname "$0")'"; exit 1; }

gladevcp -u ./meter_scale.py ./meter_scale.ui &

# wait for gladevcp startup
sleep 3
# start sim_pin to exercise pins
sim_pin \
   meter_scale.hal_table1/mode=toggle \
   meter_scale.max-value \
   meter_scale.meter \

exit 0
