#!/bin/bash

Error[0]="hm2/hm2_test\.0: invalid cookie"
Error[1]="hm2/hm2_test\.0: invalid config name"
Error[2]="hm2/hm2_test\.0: invalid IDROM type"
Error[3]="hm2/hm2_test\.0: invalid IDROM type"
Error[4]="hm2/hm2_test\.0: invalid IDROM PortWidth"
Error[5]="hm2/hm2_test\.0: invalid IDROM PortWidth"
Error[6]="hm2/hm2_test\.0: IDROM IOPorts is 0 but llio num_ioport_connectors is \d+"
Error[7]="hm2/hm2_test\.0: IDROM IOWidth is \d+, but IDROM IOPorts is \d+ and IDROM PortWidth is \d+"
Error[8]="hm2/hm2_test\.0: IDROM IOPorts is \d+ but llio num_ioport_connectors is \d+"
Error[9]="hm2/hm2_test\.0: IDROM ClockLow is \d+, that's too low"
Error[10]="hm2/hm2_test\.0: IDROM ClockHigh is \d+, that's too low"
Error[11]="hm2/hm2_test\.0: pin 0 primary tag is 0 \(end-of-list sentinel\), expected 144 pins!"
Error[12]="hm2/hm2_test\.0: [^:]+: invalid pin number 0"
Error[13]="hm2/hm2_test\.0: invalid IDROM PortWidth 24, this board has 5 pins per connector, aborting load"
Error[14]="hm2/hm2_test\.0: IDROM IOPorts is 0 but llio num_ioport_connectors is \d+"

result=0

TEST_PATTERN=0
while [ -n "${Error[$TEST_PATTERN]}" ]; do
    export TEST_PATTERN
    halrun -f broken-load-test.hal >halrun-stdout 2>halrun-stderr
    ./check-dmesg.py "${Error[$TEST_PATTERN]}" || exit $?
    TEST_PATTERN=$((TEST_PATTERN + 1))
done

exit $result
