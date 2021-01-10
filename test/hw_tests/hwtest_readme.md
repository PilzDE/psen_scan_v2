<!--
Copyright (c) 2020-2021 Pilz GmbH & Co. KG

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

-->

# Hardware Tests with psen_scan_v2

## Build
To build the hardware tests execute
```
catkin_make
catkin_make tests -DENABLE_HARDWARE_TESTING=ON
```
in your catkin workspace.

## Setup the reference scan
This step is only needed if the setup of the scanner or something within its environment changed.

First startup the scanner
```
roslaunch psen_scan_v2 psen_scan_v2.launch angle_start:=-1.2 angle_end:=1.2
```

After this record the bag file

```
export HW_TEST_SCAN_COMPARE_TESTFILE=<your/desired/path/file.bag>
rosbag record -a -O $HW_TEST_SCAN_COMPARE_TESTFILE --duration 10
```

## Run
To run the hardware tests execute
```
export HW_TEST_SCAN_COMPARE_TESTFILE=<path/to/reference/file.bag>
rostest psen_scan_v2 hwtest_scan_compare.test
```
