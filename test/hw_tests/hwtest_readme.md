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

## Build and run using `catkin_make`
To build the hardware tests execute
```
catkin_make
catkin_make tests -DENABLE_HARDWARE_TESTING=ON
```
in your catkin workspace.

## Build and run using `catkin`
To build and run the hardware tests using `catkin` run something like
```
SENSOR_IP=192.168.0.100 catkin run_tests --cmake-args -DENABLE_HARDWARE_TESTING=ON && catkin_test_results
```
## Build and run using `industrial_ci`
Setup `industrial_ci` to run locally using [this instructions](https://github.com/ros-industrial/industrial_ci/blob/master/doc/index.rst#simplest-way-to-run-locally).
Then run
```
rosrun industrial_ci run_ci ROS_DISTRO=noetic ROS_REPO=main \
CMAKE_ARGS="-DENABLE_HARDWARE_TESTING=ON" DOCKER_RUN_OPTS="--env \
HOST_IP=192.168.0.122 --env SENSOR_IP=192.168.0.100 -p 55000-55020:55000-55020/udp"
```
note that you especially need to setup the `HOST_IP` to be the IP of your actually system
in order to receive the data inside the docker container used by industrial_ci.

### With a custom ROOT_CA and apt proxy
If you need to use a custom ROOT_CA and have a apt-proxy the command for running `industrial_ci` locally extends to
```
rosrun industrial_ci run_ci ROS_DISTRO=noetic ROS_REPO=main \
CMAKE_ARGS="-DENABLE_HARDWARE_TESTING=ON" \
DOCKER_RUN_OPTS="--env HOST_IP=192.168.0.122 --env SENSOR_IP=192.168.0.100 \
-p 55000-55020:55000-55020/udp \
-v /usr/local/share/ca-certificates:/usr/local/share/ca-certificates:ro" \
APT_PROXY=http://172.20.20.104:3142
```

## Hardware Test `hwtest_zoneset_switching`
This test checks if the published active zoneset switches from zero to one if the corresponding IO on the scanner change.
To do this it uses a saintsmart relay to trigger this IO change from ROS context and thus needs additional HW and dependencies.

### Build the test
Same as above however **additionally** `-DENABLE_HARDWARE_TESTING_WITH_RELAY=ON` needs to be defined at compile time.

### Run the test
At runtime the test needs the ros-<distro>-sainsmart-relay-usb package.
It is expected that the scanner power is on relay 3 and the zone IO on relay 4 and that the scanner is turned on beforehand.

## Hardware Test `hwtest_scan_compare`
The `hwtest_scan_compare` compares scanner data to a set of prerecorded data in order to detect unwanted changes in the data itself (shifts, flips, ...).

### Build the test
Same as above however **additionally** `-DENABLE_HARDWARE_TESTING_WITH_REFERENCE_SCAN=ON` needs to be defined at compile time. At runtime the reference file needs to be set within the environment variable `HW_TEST_SCAN_COMPARE_TESTFILE`, for details see the instructions below.
### Setup the reference scan
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

### Run standalone test
Execute
```
export HW_TEST_SCAN_COMPARE_TESTFILE=<path/to/reference/file.bag>
./devel/lib/psen_scan_v2/hwtest_scan_compare_standalone
```

### Run `rostest`
Execute
```
export HW_TEST_SCAN_COMPARE_TESTFILE=<path/to/reference/file.bag>
rostest psen_scan_v2 hwtest_scan_compare.test
```

### Build and run using `industrial_ci`
In addition to the arguments displayed above, you need to make the reference scan available to the docker container. Firstly, create a folder containing the bag-file with the reference scan. Then add the following to the `DOCKER_RUN_OPTS`:
```
-v <your/desired/path/to/folder>:/testfiles --env HW_TEST_SCAN_COMPARE_TESTFILE=/testfiles/<file.bag>
```
and the following to the `CMAKE_ARGS` option:
```
-DENABLE_HARDWARE_TESTING_WITH_REFERENCE_SCAN=ON
```

## Hardware Test `hwtest_timestamp_standalone`
The `hwtest_timestamp_standalone` compares the timestamp to data from udp packets which are captured via wireshark.

### Build the test
Same as above however **additionally** `-ENABLE_HARDWARE_TESTING_WITH_WIRESHARK=ON` needs to be defined at compile time. At runtime a text file needs to be set within the environment variable `UDP_DATA_FILENAME`, for details see the instructions below.

### Setup wireshark
Install the apt-package `tshark` which is required to run the following `sh`-script.

### Run tests
Firstly start recording udp packets in parallel to the tests execution:
```
./test/scripts/record_udp_packets.sh <network_interface> <timeout_in_seconds> 55000 55020 > $UDP_DATA_FILENAME
```
Then execute
```
export UDP_DATA_FILENAME=<path/to/udp/data/file.txt>
./devel/lib/psen_scan_v2/hwtest_timestamp_standalone
```

### Build and run using `industrial_ci`
In addition to the arguments displayed above, you need to make the udp data file available to the docker container. Firstly, create a folder to contain the udp data file. Remember to start recording the udp packets before running `industrial_ci`. Then add the following to the `DOCKER_RUN_OPTS`:
```
-v <your/desired/path/to/folder>:/testfiles --env UDP_DATA_FILENAME=/testfiles/<file.txt>
```
and the following to the `CMAKE_ARGS` option:
```
-DENABLE_HARDWARE_TESTING_WITH_WIRESHARK=ON
```
