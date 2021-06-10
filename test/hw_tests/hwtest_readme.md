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
HOST_IP=192.168.0.122 --env SENSOR_IP=192.168.0.100 -p 55115:55115/udp -p 55116:55116/udp"
```
note that you especially need to setup the `HOST_IP` to be the IP of your actually system
in order to receive the data inside the docker container used by industrial_ci.

### With a custom ROOT_CA and apt proxy
If you need to use a custom ROOT_CA and have a apt-proxy the command for running `industrial_ci` locally extends to
```
rosrun industrial_ci run_ci ROS_DISTRO=noetic ROS_REPO=main \
CMAKE_ARGS="-DENABLE_HARDWARE_TESTING=ON" \
DOCKER_RUN_OPTS="--env HOST_IP=192.168.0.122 --env SENSOR_IP=192.168.0.100 \
-p 55115:55115/udp -p 55116:55116/udp \
-v /usr/local/share/ca-certificates:/usr/local/share/ca-certificates:ro" \
APT_PROXY=http://172.20.20.104:3142

```

## Run `hwtest_scan_compare`
The `hwtest_scan_compare` compares scanner data to a set of prerecorded data in order to detect unwanted changes in the data itself (shifts, flips, ...).

### Build the test
Same as above however **additionally** `-ENABLE_HARDWARE_TESTING_WITH_REFERENCE_SCAN=ON` needs to be defined at compile time. At runtime the reference file needs to be set within the environment variable `HW_TEST_SCAN_COMPARE_TESTFILE`, for details see the instructions below.
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

### Run using `rostest`
To run the hardware tests execute
```
export HW_TEST_SCAN_COMPARE_TESTFILE=<path/to/reference/file.bag>
rostest psen_scan_v2 hwtest_scan_compare.test
```
