[![PILZ](img/pilz-logo.png)](https://www.pilz.com)

# PILZ safety laser scanner PSENscan

## Package: psen_scan_v2

The **psen_scan_v2** package is a ROS integration of the PSENscan safety laser scanner product. It lets you integrate the laser scanner data into your ROS Environment easily. Using the standard [sensor_msgs/LaserScan][] message format ensures compatibility with other laserscan-post-processing nodes such as [gmapping][]. For a general overview and link collection we refer to the [wiki page](http://wiki.ros.org/psen_scan_v2).

<p align="center">
<img src="img/PSENscan.jpg" alt="PILZ safety laser scanner" title="PILZ safety laser scanner">
</p>

PSENscan firmware >= 3.1.0 is supported on the following models:

| Type | Features | Order number |
|------|----------|--------------|
| |	Common features:<ul><li>compliant and approved in accordance with: EN/IEC 61496-1: Type 3, EN ISO 13849-1: PL d, IEC 61508: SIL 2</li><li>opening angle: 275°</li><li>operating range: 3.0 or 5.5 m safety zone, 40 m warning zone</li><li>reaction time: 62 ms</li><li>Protection type: IP65</li><li>Dimensions (H x W x D) in mm: 152 x 102 x 112.5</li></ul> | |
| |Light versions	Additional features: Muting, EDM, Override | |
| PSEN sc L 3.0 08-12	3.0 m | safety zone, 8 or 12-pin exchangeable memory module |	6D000012 |
| PSEN sc L 5.5 08-12	5.5 m | safety zone, 8 or 12-pin exchangeable memory module	| 6D000013 |
| | Master versions	Additional features: Muting, EDM, Override, restart in accordance with EN ISO 61496-3, vertical applications| |
| PSEN sc M 3.0 08-12	3.0 m | safety zone, 8 or 12-pin exchangeable memory module	| 6D000016 |
| PSEN sc M 5.5 08-12	5.5 m | safety zone, 8 or 12-pin exchangeable memory module	| 6D000017 |

## Table of Contents

1. [Installation](#installation)
2. [Build Status](#build-status)
3. [Setup](#setup)
4. [ROS API](#ros-api)
5. [Usage](#usage)

## Installation
To use the package, you can install prebuilt packages with
```
sudo apt install ros-$ROS_DISTRO-psen-scan-v2
```

## Migration
To update your ROS environment from the former `psen_scan` package (which supported firmware versions up to 3.0), please execute the following steps:
1. Update scanner firmware using PSENscan Configurator (unless the device has firmware 3.1 already)
2. Install the new ROS package ```sudo apt install ros-$ROS_DISTRO-psen-scan-v2```
3. Replace the launch file arguments:
	* `password` and `x_axis_rotation` are obsolete and should be dropped
	* `angle_start` and `angle_end` are now in radians, in direction of the x axis of the scanner tf frame
4. In your application launch file / roslaunch command: replace all occurrences of `psen_scan` with `psen_scan_v2`

## Build Status
| Platform | Melodic | Noetic |
| -------- | ------- | ------ |
| CI | [![CI-Melodic](https://github.com/PilzDE/psen_scan_v2/workflows/CI-Melodic/badge.svg?event=push)](https://github.com/PilzDE/psen_scan_v2/actions?query=event%3Apush+workflow%3ACI-Melodic+branch%3Amain) | [![CI-Noetic](https://github.com/PilzDE/psen_scan_v2/workflows/CI-Noetic/badge.svg?event=push)](https://github.com/PilzDE/psen_scan_v2/actions?query=event%3Apush+workflow%3ACI-Noetic+branch%3Amain) |
| Buildfarm src | [![Build Status](http://build.ros.org/job/Msrc_uB__psen_scan_v2__ubuntu_bionic__source/badge/icon)](http://build.ros.org/job/Msrc_uB__psen_scan_v2__ubuntu_bionic__source/) | Coming Soon! |
| Buildfarm bin | [![Build Status](http://build.ros.org/job/Mbin_uB64__psen_scan_v2__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros.org/job/Mbin_uB64__psen_scan_v2__ubuntu_bionic_amd64__binary/) | Coming Soon! |


## Branching model
`main` is considered to be the active development branch, it targets the ROS distributions `melodic` and `noetic`.

## Setup

Needed Equipment:
- PSENscan safety laser scanner
- ROS Machine

## ROS API

### Published Topics
/laser_scanner/scan ([sensor_msgs/LaserScan][])<br/>
Publishes a complete scan from the PSENscan safety laser scanner.

### Parameters
_host_ip_ (_string_, default: "192.168.0.50")<br/>
IP-Address of host machine.

_host_udp_port_data_ (_int_, default: 55115)<br/>
UDP Port on which monitoring frames (scans) should be received.

_host_udp_port_control_ (_int_, default: 55116)<br/>
UDP Port used to send commands (start/stop) and receive the corresponding replies.

_sensor_ip_ (_string_, default: "192.168.0.10")<br/>
IP-Address of safety laser scanner.

_prefix_ (_string_, default: "laser_1")<br/>
Name of this scanner that can be changed to differentiate between multiple units.

_angle_start_ (_double_, default: -2.40 (= -137.5 deg))<br/>
Start angle of measurement. (Radian)

_angle_end_ (_double_, default: 2.40 (= 137.5 deg))<br/>
End angle of measurement. (Radian)

## Usage
To start reading from the safety laser scanner and publishing complete scans execute `roslaunch psen_scan_v2 psen_scan_v2.launch` in a command line. This will launch the ROS Node with the default configuration.

If you wish to set parameters from the command line, add them to the end of the command as follows: `parameter:=value`, separated by spaces.

```bash
roslaunch psen_scan_v2 psen_scan_v2.launch sensor_ip:=192.168.0.10 host_ip:=192.168.0.20 host_udp_port_data:=3050
```
This example configures the safety laser scanner at 192.168.0.10 to send it´s frames to 192.168.0.20:3050.

## TF Frames
The location of the TF frames is shown in the image below.
These names are defined by the aforementioned launchfile parameter `prefix`.
Changing them is necessary for instance when running multiple scanners.
<p align="center">
<img src="img/frames.png" width="800px" alt="PILZ safety laser scanner frames" title="frames">
</p>

## Defining the scan range
You can adjust the scan field to your needs by changing _angle_start_ and _angle_end_.
The published ([sensor_msgs/LaserScan][]) will only contain data within the given angle limits.
Both limits are defined within the _laser_1_scan_ frame as shown in the image below.

<p align="center">
<img src="img/angle_limits.png" width="800px" alt="Limit visualization" title="Limit visualization">
</p>

### Adjust expert parameters
If you've created an application package with your own launch file as described in the
[tutorials](http://wiki.ros.org/psen_scan_v2/Tutorials/),
you can easily adjust the aforementioned configuration parameters.

## Test concept
![psen_scan_test_concept](doc/test_architecture.svg)

## You need further information?
Our international hotline staff will support you individually about our ROS packages at
ros@pilz.de

Find more information about the PILZ safety laser scanner on the [product website](https://www.pilz.com/en-INT/eshop/00106002197131/PSENscan-Safety-Laser-Scanner).

## Visit us at [pilz.com](https://www.pilz.com)
PILZ is an international-scale, innovative automation technology company.
PILZ uses its solutions to create safety for man, machine and the environment.
In addition to head office in Ostfildern near Stuttgart,
the family business is represented over 2,400
employees at 42 subsidiaries and branches on all
continents.

The company’s products include sensor technology, electronic monitoring relays, safety
relays, configurable and programmable control systems, automation solutions with motion
control, systems for industrial communication as well as visualization solutions and
operator terminals.

PILZ solutions can be used in all areas of mechanical engineering, including the packaging
and automotive sector, plus the railway technology, press and wind energy sectors.
These solutions ensure that baggage handling systems run safely at airports and
funiculars or roller coasters travel safely; they also guarantee fire protection and energy
supply in buildings.


[sensor_msgs/LaserScan]: http://docs.ros.org/noetic/api/sensor_msgs/html/msg/LaserScan.html
[gmapping]: http://wiki.ros.org/gmapping
