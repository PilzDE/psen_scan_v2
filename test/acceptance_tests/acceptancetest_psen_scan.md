<!--
Copyright (c) 2020 Pilz GmbH & Co. KG

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

# Acceptance Test PSENscan using Rviz

## Prerequisites
  - Properly configure the PSENscan safety laser scanner.
  - Connect it to power and ethernet.

### Test Sequence

  0. Wait for the PSENscan to be fully powered up.

  1. Run
  ```
  roslaunch psen_scan_v2 psen_scan.launch sensor_ip:=<sensor_ip> host_ip:=<host_ip> host_udp_port:=<host_udp_port>
  ```
  Replace the variables in brackets with appropriate values for your setup.

  2. Place your hand infront of the PSENscan.

  3. Move your hand left and right around the PSENscan.

### Expected Results

  0. The PSENscan screen shows a coloured ring with text in the middle.

  1. Rviz shows a red laserscan centered around the origin that matches your surroundings as well as some axes.

  2. Rviz shows distance values near to the center toward the X-Axis.

  3. Rviz shows distance values near to the center moving left and right around the origin in synch with your hand movements.
