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

# Acceptance Test PSENscan using Rviz

## Prerequisites
  - Properly configure the PSENscan safety laser scanner.
  - Connect it to power and ethernet.
  - Perform the preparations described in [hwtest_readme.md](https://github.com/PilzDE/psen_scan_v2/blob/main/test/hw_tests/hwtest_readme.md)

### Test Sequence

  0. Wait for the PSENscan to be fully powered up.

  1. Run `rostest psen_scan_v2 hwtest_scan_compare.test`.

  2. Run `rostest psen_scan_v2 hwtest_scan_compare.test` and move your hand infront of the PSENscan.

### Expected Results

  0. The PSENscan screen shows a coloured ring with text in the middle.

  1. The test result is success.

  2. The test result is failure.
