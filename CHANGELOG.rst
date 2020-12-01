^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package psen_scan_v2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.1.2 (2020-11-24)
------------------
* Add Scanner ports in ScannerConfiguration
* Switch branching model: Introduce main branch
* Use github actions
* Adds missing error bit
* Contributors: Pilz GmbH and Co. KG

0.1.1 (2020-11-10)
------------------
* Add intensity data to LaserScan
* Print user message if dropped MonitoringFrames are detected
* Print user message in case of timeout while waiting for MonitoringFrames
* Adjust min and max scanner range. Fixes #70
* Print diagnostic messages from PSENscan
* Notify user about start retry
* Add Continuous Acceptance Test
* Contributors: Pilz GmbH and Co. KG

0.1.0 (2020-10-19)
------------------
* Initial release for the support of PSENscan firmware >= 3.1.0
* Start and stop the PSENscan monitoring function
* Publish measurement data of each monitoring frame as a single ROS LaserScan message
* Retry activation of the monitoring function on start reply timeout
* Adding urdf for scanner and swapping Z axis of TF frame where scan is published to be sent in correct order
* Contributors: Pilz GmbH and Co. KG
