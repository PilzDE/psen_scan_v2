^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package psen_scan_v2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2020-10-19)
------------------
* Initial release for the support of PSENscan firmware >= 3.1.0
* Start and stop the PSENscan monitoring function
* Publish measurement data of each monitoring frame as a single ROS LaserScan message
* Retry activation of the monitoring function on start reply timeout
* Adding urdf for scanner and swapping Z axis of TF frame where scan is published to be sent in correct order
* Contributors: Pilz GmbH and Co. KG
