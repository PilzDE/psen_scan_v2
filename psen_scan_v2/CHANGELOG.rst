^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package psen_scan_v2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2021-03-10)
------------------
* Make specifying host ip optional
* Reorder Readme sections
* API change: A scan is published only if it contains measurement data
* Make tests build in standalone with or without ROS installed
* Document key components in a meaningful expressive way
* Enable building the standalone lib using MSVC
* Enable separate building of cpp-lib; extract sources into subproject psen_scan_v2_standalone
* Renames ScanRange and DefaultScanRange to improve usability
* Improve namespace hierarchy and move files to respective subfolders
* Use defaults ports in ScannerConfiguration
* Contributors: Pilz GmbH and Co. KG

0.1.4 (2021-01-13)
------------------
* Add internal error handling for detecting regressions in the scanner firmware
* Make specifying udp ports optional
* Contributors: Pilz GmbH and Co. KG


0.1.3 (2020-12-09)
------------------
* Add ROS noetic support (`#103 <https://github.com/PilzDE/psen_scan_v2/issues/103>`_)
* Use TYPED_TEST_SUITE instead of deprecated TYPED_TEST_CASE
* Apply fixes from clang-format (`#113 <https://github.com/PilzDE/psen_scan_v2/issues/113>`_)
* Spelling measurements (`#112 <https://github.com/PilzDE/psen_scan_v2/issues/112>`_)
* Directly use fmt lib instead of rosfmt (`#108 <https://github.com/PilzDE/psen_scan_v2/issues/108>`_)
* Fix clang tidy errors (`#109 <https://github.com/PilzDE/psen_scan_v2/issues/109>`_)
* Feature/api documentation improvement (`#100 <https://github.com/PilzDE/psen_scan_v2/issues/100>`_)
* Improve the API documentation
* Add ROS Noetic support
* Contributors: Pilz GmbH and Co. KG

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
