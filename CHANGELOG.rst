^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package psen_scan_v2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.10.2 (2022-11-03)
-------------------
* Remove zoneset bits from IOState.input field since its redundant to the active_zoneset field (#311)
* Inform about missing configuration confirmation
* Ignore changes of unused IOs. Fix #321
* Fill "io_states" vector as received instead of order by theta to avoid flickering. Fix #320
* Fixed bug of multiple zonesets being visible #328
* Contributors: Pilz GmbH and Co. KG

0.10.1 (2022-01-24)
-------------------
* Separate 'PinState' formating from 'formatRange' (#313)
* Contributors: Pilz GmbH and Co. KG

0.10.0 (2022-01-20)
-------------------
* Change to new versioning method for ROS1 (0.10.0+) and ROS2 (0.20.0+).
* Deprecate get/set methods with 'get/set' prefix in favour of methods without the prefix(#298)
* ADD IO states to LaserScan and publish them at ~/io_state (#281)
* Renaming Slave to Subscriber (#303)
* Contributors: Pilz GmbH and Co. KG

0.3.4 (2021-12-20)
------------------
* Prevent error when diagnostics are disabled. Fix #294
* Set exception in stop-future
* Apply clang-tidy fixes to header-files. Fix #277
* Remove latched flag from active zoneset topic
* API change: convenience usage of ScannerConfiguration
* Remove unrelated parameter DEFAULT_X_AXIS_ROTATION from standalone
* Contributors: Pilz GmbH and Co. KG

0.3.3 (2021-11-10)
------------------
* Introduce error state and set exception in start-future
* Remove dependency on pilz_testutils
* Always build hardware tests
* Internal refactorings
* Importing the config xml-file and publishing the zoneconfiguration
* Add active zoneset to LaserScan
* Publish active zoneset id to ~/active_zoneset
* Add active zoneset visualization in rviz
* Contributors: Pilz GmbH and Co. KG

0.3.2 (2021-09-16)
------------------
* Improve performance of standalone part
    * Introduce RawDataPtr
    * Write to std::stringstream instead of constructing from std::string
    * Pass by reference wherever possible
* Make release build the default
* Calculate timestamp as the time of the first ray (udp communication time is neglected)
* API: Add timestamp (nanoseconds since epoch) to LaserScan
* API: Add scan counter to LaserScan
* API: remove LaserScan equality operator
* Contributors: Pilz GmbH and Co. KG

0.3.1 (2021-07-21)
------------------
* Rename urdf links
* Apply tf prefix equals node name
* API change: rename launch-file argument prefix->tf_prefix
* Contributors: Pilz GmbH and Co. KG

0.3.0 (2021-06-29)
------------------
* Set prefix for node name (same value as for tf frames)
* Introduce bringup.launch for starting only the scanner node
* Omit closing the data client before destruction. Fixes #212
* Distance value at angle_end is now included in the scan range
* Default scan range changed to [-137.4..137.4]deg at 0.1 deg resolution
* Add options for modifying resolution and enabling intensities
* Add launchfile option for disabling rviz at startup (default: enable)
* Handle the infinity codes from the scanner correctly
* Inform user about current scan range
* Fix wrong behavior when angle_start and angle_end are equal
* Contributors: Pilz GmbH and Co. KG


0.2.1 (2021-04-19)
------------------
* Fix issues with smaller angle ranges than default range (`#183 <https://github.com/PilzDE/psen_scan_v2/issues/183>`_)
* Add action for pull request todos (`#184 <https://github.com/PilzDE/psen_scan_v2/issues/184>`_)
* Contributors: Pilz GmbH and Co. KG

0.2.0 (2021-04-05)
------------------
* Combine multiple monitoring frames into one scan (`#173 <https://github.com/PilzDE/psen_scan_v2/issues/173>`_)
* Contributors: Pilz GmbH and Co. KG

* API change: Driver waits for scan round to complete before sending measurement data by default
* Introducing new parameter fragmented_scans to switch back to faster fragmented laser_scan messages
* Contributors: Pilz GmbH and Co. KG

0.1.6 (2021-03-22)
------------------
* Separate ROS and standalone CMakeLists: Closes #169
* Minor restructuring. Closes #175
* Contributors: Pilz GmbH and Co. KG

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
