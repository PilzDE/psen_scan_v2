[![PILZ](../img/pilz-logo.png)](https://www.pilz.com)

# PILZ safety laser scanner PSENscan Standalone C++ Library

The PSENscan Standalone C++ Library allows you to easily integrate the PSENscan safety laser scanner into your application without depending on ROS. You will be able to read both distance and intensity values from the PSENscan safety laser scanner, which can subsequently be used for navigation algorithms and much more.

## Table of Contents

1. [Get Started](#get-started)
2. [Usage example](#usage-example)
3. [C++ API](#c++-api)

## Get Started
First, clone this repository:
```
git clone https://github.com/PilzDE/psen_scan_v2.git
```

Next, navigate to the standalone folder and create a `build` directory.
```
cd psen_scan_v2/standalone/
mkdir build
```

Lastly, execute the following:
```
cd build/ && cmake .. && make
```

The library should now be built and available under `psen_scan_v2/standalone/build/libpsen_scan_v2_standalone.a`

## Usage example
To build the example application, which prints distance data to the screen, you can follow the first two steps as in [Get Started](#get-started) if you haven't done so already. The last command will be different:
```
cd build/ && cmake -DBUILD_EXAMPLE=ON .. && make
```

You can now execute the example program by running the following while in the `build` folder:
```
./psen_scan_v2_standalone_app
```

Feel free to take a look at the [example code](https://github.com/PilzDE/psen_scan_v2/blob/main/standalone/main.cpp) as well as the [C++ Api](#c++-api) to understand how to integrate the C++ Library into your application.

## C++ Api
For more Documentation please take a look at the generated Doxygen [Code API][]. The following classes are good to get started:
 - [LaserScan][]
 - [ScannerV2][]



[Code API]: http://docs.ros.org/en/melodic/api/psen_scan_v2/html/
[LaserScan]: http://docs.ros.org/en/melodic/api/psen_scan_v2/html/classpsen__scan__v2_1_1LaserScan.html
[ScannerV2]: http://docs.ros.org/en/melodic/api/psen_scan_v2/html/classpsen__scan__v2_1_1ScannerV2.html