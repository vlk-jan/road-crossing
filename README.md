# Road Crossing

## Overview

This package provides a ROS node to facilitate safe traverse of an autonomous robot across a road. It was done as a bachelor's thesis at FEE CTU.

The main algorithm is based on a behavior-tree structure. This work does not include the detection of incoming vehicles that are provided by a different node.

## Installation

Clone this repository into your catkin workspace.

```bash
git clone https://github.com/vlk-jan/road_crossing.git
```

For this package to work you need to install additional packages:

- [Compass](https://github.com/ctu-vras/compass) - provides the robots azimuth. The message files are also necessary.

The package also requires additional libraries:

- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) - c++ library containing the behavior-tree implementation we use.
- [GeographicLib](https://geographiclib.sourceforge.io/C++/doc/index.html) - c++ library for geographical conversions.
- [Numpy](https://numpy.org/doc/stable/) - python library for numerical calculations.
- [Overpy](https://github.com/DinoTools/python-overpy) - python library for accesing the OSM overpass API.
- [Shapely](https://shapely.readthedocs.io/en/stable/manual.html) - python library for work with geometrical objects.
- [UTM](https://github.com/Turbo87/utm) - python library for bidirectional UTM <-> WGS84 conversion.

Installing c++ libraries:

```bash
sudo apt-get install libgeographic-dev ros-$ROS_DISTRO-behaviortree-cpp-v3
```

Installing python libraries

```bash
pip install -r requirements.txt
```

## Documentation

This package does not currently have any documentation. However, the text of my bachelor thesis is [here](https://github.com/vlk-jan/bachelor_thesis), where I try to explain the code a bit.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://github.com/vlk-jan/road_crossing/blob/master/LICENSE)

## TODO

- Short-term
  - Add acceleration and width to collision calculation
  - move_to_place weight map (point-cloud) creation & implementation
  - Implement crossing_finished
  - Add road width to road cost?
- Long-term
  - Robot steering during the crossing, currently moving only forward and backward in a straight line
  - Service for obtaining better place - possible need to rewrite road cost
  - Vehicle obtaining (from ROS node) message type & implementation
