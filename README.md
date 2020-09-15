# PolygonalPerception

This repository contains the software implementation of our polygonal perception pipeline for mobile robots as presented at the IROS 2020 conference. The polygonal perception pipeline computes from an RGB-D image a floor-projected set of 2D polygons that represent the obstacles in the field of view of the sensor. The polygons can then be used for path and motion planning.

# Installation

The target operating system is Ubuntu Linux 16.04 or higher. Required packages are Qt5, OpenCV2, QGLViewer, and Armadillo. The used build system is qmake. After cloning the project, type "qmake" and then "make" in the root directory of the project. Then, you can run the application by executing ./PolygonalPerception. The application loads an example point cloud and opens a graphical interface where the point cloud is shown, the computed polygons, and the computed transformation of the camera relative to the world. The executable looks for the conf directory, the data directory, and the styles.css style sheet and won't load correctly if these items are not found for example due to using a build directory different from the project root.

# Code Documentation

We recommend you to use QtCreator to browse the source code. Load the PolygonalPerception.pro file to load the project. If asked, point QtCreator to the root of the project for the release and debug builds. The perception pipeline is located in the sense() method of RobotControl.cpp. The pipeline uses SampleGrid.cpp for the floor detection and GridModel.cpp, which is an occupancy grid implementation that contains a functions for the extractions of the polygons from the grid.


