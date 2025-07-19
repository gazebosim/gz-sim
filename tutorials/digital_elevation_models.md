# Overview

A Digital Elevation Model (DEM) is a 3D representation of a terrain's surface that does not include any objects like buildings or vegetation. DEMs are frequently created by using a combination of sensors, such as LIDAR, radar, or cameras. The terrain elevations for ground positions are sampled at regularly-spaced horizontal intervals.

The term DEM is just a generic denomination, not a specific format. In fact, the DEMs can be represented as a grid of elevations (raster) or as a vector-based triangular irregular network (TIN). Currently, Gazebo only supports raster data in the supported formats available in [GDAL](https://gdal.org/en/stable/).
