
## ECCE: Extrinsic camera calibration example

### Quickstart
To run this code, install the following dependencies:
 - [OpenCV](https://opencv.org/get-started/)
 - [GTSAM](https://gtsam.org/get_started/)

Then follow a typical CMake workflow:
```
mkdir build && cd build
cmake ..
make
```
This builds any .cpp files in examples/ with static linkage to the library code in ecce/.
They can then be run with no arguments. `camera_to_vehicle_calibration` saves several text files that can be used for analysis and visualization.

### Plotting and graphics
Various python scripts are included in the python/ directory. Package dependencies can be installed (preferably into a dedicated environment) via `pip install -r requirements.txt`.

Note: file names written from `camera_to_vehicle_calibration` are hard-coded in scripts (TODO)
