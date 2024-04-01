
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

### Refactoring plan
If I were to refactor this project, I'd ditch the `*Collection` classes and instead define a `CameraToVehicleGraph` as the central data structure:
 - Use int (or, I guess unsigned int) identifiers to key on objects instead of strings.
 - Define a `Pose3Node` struct with `id`, `pose`, `name`.
 - `unordered_map<int, Pose3Node>`: one for tags, one for cams. Key on node.id.
 - Store intrinsics (or possibly a list of them) as members for camera construction.
 - `CameraToVehicleGraph` would have `getCameraNode(i)`, `getTagNode(j)`
 - Define `Side` and `TagZone` enums.
 - Define a `CameraType` enum for cameras, onboard vs external.
 - Need a way to do something like `getCameraId(CameraType, Side, index=-1)`.
    * Try lookup by defining a composite object as the key. This could be a struct or a tuple called `CameraAttributes`. Then do `map<CameraAttributes, int> cameraLookup`. Could also specialize a hash for `CameraAttributes` and use an unordered map. The latter would be a small internal optimization that could be implemented later without affecting the interfaces.
 - Same story for `getTagId(Side, TagZone)`.
 - Define convenient interfaces to get camera/tag poses, names, etc. using the get id functions internally.
 - `pointMap` would key on a `pair<int, int>`
 - To iterate through graph:
    * Implement `graph.edges()`, which could be overloaded to query one side or all edges.
    * Make enums above iterable by defining a final `NONE` entry and checking for it.
 - Don't use `gtsam::Symbol` class anywhere on the frontend. Just use the int keys.
