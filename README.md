# rosbag2matlab
This repository contains the code to extract and visualize topics and messages from a ROS bag.

## ROS 1 / ROS 2 support

`Bag_Analyzer` accepts either a ROS 1 bag or a ROS 2 bag, and figures out
which one it's looking at from the path you give it -- no flag to set:

- **ROS 1**: a single bag file, e.g. `bags/my_experiment.bag`.
- **ROS 2**: a bag **directory**, e.g. `bags/my_experiment/`, containing a
  `metadata.yaml` file and one or more `*.db3` (sqlite3) or `*.mcap`
  storage files (the standard layout produced by `ros2 bag record`).

The detection is done by `bag_analyzer/detect_bag_format.m`, which is pure
file-system inspection (no ROS Toolbox calls): a `.bag` file is classified
as `"ros1"`, a directory with `metadata.yaml` and/or `*.db3`/`*.mcap` files
is classified as `"ros2"`, and anything else raises a clear error.

`Bag_Analyzer`'s constructor uses this to open the bag with the matching
reader (`rosbag` for ROS 1, `ros2bagreader` for ROS 2). Everything after
that -- selecting topics, reading messages, decoding them in `extractData`,
and `synchronization` -- is shared, unchanged code: the ROS Toolbox exposes
the same `select`/`readMessages`/`MessageList.Time` surface and the same
`pkg/Type` message-type strings for both bag formats, so no ROS1/ROS2
branching is needed there.

To point the demo script (`bag_analyzer.m`) at a ROS 2 bag instead of a
ROS 1 one, just set `bagname` to the ROS 2 bag directory's name (no
extension) instead of a `.bag` file's name -- the script picks whichever
one exists on disk.

See `tests/test_bag_format_detection.m` for an offline (no bag / no ROS
Toolbox required) test of the format detector and of the PoseStamped
message decoder against a synthetic ROS 2-shaped message struct.
