# Rviz Annotator
## Description

Rviz Annotator is a ROS package that integrates [rosbag](https://github.com/ros/ros_comm/tree/kinetic-devel/tools/rosbag) player into rviz plugins for playing and annotating recorded PointCloud2 messages. The user can select points displayed in rviz for manual object clusters naming and export a new .bag file. It contains three rviz plugins:
* Rosbag player
* Annotation panel
* PointCloud2 selection tool (using PointCloud2_Segments messages)

### Compiling

Build with `catkin_make install` under catkin workspace.

## Setup

1. Open `plugin_params.yaml` in the config directory to set default parameters:
	* `sel_topic`: topic for publishing selected points message
	* `marker_array_topic`: topic for publishing marker array
	* `csv_name`: name of the output .csv file after rviz save (`Ctrl + S` *only* )
	* `csv_dir`: location of the output .csv file
	* `bagfiles_dir`: directory containing .bag files
	* `pc_segments_topic`: subscriber PointCloud2_Segments topic
	* `pc2_topic`: subscriber PointCloud2 topic

1. Open `rviz` and add `RosbagPlayer` panel, `Annotation` panel and `PointSelect` tool
1. Under player panel, select a directory that contains .bag files and, under annotation panel, type PointCloud2 topic to subscribe to, or refresh to update open topics and select from the list
1. Add MarkerArray display to see annotated points

## Annotation

Activate PointSelect tool to select points on a specific frame.

Annotation panel shows selection status, depending on the initial clustering. Hit `Join` to create custom cluster or `Divide` to separate points from previous custom cluster. Type cluster name to set it and markers visualize the created clusters.

Hit `Ctrl + S` to save annotated points to csv when done.

## Export

Run `rosrun rviz_annotator expoertBag <initial_bagfile_name>` to export a new .bag file containing annotations.

Each custom cluster in the new .bag has its own color. Clusters that have not been selected manually are displayed grayed out.

### Notes

Rosbag player has backwards step as an extra feature, but does not support all the original player features.

`SyncTopics` option of the player must be checked always before performing Backstep.

The format of directory parameters must be `path/to/file` (no `/` on the beginning and end).

Currently only C++11 compiler is supported.