%%% bag analyzer %%%
clear all 
close all
clc

%% Load Bagfile
bagdir_name = "bags";
bagname = "my_robot_New_Experiment1";
bag_ext = ".bag"; % ROS 1 bag file extension; ROS 2 bags are a directory instead (no extension)

% Add bag dir
addpath("bag_analyzer");
addpath(bagdir_name);

% fullfile makes compatible with windows or ubuntu paths
% Ubuntu "/" | Windows "\"
%
% Bag_Analyzer accepts either:
%   - a ROS 1 bag FILE:      bags/<bagname>.bag
%   - a ROS 2 bag DIRECTORY: bags/<bagname>/   (contains metadata.yaml + .db3/.mcap)
% The format is auto-detected from the path (bag_analyzer/detect_bag_format.m),
% so nothing else below needs to change based on which one you point at.
ros1_bag_path = fullfile(bagdir_name, bagname + bag_ext);
ros2_bag_path = fullfile(bagdir_name, bagname);

if isfolder(ros2_bag_path) && ~isfile(ros1_bag_path)
    % Only prefer the ROS 2 directory when there's no same-named ROS 1
    % file, so the existing ROS 1 setup keeps working unchanged.
    bag_path = ros2_bag_path;
else
    bag_path = ros1_bag_path;
end

% bag = rosbag(bag_path); % (ROS 1) — now handled inside Bag_Analyzer via detect_bag_format
bag_analyzer_obj = Bag_Analyzer(bag_path, "quaternion_order", "xyzw");

%% Synchronization
% Sampling Frequency
fs = 1.0e+2;

% Call synchronization method
[merged_time, merged_dataset, sync_marker_dict, topics] = bag_analyzer_obj.synchronization(1/fs, "interpolation_method", 'previous');

% Renamed Markers
sync_marker_dict = rename_markers(sync_marker_dict);