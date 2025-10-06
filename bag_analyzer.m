%%% bag analyzer %%%
clear all 
close all
clc

%% Load Bagfile
bagdir_name = "bags";
bagname = "my_robot_New_Experiment1";
bag_ext = ".bag";

% Add bag dir
addpath("bag_analyzer");
addpath(bagdir_name);

% fullfile makes compatible with windows or ubuntu paths
% Ubuntu "/" | Windows "\"
% bag = rosbag(fullfile(bagdir_name, bagname + bag_ext));
bag_analyzer_obj = Bag_Analyzer(fullfile(bagdir_name, bagname + bag_ext), "quaternion_order", "xyzw");

%% Synchronization
% Sampling Frequency
fs = 1.0e+2;

% Call synchronization method
[merged_time, merged_dataset, sync_marker_dict, topics] = bag_analyzer_obj.synchronization(1/fs, "interpolation_method", 'previous');

% Renamed Markers
sync_marker_dict = rename_markers(sync_marker_dict);