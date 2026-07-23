%TEST_BAG_FORMAT_DETECTION Static/offline tests for ROS 1 / ROS 2 bag support.
%
%   This is a STATIC, OFFLINE test script: it only fabricates temporary
%   file/folder fixtures on disk and synthetic in-memory message structs.
%   It does NOT open a real .bag or ROS 2 bag directory, does NOT call
%   rosbag()/ros2bagreader(), and needs NO ROS network / ROS Toolbox
%   installation. It is therefore safe to run without any real bag file.
%
%   It must be run with the current folder (or path) including
%   rosbag2matlab/bag_analyzer, i.e. from inside rosbag2matlab/:
%
%       cd rosbag2matlab
%       run tests/test_bag_format_detection.m
%
%   Per task instructions this script has been written but intentionally
%   NOT executed as part of this change.
%
%   Coverage:
%     (a) detect_bag_format.m classifies fabricated fixtures correctly:
%           - an empty ROS 1 "*.bag" file            -> "ros1"
%           - a directory containing "metadata.yaml"  -> "ros2"
%           - a directory containing a "*.db3" file   -> "ros2"
%           - a directory with neither marker         -> errors
%     (b) Bag_Analyzer.decodePoseStamped (the static, dependency-free core
%         of the geometry_msgs/PoseStamped branch of Bag_Analyzer.extractData)
%         fed a synthetic ROS2-shaped struct message ('DataFormat','struct'
%         layout, which is identical between ROS1 and ROS2 for this message
%         type) decodes to the expected 7x1 [pos; quat] column, in both
%         supported quaternion orders.

% Make sure detect_bag_format.m / Bag_Analyzer.m are on the path when this
% script is run standalone from tests/.
this_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(this_dir, '..', 'bag_analyzer'));

n_pass = 0;
n_fail = 0;

%% ---- (a) detect_bag_format: fabricated fixtures --------------------------

tmp_root = tempname;
mkdir(tmp_root);
cleanupObj = onCleanup(@() rmdir(tmp_root, 's')); %#ok<NASGU> % auto-clean temp fixtures

% Fixture 1: an empty ROS 1 .bag FILE
ros1_file = fullfile(tmp_root, 'foo.bag');
fid = fopen(ros1_file, 'w');
fclose(fid);
[n_pass, n_fail] = check(n_pass, n_fail, 'ROS1 .bag file -> "ros1"', ...
    detect_bag_format(ros1_file), "ros1");

% Fixture 2: a DIRECTORY containing metadata.yaml (ROS 2)
ros2_dir_meta = fullfile(tmp_root, 'ros2_bag_meta');
mkdir(ros2_dir_meta);
fid = fopen(fullfile(ros2_dir_meta, 'metadata.yaml'), 'w');
fclose(fid);
[n_pass, n_fail] = check(n_pass, n_fail, 'ROS2 dir with metadata.yaml -> "ros2"', ...
    detect_bag_format(ros2_dir_meta), "ros2");

% Fixture 3: a DIRECTORY containing a .db3 file, no metadata.yaml (ROS 2)
ros2_dir_db3 = fullfile(tmp_root, 'ros2_bag_db3');
mkdir(ros2_dir_db3);
fid = fopen(fullfile(ros2_dir_db3, 'rosbag2_2024_01_01-00_00_00_0.db3'), 'w');
fclose(fid);
[n_pass, n_fail] = check(n_pass, n_fail, 'ROS2 dir with .db3 -> "ros2"', ...
    detect_bag_format(ros2_dir_db3), "ros2");

% Fixture 4: a DIRECTORY containing a .mcap file (ROS 2, alternate storage)
ros2_dir_mcap = fullfile(tmp_root, 'ros2_bag_mcap');
mkdir(ros2_dir_mcap);
fid = fopen(fullfile(ros2_dir_mcap, 'rosbag2_2024_01_01-00_00_00_0.mcap'), 'w');
fclose(fid);
[n_pass, n_fail] = check(n_pass, n_fail, 'ROS2 dir with .mcap -> "ros2"', ...
    detect_bag_format(ros2_dir_mcap), "ros2");

% Fixture 5: a directory that looks like neither format -> must error
neither_dir = fullfile(tmp_root, 'not_a_bag');
mkdir(neither_dir);
try
    detect_bag_format(neither_dir);
    fprintf('[FAIL] directory matching neither format should have errored\n');
    n_fail = n_fail + 1;
catch
    fprintf('[PASS] directory matching neither format correctly errors\n');
    n_pass = n_pass + 1;
end

% Fixture 6: a path that does not exist at all -> must error
try
    detect_bag_format(fullfile(tmp_root, 'does_not_exist.bag'));
    fprintf('[FAIL] nonexistent path should have errored\n');
    n_fail = n_fail + 1;
catch
    fprintf('[PASS] nonexistent path correctly errors\n');
    n_pass = n_pass + 1;
end

%% ---- (b) decodePoseStamped: ROS1-shaped (PascalCase) struct message -----

% Synthetic geometry_msgs/PoseStamped message in ROS1's PascalCase struct
% shape (Pose.Position.X, ...). NOTE: this layout is NOT actually
% identical between ROS1 and ROS2 -- a real ROS2 bag's struct output uses
% lowercase field names instead (pose.position.x), confirmed against a
% real ROS2 bag (see Bag_Analyzer.gf() and its callers). This fixture
% exercises the ROS1 branch of Bag_Analyzer.gf()'s dual-case lookup; the
% ROS2 branch is exercised separately below.
msg.MessageType = 'geometry_msgs/PoseStamped';
msg.Pose.Position.X = 1.0;
msg.Pose.Position.Y = 2.0;
msg.Pose.Position.Z = 3.0;
msg.Pose.Orientation.W = 0.7071;
msg.Pose.Orientation.X = 0.0;
msg.Pose.Orientation.Y = 0.7071;
msg.Pose.Orientation.Z = 0.0;

msg_cell = {msg};

expected_wxyz = [1.0; 2.0; 3.0; 0.7071; 0.0; 0.7071; 0.0];
expected_xyzw = [1.0; 2.0; 3.0; 0.0; 0.7071; 0.0; 0.7071];

result_wxyz = Bag_Analyzer.decodePoseStamped(msg_cell, "wxyz");
[n_pass, n_fail] = check(n_pass, n_fail, 'decodePoseStamped (ROS1 PascalCase) wxyz order', ...
    isequal(result_wxyz, expected_wxyz), true);

result_xyzw = Bag_Analyzer.decodePoseStamped(msg_cell, "xyzw");
[n_pass, n_fail] = check(n_pass, n_fail, 'decodePoseStamped (ROS1 PascalCase) xyzw order', ...
    isequal(result_xyzw, expected_xyzw), true);

%% ---- (c) decodePoseStamped: ROS2-shaped (lowercase) struct message ------

% Same message, but with the field-name layout a REAL ROS2 bag actually
% produces (verified: rosbag2matlab/bags/ros2bags/
% collocated_cycle_2026_06_26-19_09_51/, see inspect_real_ros2_bag.m).
% Exercises Bag_Analyzer.gf()'s ROS2 fallback branch.
msg_ros2.MessageType = 'geometry_msgs/PoseStamped';
msg_ros2.pose.position.x = 1.0;
msg_ros2.pose.position.y = 2.0;
msg_ros2.pose.position.z = 3.0;
msg_ros2.pose.orientation.w = 0.7071;
msg_ros2.pose.orientation.x = 0.0;
msg_ros2.pose.orientation.y = 0.7071;
msg_ros2.pose.orientation.z = 0.0;

msg_cell_ros2 = {msg_ros2};

result_wxyz_ros2 = Bag_Analyzer.decodePoseStamped(msg_cell_ros2, "wxyz");
[n_pass, n_fail] = check(n_pass, n_fail, 'decodePoseStamped (ROS2 lowercase) wxyz order', ...
    isequal(result_wxyz_ros2, expected_wxyz), true);

result_xyzw_ros2 = Bag_Analyzer.decodePoseStamped(msg_cell_ros2, "xyzw");
[n_pass, n_fail] = check(n_pass, n_fail, 'decodePoseStamped (ROS2 lowercase) xyzw order', ...
    isequal(result_xyzw_ros2, expected_xyzw), true);

%% ---- (d) Bag_Analyzer.gf(): dual-case field accessor ---------------------

s1.Position = 42;
s2.position = 42;
s3.SomethingElse = 1;

[n_pass, n_fail] = check(n_pass, n_fail, 'gf() finds ROS1 PascalCase field', ...
    Bag_Analyzer.gf(s1, 'Position'), 42);
[n_pass, n_fail] = check(n_pass, n_fail, 'gf() falls back to ROS2 lowercase field', ...
    Bag_Analyzer.gf(s2, 'Position'), 42);

try
    Bag_Analyzer.gf(s3, 'Position');
    fprintf('[FAIL] gf() should error when neither field variant exists\n');
    n_fail = n_fail + 1;
catch
    fprintf('[PASS] gf() correctly errors when neither field variant exists\n');
    n_pass = n_pass + 1;
end

%% ---- Summary --------------------------------------------------------------
fprintf('\n%d passed, %d failed\n', n_pass, n_fail);
if n_fail > 0
    error('test_bag_format_detection:AssertionsFailed', ...
        '%d assertion(s) failed', n_fail);
end

%% ---- Local helpers ------------------------------------------------------
function [n_pass, n_fail] = check(n_pass, n_fail, label, actual, expected)
    if isequal(actual, expected)
        fprintf('[PASS] %s\n', label);
        n_pass = n_pass + 1;
    else
        fprintf('[FAIL] %s (got %s, expected %s)\n', label, ...
            fmt_val(actual), fmt_val(expected));
        n_fail = n_fail + 1;
    end
end

function s = fmt_val(v)
    % Robust-to-print formatter: mat2str() does not reliably support the
    % "string" type across MATLAB versions, and our expected/actual values
    % here are a mix of string ("ros1"/"ros2"), logical (true), and numeric
    % (pose vectors).
    if isstring(v) || ischar(v)
        s = char(v);
    elseif islogical(v)
        if v
            s = 'true';
        else
            s = 'false';
        end
    else
        s = mat2str(v);
    end
end
