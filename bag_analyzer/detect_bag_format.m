%%% ROS Bag Format Detector %%%
function fmt = detect_bag_format(bag_path)
%DETECT_BAG_FORMAT Determine whether bag_path is a ROS 1 or ROS 2 bag.
%
%   fmt = DETECT_BAG_FORMAT(bag_path) inspects bag_path and returns:
%       "ros1" - bag_path is a single ROS 1 bag FILE (extension ".bag",
%                readable with the ROS 1 rosbag() reader).
%       "ros2" - bag_path is a DIRECTORY holding a ROS 2 bag, i.e. it
%                contains a "metadata.yaml" file and/or one or more
%                "*.db3" (sqlite3) or "*.mcap" storage files, readable
%                with the ROS 2 ros2bagreader() reader.
%
%   Errors clearly if bag_path does not exist, or exists but matches
%   neither the ROS 1 file pattern nor the ROS 2 directory pattern.
%
%   This function does no ROS-specific work itself (no rosbag/ros2bagreader
%   calls) -- it is pure file-system inspection, so it can be exercised in
%   tests without any bag or ROS Toolbox installation.

    arguments
        bag_path (1,1) string
    end

    if ~isfile(bag_path) && ~isfolder(bag_path)
        error('detect_bag_format:NotFound', ...
            'Bag path does not exist: %s', bag_path);
    end

    % --- ROS 1: a single *.bag file -------------------------------------
    if isfile(bag_path)
        [~, ~, ext] = fileparts(bag_path);
        if strcmpi(ext, '.bag')
            fmt = "ros1";
            return;
        end
        error('detect_bag_format:UnknownFormat', ...
            'File "%s" is not a recognized ROS 1 bag (.bag) file.', bag_path);
    end

    % --- ROS 2: a directory containing metadata.yaml and/or .db3/.mcap --
    % (bag_path is guaranteed to be a folder at this point)
    has_metadata = isfile(fullfile(bag_path, 'metadata.yaml'));
    has_db3  = ~isempty(dir(fullfile(bag_path, '*.db3')));
    has_mcap = ~isempty(dir(fullfile(bag_path, '*.mcap')));

    if has_metadata || has_db3 || has_mcap
        fmt = "ros2";
        return;
    end

    error('detect_bag_format:UnknownFormat', ...
        ['Directory "%s" does not look like a ROS 2 bag (expected a ' ...
         '"metadata.yaml" file and/or "*.db3" / "*.mcap" storage files).'], ...
        bag_path);
end
