%%%  Bag Analyzer %%%

classdef Bag_Analyzer < handle
    %% Attributes
    properties
        %% Bag Object
        bag_obj

        %% Bag Format ("ros1" | "ros2") - see detect_bag_format.m.
        % Set once in the constructor and used to dispatch the handful of
        % call sites where the ROS 1 / ROS 2 reader APIs genuinely differ
        % (currently: only how the reader object itself is opened).
        bag_format

        %% Time Information
        start_time
        end_time
        bag_duration

        %% Topics Information
        topic_names
        n_topics
        msg_type
        n_msgs

        %% Timeseries
        topics_ts
        synchronized_topics

        %% Marker & VICON Utilities
        marker_dictionary

        %% Preferences
        quaternion_order
        use_parallel
    end

    %% Methods
    methods
        % Constructor
        function obj = Bag_Analyzer(bag_name, options)
            arguments
                bag_name
                options.quaternion_order = "wxyz";
                options.use_parallel = false;
            end
            % --- DISPATCH POINT 1: open the bag -----------------------------
            % Detect ROS 1 (*.bag file) vs ROS 2 (bag directory) from the
            % path itself, then open with the matching reader. Everything
            % downstream (select/readMessages/MessageList/extractData) is
            % shared and format-agnostic -- see the mapping notes at each
            % remaining dispatch point below.
            obj.bag_format = detect_bag_format(bag_name);
            switch obj.bag_format
                case "ros1"
                    obj.bag_obj = rosbag(bag_name);
                case "ros2"
                    % ASSUMPTION (unverified without a MATLAB + ROS 2
                    % Toolbox install): ros2bagreader() takes the bag
                    % FOLDER path (containing metadata.yaml) exactly like
                    % rosbag() takes a .bag file, and exposes the same
                    % StartTime/EndTime/AvailableTopics surface used below.
                    obj.bag_obj = ros2bagreader(bag_name);
            end

            % Quaternion Order
            obj.quaternion_order = options.quaternion_order;
            obj.use_parallel = options.use_parallel;

            % --- DISPATCH POINT 2: time information -------------------------
            % Both rosbag (ROS1) and ros2bagreader (ROS2) expose StartTime /
            % EndTime as numeric seconds, so no format branch is needed here
            % -- but route through to_seconds() as a defensive normalizer in
            % case the ROS2 reader ever returns duration/datetime instead of
            % a plain double (ASSUMPTION: not verifiable without MATLAB).
            obj.start_time = Bag_Analyzer.to_seconds(obj.bag_obj.StartTime);
            obj.end_time = Bag_Analyzer.to_seconds(obj.bag_obj.EndTime);
            obj.bag_duration = obj.end_time - obj.start_time;

            % --- DISPATCH POINT 3: topic list --------------------------------
            % rosbag's BagSelection exposes AvailableTopics as a table whose
            % row names are the topic names (ROS1: bag.AvailableTopics.Row).
            % ASSUMPTION (unverified without a MATLAB + ROS 2 Toolbox
            % install): ros2bagreader exposes an AvailableTopics table with
            % the same Row/Topic shape, per the ROS Toolbox documentation's
            % description of the two readers as API-parallel. If this turns
            % out to differ, isolate the ROS2 case behind obj.bag_format here.
            obj.topic_names = obj.bag_obj.AvailableTopics.Row';
            obj.n_topics = length(obj.topic_names);

            % Init Marker Dictionary
            obj.marker_dictionary = struct();

            % Extract Topics & Msgs
            obj.extractMsgs();
        end

        function msg_data = extractData(obj, msg_cell)
            % Init
            num_msgs = length(msg_cell);
            if num_msgs == 0
                msg_data = [];
                return;
            end

            switch msg_cell{1}.MessageType
                case {'sensor_msgs/Image', 'sensor_msgs/CompressedImage'}
                    % 1. Get image size
                    first_img = rosReadImage(msg_cell{1});
                    img_size = size(first_img);

                    % 2. Preallocate using uint8
                    msg_data = zeros([img_size, num_msgs], 'uint8');

                    % 3. Extract Image
                    if obj.use_parallel
                        parfor i = 1:num_msgs
                            msg_data(:, :, :, i) = rosReadImage(msg_cell{i});
                        end
                    else
                        for i = 1:num_msgs
                            msg_data(:, :, :, i) = rosReadImage(msg_cell{i});
                        end
                    end

                case 'std_msgs/Float32MultiArray'
                    % Preallocate based on the first message
                    first_msg = double(msg_cell{1}.Data);
                    msg_data = NaN(length(first_msg), num_msgs);

                    for i = 1:num_msgs
                        msg = double(msg_cell{i}.Data);
                        if ~isempty(msg)
                            n = min(numel(msg), size(msg_data, 1));
                            msg_data(1:n, i) = msg(1:n);
                        end
                    end

                case 'std_msgs/Float64MultiArray'
                    first_msg = double(msg_cell{1}.Data);
                    msg_data = NaN(length(first_msg), num_msgs);

                    for i = 1:num_msgs
                        msg = double(msg_cell{i}.Data);
                        if ~isempty(msg)
                            n = min(numel(msg), size(msg_data, 1));
                            msg_data(1:n, i) = msg(1:n);
                        end
                    end

                case 'sensor_msgs/JointState'
                    % Determine total rows needed from position, velocity, effort arrays
                    pos_len = length(msg_cell{1}.Position);
                    vel_len = length(msg_cell{1}.Velocity);
                    eff_len = length(msg_cell{1}.Effort);
                    msg_data = zeros(pos_len + vel_len + eff_len, num_msgs);

                    for i = 1:num_msgs
                        msg_data(:, i) = [msg_cell{i}.Position; msg_cell{i}.Velocity; msg_cell{i}.Effort];
                    end

                case 'geometry_msgs/PoseStamped'
                    % Message struct field layout for geometry_msgs/PoseStamped
                    % is identical whether the struct came from a ROS1 or a
                    % ROS2 bag (MATLAB's DataFormat='struct' output unifies
                    % both), so the decode logic is shared. Factored into a
                    % Static method so it can be unit-tested (see
                    % tests/test_bag_format_detection.m) without needing a
                    % live Bag_Analyzer instance / real bag file.
                    msg_data = Bag_Analyzer.decodePoseStamped(msg_cell, obj.quaternion_order);

                case 'geometry_msgs/TransformStamped'
                    msg_data = zeros(7, num_msgs);

                    for i = 1:num_msgs
                        se3_data = msg_cell{i}.Transform;
                        msg_data(:, i) = [se3_data.Translation.X; se3_data.Translation.Y; se3_data.Translation.Z;
                                        se3_data.Rotation.W; se3_data.Rotation.X; se3_data.Rotation.Y; se3_data.Rotation.Z];
                    end

                case 'geometry_msgs/PointStamped'
                    msg_data = zeros(3, num_msgs);

                    for i = 1:num_msgs
                        msg_data(:, i) = [msg_cell{i}.Point.X; msg_cell{i}.Point.Y; msg_cell{i}.Point.Z];
                    end

                case 'geometry_msgs/WrenchStamped'
                    msg_data = zeros(6, num_msgs); % 3 force + 3 torque

                    for i = 1:num_msgs
                        msg_data(:, i) = [msg_cell{i}.Wrench.Force.X; msg_cell{i}.Wrench.Force.Y; msg_cell{i}.Wrench.Force.Z;
                                        msg_cell{i}.Wrench.Torque.X; msg_cell{i}.Wrench.Torque.Y; msg_cell{i}.Wrench.Torque.Z];
                    end

                case 'vicon_bridge/Markers'
                    [msg_data, obj.marker_dictionary] = marker_management(msg_cell, "skip_unknown", true);

                case 'sensor_msgs/PointCloud'
                    % Preallocate based on the number of points in the first message
                    n_points = length(msg_cell{1}.Points);
                    msg_data = zeros(n_points * 3, num_msgs);

                    for i = 1:num_msgs
                        pts = msg_cell{i}.Points;
                        points_pos = zeros(length(pts) * 3, 1);

                        for j = 1:length(pts)
                            idx = (j-1)*3 + 1;
                            points_pos(idx:idx+2) = [pts(j).X; pts(j).Y; pts(j).Z];
                        end

                        % Handle potential size mismatches safely if N points changes over time
                        if length(points_pos) == size(msg_data, 1)
                            msg_data(:, i) = points_pos;
                        else
                            n = min(length(points_pos), size(msg_data, 1));
                            msg_data(1:n, i) = points_pos(1:n);
                        end
                    end

                case 'sensor_msgs/PointCloud2'
                    % Determine data size efficiently using the first message
                    first_points = double(rosReadXYZ(msg_cell{1})');
                    msg_data = zeros(numel(first_points), num_msgs);

                    for i = 1:num_msgs
                        points = double(rosReadXYZ(msg_cell{i})');
                        points = points(:);
                        % Guard against a varying number of points per message
                        if numel(points) == size(msg_data, 1)
                            msg_data(:, i) = points;
                        else
                            n = min(numel(points), size(msg_data, 1));
                            msg_data(1:n, i) = points(1:n);
                        end
                    end

                case 'dynamic_manipulation_dlo/MarkerRigidBodyPoses'
                    msg_data = zeros(7, num_msgs);
                    vicon_format_markers = cell(1, num_msgs); % Preallocate cell array

                    for i = 1:num_msgs
                        % Save RigidBodyPose
                        if obj.quaternion_order == "wxyz"
                            msg_data(:, i) = [msg_cell{i}.RigidBodyPose.Position.X; msg_cell{i}.RigidBodyPose.Position.Y; msg_cell{i}.RigidBodyPose.Position.Z;
                                            msg_cell{i}.RigidBodyPose.Orientation.W; msg_cell{i}.RigidBodyPose.Orientation.X; msg_cell{i}.RigidBodyPose.Orientation.Y; msg_cell{i}.RigidBodyPose.Orientation.Z];
                        elseif obj.quaternion_order == "xyzw"
                            msg_data(:, i) = [msg_cell{i}.RigidBodyPose.Position.X; msg_cell{i}.RigidBodyPose.Position.Y; msg_cell{i}.RigidBodyPose.Position.Z;
                                            msg_cell{i}.RigidBodyPose.Orientation.X; msg_cell{i}.RigidBodyPose.Orientation.Y; msg_cell{i}.RigidBodyPose.Orientation.Z; msg_cell{i}.RigidBodyPose.Orientation.W];
                        end

                        % Marker Management
                        if(~isempty(msg_cell{i}.MarkerIds))
                            n_markers = length(msg_cell{i}.MarkerIds);
                            % Preallocate the struct array to avoid fragmentation
                            vicon_format_markers{i}.Markers_ = repmat(struct('Occluded', 0, 'SubjectName', '', 'MarkerName', '', 'Translation', struct('X', 0, 'Y', 0, 'Z', 0)), 1, n_markers);

                            for j = 1:n_markers
                                vicon_format_markers{i}.Markers_(j).Occluded = 0;
                                vicon_format_markers{i}.Markers_(j).SubjectName = '';
                                vicon_format_markers{i}.Markers_(j).MarkerName = "marker_" + msg_cell{i}.MarkerIds(j);
                                vicon_format_markers{i}.Markers_(j).Translation.X = msg_cell{i}.MarkerPoses.Poses(j).Position.X * 1000;
                                vicon_format_markers{i}.Markers_(j).Translation.Y = msg_cell{i}.MarkerPoses.Poses(j).Position.Y * 1000;
                                vicon_format_markers{i}.Markers_(j).Translation.Z = msg_cell{i}.MarkerPoses.Poses(j).Position.Z * 1000;
                            end
                        else
                            % Keep a valid (empty) entry so downstream cellfun checks don't fail
                            vicon_format_markers{i}.Markers_ = repmat(struct('Occluded', 0, 'SubjectName', '', 'MarkerName', '', 'Translation', struct('X', 0, 'Y', 0, 'Z', 0)), 1, 0);
                        end
                    end

                    % Marker Management Finalization
                    if(~all(cellfun(@(x) isempty(x.MarkerIds), msg_cell)))
                        [~, obj.marker_dictionary] = marker_management(vicon_format_markers, "skip_unknown", true, ...
                                                                        "replace_style", 'delete', "preserve_order", true);
                    end

                otherwise
                    msg_data = msg_cell;
            end
        end

        % Extract Topics & Msgs
        function extractMsgs(obj)
            for i = 1:obj.n_topics
                % --- DISPATCH POINT 4: topic selection + message read -------
                % select(...,'Topic',...) and readMessages(...,'DataFormat',
                % 'struct') are documented by the ROS Toolbox to work on
                % both the ROS1 BagSelection and the ROS2 reader/selection
                % object with identical syntax, so no format branch is
                % needed here for either obj.bag_format value.
                % ASSUMPTION (unverified without MATLAB + ROS 2 Toolbox):
                % the ROS2 selection object returned by select() supports
                % the same readMessages(...,'DataFormat','struct') call.
                topic_cell = select(obj.bag_obj, 'Topic', obj.topic_names{i});
                % msgs
                msg_cell = readMessages(topic_cell,'DataFormat','struct');

                % Guard against empty topics
                if isempty(msg_cell)
                    obj.msg_type{i} = '';
                    obj.n_msgs(i) = 0;
                    obj.topics_ts{i} = struct('Time', [], 'Data', []);
                    continue;
                end

                % --- DISPATCH POINT 5: message timestamps -------------------
                % sel.MessageList.Time is documented as the same table/
                % column shape for both ROS1 BagSelection and ROS2
                % selection objects, so this subtraction is format-agnostic.
                % ASSUMPTION (unverified without MATLAB + ROS 2 Toolbox):
                % ROS2's MessageList.Time uses the same numeric-seconds
                % units as ROS1's (both relative to obj.start_time, itself
                % normalized via Bag_Analyzer.to_seconds above).
                % Time
                topic_time = topic_cell.MessageList.Time - obj.start_time;
                % Type
                obj.msg_type{i} = msg_cell{1}.MessageType;
                % N° of Msgs
                obj.n_msgs(i) = length(msg_cell);
                % Build Timeseries
                obj.topics_ts{i} = struct('Time', topic_time', 'Data', obj.extractData(msg_cell));
            end
        end

        % Synchronization
        function [merged_time, merged_dataset, sync_marker_dic, topics] = synchronization(obj, resampling_period, mask, options)
            arguments
                obj
                resampling_period = 1.0e-2;          % 10 ms -> 100 Hz (was 1e2 = 100 s, a bug)
                mask = true(1, obj.n_topics);
                options.interpolation_method = 'linear';
            end

            % Merged Time Definition
            merged_time = 0:resampling_period:obj.bag_duration;
            merged_dataset = cell(1, obj.n_topics);
            topics = cell(1, obj.n_topics);

            % Interpolate
            for i = 1:obj.n_topics
                % Skip empty / unsupported topics: only numeric Data can be interpolated.
                ts_i = obj.topics_ts{i};

                % Must be a SCALAR struct. If ts_i is a struct ARRAY, then
                % ts_i.Data expands to a comma-separated list, and any builtin
                % called on it (isnumeric, class, isempty, ...) receives multiple
                % arguments and errors ("Too many input arguments" /
                % "Arguments must contain a character vector"). Requiring a
                % scalar struct here is what prevents that.
                if isempty(ts_i) || ~isstruct(ts_i) || ~isscalar(ts_i) || ~mask(i)
                    continue;
                end
                if ~isfield(ts_i, 'Data') || ~isfield(ts_i, 'Time')
                    continue;
                end

                % Pull fields into locals now that ts_i is guaranteed scalar.
                data_i = ts_i.Data;
                time_i = ts_i.Time;

                % Class-name check instead of isnumeric(): isnumeric dispatches
                % on the argument's class, and some array types (dlarray,
                % gpuArray, distributed, ...) ship their own isnumeric method.
                % Comparing the class name cannot mis-dispatch.
                numeric_classes = {'double','single', ...
                                   'int8','int16','int32','int64', ...
                                   'uint8','uint16','uint32','uint64'};
                if ~any(strcmp(class(data_i), numeric_classes))
                    continue;
                end
                if isempty(data_i) || isempty(time_i)
                    continue;
                end

                % Merge Dataset
                if (obj.msg_type{i} == "sensor_msgs/Image") || (obj.msg_type{i} == "sensor_msgs/CompressedImage")

                    % Get indices
                    fake_data = (1:length(time_i))';

                    % Zero-order hold is intentional for images: hold the most
                    % recent frame ('previous'), NOT 'nearest' (which would pull
                    % a future frame and break causality).
                    fake_dataset = interp1(time_i', fake_data, merged_time', 'previous');

                    % Preallocate the synchronized dataset with zeros (uint8)
                    single_frame_size = size(data_i(:, :, :, 1));
                    num_sync_frames = length(merged_time);
                    merged_dataset{i} = zeros([single_frame_size, num_sync_frames], 'uint8');

                    % Fill the dataset (NaN -> no previous frame yet -> leave black)
                    for j = 1:num_sync_frames
                        if ~isnan(fake_dataset(j))
                            merged_dataset{i}(:, :, :, j) = data_i(:, :, :, fake_dataset(j));
                        end
                    end

                    clear fake_data fake_dataset single_frame_size num_sync_frames
                else
                    % NaN-aware interpolation.
                    % Occluded/absent markers produce NaN columns. Plain interp1
                    % over such data propagates NaN and, once forward-filled
                    % anywhere downstream, looks like stepped (ZOH) data.
                    % We interpolate each row over only its valid samples so gaps
                    % are bridged smoothly instead of held.
                    merged_dataset{i} = obj.interp_rows_nan_aware( ...
                        time_i', data_i, ...
                        merged_time', options.interpolation_method);
                end

                % Store topic names
                topics{i} = obj.topic_names{i};
            end

            %% Synchronize Marker Dictionary
            vicon_idx     = find(strcmp(obj.msg_type, "vicon_bridge/Markers"), 1, 'last');
            optitrack_idx = find(strcmp(obj.msg_type, "dynamic_manipulation_dlo/MarkerRigidBodyPoses"), 1, 'last');

            % Choose a SINGLE source topic for the marker time base.
            % (Was idx = [vicon_idx, optitrack_idx]; indexing topics_ts{idx}
            %  with a 2-vector crashed when both existed.)
            marker_time_idx = optitrack_idx;
            if isempty(marker_time_idx)
                marker_time_idx = vicon_idx;
            end

            if ~isempty(marker_time_idx) && ~isempty(fieldnames(obj.marker_dictionary))
                src_time = obj.topics_ts{marker_time_idx}.Time';

                % marker_dictionary is a struct -> use fieldnames + dot access
                % (the original dictionary()/fieldnames() mix was inconsistent).
                marker_names = fieldnames(obj.marker_dictionary);

                for j = 1:numel(marker_names)
                    obj.marker_dictionary.(marker_names{j}) = obj.interp_rows_nan_aware( ...
                        src_time, obj.marker_dictionary.(marker_names{j}), ...
                        merged_time', options.interpolation_method);
                end
            end

            % Remove Skipped Topics
            keep = ~cellfun('isempty', merged_dataset);
            merged_dataset = merged_dataset(keep);
            topics = topics(keep);

            % Updated Marker Dictionary
            sync_marker_dic = obj.marker_dictionary;

            % Store for convenience
            obj.synchronized_topics = topics;
        end
    end

    methods (Static, Access = private)
        function out = interp_rows_nan_aware(src_time, data, query_time, method)
            % Interpolate row-wise data (rows = signals, cols = time samples)
            % onto query_time, handling NaN gaps per row.
            %
            % src_time  : column vector [N x 1]
            % data      : [R x N]  (R signals over N source samples)
            % query_time: column vector [M x 1]
            % out       : [R x M]

            src_time = src_time(:);
            query_time = query_time(:);

            R = size(data, 1);
            M = numel(query_time);
            out = NaN(R, M);

            for r = 1:R
                y = data(r, :);
                valid = ~isnan(y) & (~isnan(src_time(:)'));

                switch nnz(valid)
                    case 0
                        % No data at all -> leave NaN
                        continue;
                    case 1
                        % Single sample -> constant value across the range
                        out(r, :) = y(valid);
                    otherwise
                        % Interpolate over valid samples only (bridges NaN gaps
                        % smoothly instead of holding a stepped value).
                        out(r, :) = interp1(src_time(valid), y(valid), ...
                                            query_time, method);
                end
            end
        end
    end

    methods (Static)
        function msg_data = decodePoseStamped(msg_cell, quaternion_order)
            % Decode a cell array of geometry_msgs/PoseStamped struct
            % messages (DataFormat='struct') into a 7xN [pos; quat] matrix.
            %
            % Pulled out of extractData's switch-case as a public Static
            % method (no dependency on a live Bag_Analyzer instance / bag
            % object) so it can be exercised directly by
            % tests/test_bag_format_detection.m with a synthetic message
            % struct -- including a synthetic ROS2-shaped one, since the
            % struct field layout for this message type is identical
            % between ROS1 and ROS2 DataFormat='struct' output.
            %
            % msg_cell        : cell array of PoseStamped structs
            % quaternion_order: "wxyz" or "xyzw"
            num_msgs = length(msg_cell);
            msg_data = zeros(7, num_msgs); % 3 for pos + 4 for quat

            for i = 1:num_msgs
                if quaternion_order == "wxyz"
                    msg_data(:, i) = [msg_cell{i}.Pose.Position.X; msg_cell{i}.Pose.Position.Y; msg_cell{i}.Pose.Position.Z;
                                    msg_cell{i}.Pose.Orientation.W; msg_cell{i}.Pose.Orientation.X; msg_cell{i}.Pose.Orientation.Y; msg_cell{i}.Pose.Orientation.Z];
                elseif quaternion_order == "xyzw"
                    msg_data(:, i) = [msg_cell{i}.Pose.Position.X; msg_cell{i}.Pose.Position.Y; msg_cell{i}.Pose.Position.Z;
                                    msg_cell{i}.Pose.Orientation.X; msg_cell{i}.Pose.Orientation.Y; msg_cell{i}.Pose.Orientation.Z; msg_cell{i}.Pose.Orientation.W];
                end
            end
        end
    end

    methods (Static, Access = private)
        function t = to_seconds(val)
            % Normalize a bag StartTime/EndTime value to a plain numeric
            % scalar in seconds, regardless of whether the underlying
            % reader returned a double (ROS1's documented behavior), or a
            % duration/datetime (defensive: unverified for ROS2's
            % ros2bagreader without a MATLAB + ROS 2 Toolbox install).
            if isduration(val)
                t = seconds(val);
            elseif isdatetime(val)
                t = posixtime(val);
            else
                t = double(val);
            end
        end
    end
end