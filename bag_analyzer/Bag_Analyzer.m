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
                    % VERIFIED (probe_ros2_assumptions.m, synthetic
                    % ros2bagwriter-produced bag): ros2bagreader() takes
                    % the bag FOLDER path (containing metadata.yaml) and
                    % exposes StartTime/EndTime/AvailableTopics with the
                    % same shapes used below.
                    obj.bag_obj = ros2bagreader(bag_name);
            end

            % Quaternion Order
            obj.quaternion_order = options.quaternion_order;
            obj.use_parallel = options.use_parallel;

            % --- DISPATCH POINT 2: time information -------------------------
            % VERIFIED (probe_ros2_assumptions.m): both rosbag (ROS1) and
            % ros2bagreader (ROS2) expose StartTime/EndTime as plain double
            % seconds, so no format branch is needed here -- to_seconds()
            % is kept as a defensive normalizer regardless.
            obj.start_time = Bag_Analyzer.to_seconds(obj.bag_obj.StartTime);
            obj.end_time = Bag_Analyzer.to_seconds(obj.bag_obj.EndTime);
            obj.bag_duration = obj.end_time - obj.start_time;

            % --- DISPATCH POINT 3: topic list --------------------------------
            % VERIFIED (probe_ros2_assumptions.m): ros2bagreader's
            % AvailableTopics table exposes the same Row-is-topic-names
            % shape as ROS1's rosbag/BagSelection.AvailableTopics.
            obj.topic_names = obj.bag_obj.AvailableTopics.Row';
            obj.n_topics = length(obj.topic_names);

            % Init Marker Dictionary
            obj.marker_dictionary = struct();

            % Extract Topics & Msgs
            obj.extractMsgs();
        end

        function msg_data = extractData(obj, msg_cell)
            % FIXED for ROS2 (was: CONFIRMED CRASH, probe_bag_analyzer_e2e.m).
            % Every case below that reaches into message sub-fields
            % (Pose.Position.X, Transform.Translation.X, Wrench.Force.X,
            % Position/Velocity/Effort, Point.X, ...) used to assume ROS1's
            % PascalCase struct field naming, which crashed on a real ROS2
            % bag's lowercase field names ("Unrecognized field name
            % 'Pose'"). All STANDARD message types below (std_msgs,
            % sensor_msgs/JointState, geometry_msgs/*) now go through
            % Bag_Analyzer.gf(), a dual-case field accessor: tries the
            % ROS1 PascalCase name first, falls back to the same name with
            % its first letter lowercased (ROS2's convention for these
            % single-word field names) otherwise. Verified two ways:
            % (1) a real ROS2 bag (rosbag2matlab/bags/ros2bags/
            % collocated_cycle_2026_06_26-19_09_51/, see
            % inspect_real_ros2_bag.m) directly confirms sensor_msgs/
            % JointState (position/velocity/effort) and std_msgs/
            % Float64MultiArray (data), plus the nested pose.position/
            % pose.orientation pattern via a visualization_msgs/Marker
            % message on that bag; (2) the remaining geometry_msgs types
            % (Transform, Wrench, Point) follow the same single-word
            % lower-first-letter convention per the public ROS2
            % geometry_msgs interface definitions (translation/rotation,
            % force/torque, x/y/z), not just an assumption.
            %
            % NOT fixed -- left PascalCase-only, still ROS1-shaped:
            % 'vicon_bridge/Markers' and 'dynamic_manipulation_dlo/
            % MarkerRigidBodyPoses'. Both are custom message types with no
            % ROS2 equivalent found in the real bag above (it uses
            % different custom types instead: candle_ros2/*,
            % mocap_optitrack_interfaces/*, neither of which MATLAB could
            % even decode without ros2genmsg). Their multi-word fields
            % (MarkerIds, MarkerPoses, RigidBodyPose) may follow ROS2's
            % snake_case convention rather than the simple lower-first-
            % letter rule verified above for single-word fields -- do not
            % assume gf() applies here without checking the real .msg
            % source or a bag that actually contains one of these topics.
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
                    first_msg = double(Bag_Analyzer.gf(msg_cell{1}, 'Data'));
                    msg_data = NaN(length(first_msg), num_msgs);

                    for i = 1:num_msgs
                        msg = double(Bag_Analyzer.gf(msg_cell{i}, 'Data'));
                        if ~isempty(msg)
                            n = min(numel(msg), size(msg_data, 1));
                            msg_data(1:n, i) = msg(1:n);
                        end
                    end

                case 'std_msgs/Float64MultiArray'
                    % Field name VERIFIED against a real ROS2 bag (data, not Data).
                    first_msg = double(Bag_Analyzer.gf(msg_cell{1}, 'Data'));
                    msg_data = NaN(length(first_msg), num_msgs);

                    for i = 1:num_msgs
                        msg = double(Bag_Analyzer.gf(msg_cell{i}, 'Data'));
                        if ~isempty(msg)
                            n = min(numel(msg), size(msg_data, 1));
                            msg_data(1:n, i) = msg(1:n);
                        end
                    end

                case 'sensor_msgs/JointState'
                    % Field names VERIFIED against a real ROS2 bag
                    % (position/velocity/effort, not Position/Velocity/Effort).
                    % Determine total rows needed from position, velocity, effort arrays
                    pos_len = length(Bag_Analyzer.gf(msg_cell{1}, 'Position'));
                    vel_len = length(Bag_Analyzer.gf(msg_cell{1}, 'Velocity'));
                    eff_len = length(Bag_Analyzer.gf(msg_cell{1}, 'Effort'));
                    msg_data = zeros(pos_len + vel_len + eff_len, num_msgs);

                    for i = 1:num_msgs
                        msg_data(:, i) = [Bag_Analyzer.gf(msg_cell{i}, 'Position'); ...
                                           Bag_Analyzer.gf(msg_cell{i}, 'Velocity'); ...
                                           Bag_Analyzer.gf(msg_cell{i}, 'Effort')];
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
                    % Field names per the public ROS2 geometry_msgs/Transform
                    % interface definition (translation/rotation, not
                    % Translation/Rotation) -- same single-word
                    % lower-first-letter convention verified above.
                    msg_data = zeros(7, num_msgs);

                    for i = 1:num_msgs
                        se3_data = Bag_Analyzer.gf(msg_cell{i}, 'Transform');
                        trans = Bag_Analyzer.gf(se3_data, 'Translation');
                        rot = Bag_Analyzer.gf(se3_data, 'Rotation');
                        msg_data(:, i) = [Bag_Analyzer.gf(trans,'X'); Bag_Analyzer.gf(trans,'Y'); Bag_Analyzer.gf(trans,'Z');
                                        Bag_Analyzer.gf(rot,'W'); Bag_Analyzer.gf(rot,'X'); Bag_Analyzer.gf(rot,'Y'); Bag_Analyzer.gf(rot,'Z')];
                    end

                case 'geometry_msgs/PointStamped'
                    msg_data = zeros(3, num_msgs);

                    for i = 1:num_msgs
                        pt = Bag_Analyzer.gf(msg_cell{i}, 'Point');
                        msg_data(:, i) = [Bag_Analyzer.gf(pt,'X'); Bag_Analyzer.gf(pt,'Y'); Bag_Analyzer.gf(pt,'Z')];
                    end

                case 'geometry_msgs/WrenchStamped'
                    % Field names per the public ROS2 geometry_msgs/Wrench
                    % interface definition (force/torque, not Force/Torque).
                    msg_data = zeros(6, num_msgs); % 3 force + 3 torque

                    for i = 1:num_msgs
                        w = Bag_Analyzer.gf(msg_cell{i}, 'Wrench');
                        f = Bag_Analyzer.gf(w, 'Force');
                        t = Bag_Analyzer.gf(w, 'Torque');
                        msg_data(:, i) = [Bag_Analyzer.gf(f,'X'); Bag_Analyzer.gf(f,'Y'); Bag_Analyzer.gf(f,'Z');
                                        Bag_Analyzer.gf(t,'X'); Bag_Analyzer.gf(t,'Y'); Bag_Analyzer.gf(t,'Z')];
                    end

                case 'vicon_bridge/Markers'
                    % NOT fixed for ROS2 -- see the header comment above
                    % extractData: custom message, no ROS2 equivalent found
                    % to verify field names against.
                    [msg_data, obj.marker_dictionary] = marker_management(msg_cell, "skip_unknown", true);

                case 'sensor_msgs/PointCloud'
                    % Preallocate based on the number of points in the first message
                    n_points = length(Bag_Analyzer.gf(msg_cell{1}, 'Points'));
                    msg_data = zeros(n_points * 3, num_msgs);

                    for i = 1:num_msgs
                        pts = Bag_Analyzer.gf(msg_cell{i}, 'Points');
                        points_pos = zeros(length(pts) * 3, 1);

                        for j = 1:length(pts)
                            idx = (j-1)*3 + 1;
                            points_pos(idx:idx+2) = [Bag_Analyzer.gf(pts(j),'X'); Bag_Analyzer.gf(pts(j),'Y'); Bag_Analyzer.gf(pts(j),'Z')];
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
                    % NOT fixed for ROS2 -- see the header comment above
                    % extractData: custom message, no ROS2 equivalent found
                    % to verify field names against.
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

                case 'candle_ros2/TrackedMarkerArray'
                    % Field names per candle_ros2/msg/TrackedMarkerArray.msg
                    % + TrackedMarker.msg (docs/ros2_custom_messages.md in
                    % the superproject): top-level 'markers' and per-marker
                    % 'id'/'position'/'velocity' are ROS2-only (no ROS1
                    % PascalCase original), accessed directly by their .msg
                    % snake_case spelling -- VERIFIED via ros2genmsg:
                    % ros2message('candle_ros2/TrackedMarkerArray') exposes
                    % header/markers, and TrackedMarker exposes
                    % id/position/velocity, exactly as accessed here. The
                    % nested position/
                    % velocity Point/Vector3 .X/.Y/.Z leaves ARE the
                    % already-verified single-word ROS2 convention, so those
                    % go through gf().
                    %
                    % Routed through marker_management() (the same "vicon
                    % format" pipeline as the dynamic_manipulation_dlo case
                    % above) rather than a raw numeric array, so downstream
                    % consumers (synchronization, obj.marker_dictionary) see
                    % a consistent marker representation regardless of which
                    % bag format/message type produced it. TrackedMarker's
                    % `velocity` field has no slot in the vicon Markers_
                    % format (Occluded/SubjectName/MarkerName/Translation
                    % only) and is DROPPED here -- flagged in case the user
                    % wants filtered marker velocities preserved separately
                    % (e.g. as their own topic/array) rather than discarded.
                    %
                    % Units: geometry_msgs/Point position is meters (ROS
                    % convention, matches the plain /xd_tip, /xd_m1..3
                    % Point topics in the same bag). marker_management
                    % divides Translation by 1000 to produce meters in
                    % msg_data (it expects vicon-style mm input, see its
                    % Step 3), so multiply by 1000 here to round-trip back
                    % to the original meters -- same convention the
                    % dynamic_manipulation_dlo case above already uses.
                    vicon_format_markers = cell(1, num_msgs);

                    for i = 1:num_msgs
                        markers = msg_cell{i}.markers;
                        n_markers_i = length(markers);
                        vicon_format_markers{i}.Markers_ = repmat(struct('Occluded', 0, 'SubjectName', '', 'MarkerName', '', 'Translation', struct('X', 0, 'Y', 0, 'Z', 0)), 1, n_markers_i);

                        for j = 1:n_markers_i
                            mk  = markers(j);
                            pos = mk.position; % geometry_msgs/Point, direct access (ROS2-only field)
                            vicon_format_markers{i}.Markers_(j).Occluded = 0;
                            vicon_format_markers{i}.Markers_(j).SubjectName = '';
                            vicon_format_markers{i}.Markers_(j).MarkerName = "marker_" + double(mk.id);
                            vicon_format_markers{i}.Markers_(j).Translation.X = Bag_Analyzer.gf(pos, 'X') * 1000;
                            vicon_format_markers{i}.Markers_(j).Translation.Y = Bag_Analyzer.gf(pos, 'Y') * 1000;
                            vicon_format_markers{i}.Markers_(j).Translation.Z = Bag_Analyzer.gf(pos, 'Z') * 1000;
                        end
                    end

                    [msg_data, obj.marker_dictionary] = marker_management(vicon_format_markers, "skip_unknown", true, ...
                                                                        "replace_style", 'delete', "preserve_order", true);

                case 'mocap_optitrack_interfaces/MarkerArray'
                    % Field names per mocap_optitrack_interfaces/msg/
                    % MarkerArray.msg + Marker.msg (docs/
                    % ros2_custom_messages.md in the superproject):
                    % top-level 'markers' and per-marker 'id'/'position' are
                    % ROS2-only, accessed directly by their .msg snake_case
                    % spelling -- VERIFIED via ros2genmsg:
                    % ros2message('mocap_optitrack_interfaces/Marker') exposes
                    % position/id/type, exactly as accessed here (the string
                    % 'type' field is dropped below). Nested position .X/.Y/.Z
                    % leaves ARE the already-verified single-word ROS2
                    % convention, so those go through gf(). The `type`
                    % field (string: Active/Labeled/Unlabeled) is
                    % non-numeric and has no slot in the vicon Markers_
                    % format -- dropped.
                    %
                    % Routed through marker_management() (same "vicon
                    % format" pipeline as dynamic_manipulation_dlo /
                    % TrackedMarkerArray above) so obj.marker_dictionary and
                    % synchronization see a consistent representation.
                    % Units: same meters -> *1000 -> marker_management
                    % /1000 -> meters round trip as TrackedMarkerArray
                    % above (geometry_msgs/Point is meters by ROS
                    % convention; not yet numerically confirmed for this
                    % specific topic since it can't be decoded in this
                    % install).
                    vicon_format_markers = cell(1, num_msgs);

                    for i = 1:num_msgs
                        markers = msg_cell{i}.markers;
                        n_markers_i = length(markers);
                        vicon_format_markers{i}.Markers_ = repmat(struct('Occluded', 0, 'SubjectName', '', 'MarkerName', '', 'Translation', struct('X', 0, 'Y', 0, 'Z', 0)), 1, n_markers_i);

                        for j = 1:n_markers_i
                            mk  = markers(j);
                            pos = mk.position; % geometry_msgs/Point, direct access (ROS2-only field)
                            vicon_format_markers{i}.Markers_(j).Occluded = 0;
                            vicon_format_markers{i}.Markers_(j).SubjectName = '';
                            vicon_format_markers{i}.Markers_(j).MarkerName = "marker_" + double(mk.id);
                            vicon_format_markers{i}.Markers_(j).Translation.X = Bag_Analyzer.gf(pos, 'X') * 1000;
                            vicon_format_markers{i}.Markers_(j).Translation.Y = Bag_Analyzer.gf(pos, 'Y') * 1000;
                            vicon_format_markers{i}.Markers_(j).Translation.Z = Bag_Analyzer.gf(pos, 'Z') * 1000;
                        end
                    end

                    [msg_data, obj.marker_dictionary] = marker_management(vicon_format_markers, "skip_unknown", true, ...
                                                                        "replace_style", 'delete', "preserve_order", true);

                case 'candle_ros2/MotionCommand'
                    % Field names per candle_ros2/msg/MotionCommand.msg
                    % (docs/ros2_custom_messages.md in the superproject) --
                    % VERIFIED via ros2genmsg: ros2message(
                    % 'candle_ros2/MotionCommand') exposes drive_ids,
                    % target_position, target_velocity, target_torque,
                    % exactly as accessed here.
                    % ROS2-only message, no ROS1 PascalCase equivalent, so
                    % fields are accessed directly by their .msg snake_case
                    % spelling rather than through gf().
                    %
                    % drive_ids (uint32[]), target_position/target_velocity/
                    % target_torque (float32[]), all length n_drives --
                    % stacked vertically per message (JointState pattern),
                    % preallocated from the first message's n_drives, guarded
                    % with min() against a later message reporting a
                    % different n_drives (PointCloud pattern) rather than
                    % erroring.
                    n_drives = length(msg_cell{1}.drive_ids);
                    msg_data = zeros(4 * n_drives, num_msgs);

                    for i = 1:num_msgs
                        drive_ids       = double(msg_cell{i}.drive_ids(:));
                        target_position = double(msg_cell{i}.target_position(:));
                        target_velocity = double(msg_cell{i}.target_velocity(:));
                        target_torque   = double(msg_cell{i}.target_torque(:));
                        col = [drive_ids; target_position; target_velocity; target_torque];

                        % Guard against a varying number of drives per message
                        n = min(length(col), size(msg_data, 1));
                        msg_data(1:n, i) = col(1:n);
                    end

                case 'mocap_optitrack_interfaces/RigidBodyArray'
                    % Field names per mocap_optitrack_interfaces/msg/
                    % RigidBodyArray.msg + RigidBody.msg (docs/
                    % ros2_custom_messages.md in the superproject):
                    % top-level 'rigid_bodies' and per-body 'id'/'valid'/
                    % 'mean_error'/'pose_stamped' are ROS2-only, accessed
                    % directly by their .msg snake_case spelling -- VERIFIED
                    % via ros2genmsg: ros2message(
                    % 'mocap_optitrack_interfaces/RigidBody') exposes
                    % header/id/valid/mean_error/pose_stamped, as accessed here.
                    % This is rigid-body POSE data (numeric), not markers --
                    % same shape as the RigidBodyPose half of the
                    % dynamic_manipulation_dlo case above, not the marker
                    % half. pose_stamped is a full geometry_msgs/PoseStamped
                    % sub-struct, so its internal pose.position/.orientation
                    % nesting and .X/.Y/.Z/.W leaves ARE the already-verified
                    % single-word ROS2 convention (same pattern as
                    % Bag_Analyzer.decodePoseStamped) and go through gf().
                    % Quaternion order honors obj.quaternion_order exactly
                    % like the geometry_msgs/PoseStamped / dynamic_manipulation
                    % cases.
                    %
                    % 10 rows per body: [id; valid(0/1); mean_error;
                    % pos.x; pos.y; pos.z; quat(4)], preallocated from the
                    % first message's rigid-body count, guarded with min()
                    % against a later message reporting a different count
                    % (PointCloud pattern).
                    n_bodies = length(msg_cell{1}.rigid_bodies);
                    msg_data = zeros(10 * n_bodies, num_msgs);

                    for i = 1:num_msgs
                        bodies = msg_cell{i}.rigid_bodies;
                        n_bodies_i = length(bodies);
                        col = zeros(10 * n_bodies_i, 1);

                        for j = 1:n_bodies_i
                            b    = bodies(j);
                            pose = Bag_Analyzer.gf(b.pose_stamped, 'Pose');
                            pos  = Bag_Analyzer.gf(pose, 'Position');
                            ori  = Bag_Analyzer.gf(pose, 'Orientation');

                            if obj.quaternion_order == "wxyz"
                                quat = [Bag_Analyzer.gf(ori,'W'); Bag_Analyzer.gf(ori,'X'); Bag_Analyzer.gf(ori,'Y'); Bag_Analyzer.gf(ori,'Z')];
                            else % "xyzw"
                                quat = [Bag_Analyzer.gf(ori,'X'); Bag_Analyzer.gf(ori,'Y'); Bag_Analyzer.gf(ori,'Z'); Bag_Analyzer.gf(ori,'W')];
                            end

                            idx = (j-1)*10 + 1;
                            col(idx:idx+9) = [double(b.id); double(b.valid); double(b.mean_error); ...
                                               Bag_Analyzer.gf(pos,'X'); Bag_Analyzer.gf(pos,'Y'); Bag_Analyzer.gf(pos,'Z'); ...
                                               quat];
                        end

                        % Guard against a varying number of rigid bodies per message
                        n_rows = min(length(col), size(msg_data, 1));
                        msg_data(1:n_rows, i) = col(1:n_rows);
                    end

                otherwise
                    msg_data = msg_cell;
            end
        end

        % Extract Topics & Msgs
        function extractMsgs(obj)
            for i = 1:obj.n_topics
                % --- DISPATCH POINT 4: topic selection + message read -------
                % select(...,'Topic',...) syntax is identical for both
                % readers (VERIFIED, probe_ros2_assumptions.m).
                % readMessages is NOT identical: CONFIRMED BUG (not just an
                % assumption) -- ros2bagreader/readMessages has no
                % 'DataFormat' parameter at all (its signature is only
                % readMessages(bag) / readMessages(bag,rows)) and throws
                % "Too many input arguments" if passed one. It always
                % returns a cell array of structs by default, which is the
                % struct format ROS1 needs 'DataFormat','struct' to opt
                % into. Dispatch is required here.
                topic_cell = select(obj.bag_obj, 'Topic', obj.topic_names{i});
                % msgs
                switch obj.bag_format
                    case "ros1"
                        msg_cell = readMessages(topic_cell,'DataFormat','struct');
                    case "ros2"
                        msg_cell = readMessages(topic_cell);
                end

                % Guard against empty topics
                if isempty(msg_cell)
                    obj.msg_type{i} = '';
                    obj.n_msgs(i) = 0;
                    obj.topics_ts{i} = struct('Time', [], 'Data', []);
                    continue;
                end

                % CONFIRMED against a real ROS2 bag (probe_real_bag_e2e.m):
                % for a message type MATLAB doesn't recognize (a custom ROS2
                % message with no ros2genmsg-generated definition on the
                % path, e.g. candle_ros2/TrackedMarkerArray,
                % mocap_optitrack_interfaces/MarkerArray), readMessages does
                % NOT throw -- it emits a "not a recognized custom message"
                % warning and returns a non-empty cell whose elements are
                % empty doubles instead of structs. msg_cell{1}.MessageType
                % below would then crash with "Dot indexing is not
                % supported for variables of this type", killing the WHOLE
                % bag analysis on one unsupported topic. Skip just that
                % topic instead.
                if ~isstruct(msg_cell{1})
                    warning('Bag_Analyzer:UnrecognizedMessageType', ...
                        ['Topic "%s": MATLAB could not decode this message type ' ...
                         '(likely a custom message needing ros2genmsg). Skipping this topic.'], ...
                        obj.topic_names{i});
                    obj.msg_type{i} = '';
                    obj.n_msgs(i) = 0;
                    obj.topics_ts{i} = struct('Time', [], 'Data', []);
                    continue;
                end

                % --- DISPATCH POINT 5: message timestamps -------------------
                % VERIFIED (probe_ros2_assumptions.m): sel.MessageList.Time
                % is a plain double column of seconds on the same basis as
                % the reader's StartTime for both ROS1 and ROS2, so this
                % subtraction is format-agnostic.
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
            % struct.
            %
            % Field naming is NOT actually identical between ROS1 and ROS2
            % (contrary to what this comment used to claim): ROS1 uses
            % Pose.Position.X, ROS2 uses pose.position.x. Bag_Analyzer.gf()
            % handles both -- VERIFIED via a real ROS2 bag's
            % visualization_msgs/Marker.pose.position/.orientation nested
            % struct (see inspect_real_ros2_bag.m), which follows the same
            % pattern as PoseStamped's Pose field.
            %
            % msg_cell        : cell array of PoseStamped structs
            % quaternion_order: "wxyz" or "xyzw"
            num_msgs = length(msg_cell);
            msg_data = zeros(7, num_msgs); % 3 for pos + 4 for quat

            for i = 1:num_msgs
                pose = Bag_Analyzer.gf(msg_cell{i}, 'Pose');
                pos  = Bag_Analyzer.gf(pose, 'Position');
                ori  = Bag_Analyzer.gf(pose, 'Orientation');
                if quaternion_order == "wxyz"
                    msg_data(:, i) = [Bag_Analyzer.gf(pos,'X'); Bag_Analyzer.gf(pos,'Y'); Bag_Analyzer.gf(pos,'Z');
                                    Bag_Analyzer.gf(ori,'W'); Bag_Analyzer.gf(ori,'X'); Bag_Analyzer.gf(ori,'Y'); Bag_Analyzer.gf(ori,'Z')];
                elseif quaternion_order == "xyzw"
                    msg_data(:, i) = [Bag_Analyzer.gf(pos,'X'); Bag_Analyzer.gf(pos,'Y'); Bag_Analyzer.gf(pos,'Z');
                                    Bag_Analyzer.gf(ori,'X'); Bag_Analyzer.gf(ori,'Y'); Bag_Analyzer.gf(ori,'Z'); Bag_Analyzer.gf(ori,'W')];
                end
            end
        end
    end

    methods (Static)
        function v = gf(s, ros1_name)
            % GF Dual-case struct field accessor: ROS1 PascalCase vs ROS2's
            % lower-first-letter convention for the same single-word field.
            %
            % v = Bag_Analyzer.gf(s, 'Position') returns s.Position if that
            % field exists (ROS1 struct), otherwise s.position (ROS2
            % struct, first letter lowercased -- NOT full snake_case, which
            % does not apply to the single-word fields this is used for:
            % Position, Orientation, Translation, Rotation, Force, Torque,
            % Data, Point, Points, Transform, Wrench, X, Y, Z, W, ...).
            %
            % VERIFIED against a real ROS2 bag for Data (std_msgs/
            % Float64MultiArray) and Position/Velocity/Effort (sensor_msgs/
            % JointState); the rest follow the same rule per the public
            % ROS2 message interface definitions (see extractData's header
            % comment for exactly which). Do NOT use this for multi-word
            % custom-message fields (e.g. MarkerIds) without separately
            % confirming they don't instead use full snake_case.
            if isfield(s, ros1_name)
                v = s.(ros1_name);
                return
            end
            ros2_name = [lower(ros1_name(1)), ros1_name(2:end)];
            if isfield(s, ros2_name)
                v = s.(ros2_name);
                return
            end
            error('Bag_Analyzer:FieldNotFound', ...
                'Neither field "%s" nor "%s" found on this message struct (fields: %s).', ...
                ros1_name, ros2_name, strjoin(fieldnames(s), ', '));
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