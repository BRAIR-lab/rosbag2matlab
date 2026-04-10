%%%  Bag Analyzer %%%

classdef Bag_Analyzer < handle
    %% Attributes
    properties
        %% Bag Object
        bag_obj

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
            % Init Bag Object
            obj.bag_obj = rosbag(bag_name);

            % Quaternion Order
            obj.quaternion_order = options.quaternion_order;
            obj.use_parallel = options.use_parallel;

            % Time Information
            obj.start_time = obj.bag_obj.StartTime;
            obj.end_time = obj.bag_obj.EndTime;
            obj.bag_duration = obj.end_time - obj.start_time;

            % Topics Info
            obj.topic_names = obj.bag_obj.AvailableTopics.Row';
            obj.n_topics = length(obj.topic_names);

            % Init Marker Dictionary
            obj.marker_dictionary = dictionary([], []);

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
                    % Preallocate based on the first message (filling with NaN for safety on empty msgs)
                    first_msg = double(msg_cell{1}.Data);
                    msg_data = NaN(length(first_msg), num_msgs);
                    
                    for i = 1:num_msgs
                        msg = double(msg_cell{i}.Data);
                        if ~isempty(msg)
                            msg_data(:, i) = msg;
                        end
                    end

                case 'std_msgs/Float64MultiArray'
                    first_msg = double(msg_cell{1}.Data);
                    msg_data = NaN(length(first_msg), num_msgs);
                    
                    for i = 1:num_msgs
                        msg = double(msg_cell{i}.Data);
                        if ~isempty(msg)
                            msg_data(:, i) = msg;
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
                    msg_data = zeros(7, num_msgs); % 3 for pos + 4 for quat
                    
                    for i = 1:num_msgs
                        if obj.quaternion_order == "wxyz"
                            msg_data(:, i) = [msg_cell{i}.Pose.Position.X; msg_cell{i}.Pose.Position.Y; msg_cell{i}.Pose.Position.Z;
                                            msg_cell{i}.Pose.Orientation.W; msg_cell{i}.Pose.Orientation.X; msg_cell{i}.Pose.Orientation.Y; msg_cell{i}.Pose.Orientation.Z];
                        elseif obj.quaternion_order == "xyzw"
                            msg_data(:, i) = [msg_cell{i}.Pose.Position.X; msg_cell{i}.Pose.Position.Y; msg_cell{i}.Pose.Position.Z;
                                            msg_cell{i}.Pose.Orientation.X; msg_cell{i}.Pose.Orientation.Y; msg_cell{i}.Pose.Orientation.Z; msg_cell{i}.Pose.Orientation.W];
                        end
                    end
                
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
                        % Preallocate the column vector to avoid inner-loop reallocation
                        points_pos = zeros(length(pts) * 3, 1); 
                        
                        for j = 1:length(pts)
                            idx = (j-1)*3 + 1;
                            points_pos(idx:idx+2) = [pts(j).X; pts(j).Y; pts(j).Z];
                        end
                        
                        % Handle potential size mismatches safely if N points changes over time
                        if length(points_pos) == size(msg_data, 1)
                            msg_data(:, i) = points_pos;
                        else
                            msg_data(1:length(points_pos), i) = points_pos; 
                        end
                    end

                case 'sensor_msgs/PointCloud2'
                    % Determine data size efficiently using the first message
                    first_points = double(rosReadXYZ(msg_cell{1})');
                    msg_data = zeros(numel(first_points), num_msgs); 
                    
                    for i = 1:num_msgs
                        points = double(rosReadXYZ(msg_cell{i})');
                        msg_data(:, i) = points(:); % Using (:) is much faster than reshape()
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
                % Topic
                topic_cell = select(obj.bag_obj, 'Topic', obj.topic_names{i});
                % msgs
                msg_cell = readMessages(topic_cell,'DataFormat','struct');
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

        % Synchronization (Master Clock Approach)
        function [merged_time, merged_dataset, sync_marker_dic, topics] = synchronization(obj, resampling_period, mask, options)
            arguments
                obj
                resampling_period = 1.0e+2;
                mask = true * ones(1, obj.n_topics);
                options.interpolation_method = 'linear';
            end
            
            % 1. Determine the Master Clock (Look for Image Topic)
            image_idx = find(strcmp(obj.msg_type, "sensor_msgs/Image") | strcmp(obj.msg_type, "sensor_msgs/CompressedImage"), 1);
            
            if ~isempty(image_idx)
                % Use exact camera timestamps as the Master Clock (0 frame drops)
                merged_time = obj.topics_ts{image_idx}.Time';
                disp(['Using image topic (', obj.topic_names{image_idx}, ') as the Master Clock.']);
                
                % Calculate the native average FPS for the VideoWriter playback speed
                avg_fps = length(merged_time) / (merged_time(end) - merged_time(1));
            else
                % Fallback: Use standard resampling grid if no camera is present
                merged_time = 0:resampling_period:obj.bag_duration;
                disp('No image topic found. Using standard resampling grid.');
                avg_fps = 1 / resampling_period;
            end
            
            % Initialize outputs
            merged_dataset = cell(1, obj.n_topics);
            topics = cell(1, obj.n_topics);
            
            % 2. Process and Interpolate Each Topic
            for i = 1:obj.n_topics
                % Skip unsupported or masked topics
                if (length(obj.topics_ts{i}) ~= 1) || (~mask(i))
                    continue;
                end
                
                % Check if the current topic is an image
                if (obj.msg_type{i} == "sensor_msgs/Image") || (obj.msg_type{i} == "sensor_msgs/CompressedImage")
                    
                    % Create a safe filename for the video (replace slashes)
                    safe_topic_name = strrep(obj.topic_names{i}, '/', '_');
                    video_filename = sprintf('.%s_sync.mp4', safe_topic_name);
                    
                    % Setup VideoWriter
                    v = VideoWriter(video_filename, 'MPEG-4');
                    v.FrameRate = avg_fps;
                    open(v);
                    
                    disp(['Streaming master-clock synchronized video directly to ', video_filename, '...']);
                    
                    % Write native frames directly to disk
                    num_frames = length(obj.topics_ts{i}.Time);
                    for j = 1:num_frames
                        % Because merged_time IS this topic's time, we simply write them sequentially
                        writeVideo(v, obj.topics_ts{i}.Data(:, :, :, j));
                    end
                    close(v);
                    
                    % Store a lightweight reference string instead of a massive RAM array
                    merged_dataset{i} = sprintf('Master-sync video saved to disk: %s', video_filename);
                    
                else
                    % Continuous 1D Data: Interpolate to the Master Clock timestamps
                    merged_dataset{i} = interp1(obj.topics_ts{i}.Time', obj.topics_ts{i}.Data', ...
                                                    merged_time, options.interpolation_method);
                    % Transpose to maintain column notation
                    merged_dataset{i} = merged_dataset{i}';
                end
                
                % Store topic names
                topics{i} = obj.topic_names{i};
            end
            
            % 3. Synchronize Marker Dictionary
            vicon_idx = find(strcmp(obj.msg_type, "vicon_bridge/Markers"), 1, 'last');
            optitrack_idx = find(strcmp(obj.msg_type, "dynamic_manipulation_dlo/MarkerRigidBodyPoses"), 1, 'last');
            
            % Grab the first available marker index to safely use for timing reference
            marker_timing_idx = [];
            if ~isempty(vicon_idx)
                marker_timing_idx = vicon_idx;
            elseif ~isempty(optitrack_idx)
                marker_timing_idx = optitrack_idx;
            end
            
            if ~isempty(marker_timing_idx)
                marker_names = string(fieldnames(obj.marker_dictionary));
                for j = 1:length(marker_names)
                    % Interpolate markers to the new master clock
                    obj.marker_dictionary.(marker_names(j)) = interp1(obj.topics_ts{marker_timing_idx}.Time', ...
                        obj.marker_dictionary.(marker_names(j))', merged_time, options.interpolation_method)';
                end
            end
        
            % 4. Final Cleanup
            merged_dataset = merged_dataset(~cellfun('isempty', merged_dataset));
            topics = topics(~cellfun('isempty', topics)); % Clean up empty topics too
            
            % Updated Marker Dictionary
            sync_marker_dic = obj.marker_dictionary;
            
            disp('Synchronization complete! Check your working directory for the .mp4 file.');
        end
    end
end