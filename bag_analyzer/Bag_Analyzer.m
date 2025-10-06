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
    end

    %% Methods
    methods
        % Constructor
        function obj = Bag_Analyzer(bag_name, options)
            arguments
                bag_name
                options.quaternion_order = "wxyz";
            end
            % Init Bag Object
            obj.bag_obj = rosbag(bag_name);

            % Quaternion Order
            obj.quaternion_order = options.quaternion_order;

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

            switch msg_cell{1}.MessageType
                case 'sensor_msgs/Image'
                    % Extract Image
                    for i = 1:num_msgs
                        msg_data(:, :, :, i) = rosReadImage(msg_cell{i});
                    end
                 
                case 'sensor_msgs/CompressedImage'
                    % Extract Image
                    for i = 1:num_msgs
                        msg_data(:, :, :, i) = rosReadImage(msg_cell{i});
                    end
                 
                case 'std_msgs/Float32MultiArray'
                    % msg_data = msg_cell;
                    for i = 1:num_msgs
                        msg = double(msg_cell{i}.Data);

                        if(isempty(msg))
                            msg_data(:, i) = NaN;
                        else
                            msg_data(:, i) = msg;
                        end
                    end

                case 'std_msgs/Float64MultiArray'
                    % msg_data = msg_cell;
                    for i = 1:num_msgs
                        msg = double(msg_cell{i}.Data);

                        if(isempty(msg))
                            msg_data(:, i) = NaN;
                        else
                            msg_data(:, i) = msg;
                        end
                    end

                case 'geometry_msgs/PoseStamped'
                    for i = 1:num_msgs
                        % Save RigidBodyPose
                        if obj.quaternion_order == "wxyz"
                            msg_data(:, i) = [msg_cell{i}.Pose.Position.X; msg_cell{i}.Pose.Position.Y; msg_cell{i}.Pose.Position.Z;
                                            msg_cell{i}.Pose.Orientation.W; msg_cell{i}.Pose.Orientation.X; msg_cell{i}.Pose.Orientation.Y; msg_cell{i}.Pose.Orientation.Z];
                        elseif obj.quaternion_order == "xyzw"
                            msg_data(:, i) = [msg_cell{i}.Pose.Position.X; msg_cell{i}.Pose.Position.Y; msg_cell{i}.Pose.Position.Z;
                                            msg_cell{i}.Pose.Orientation.X; msg_cell{i}.Pose.Orientation.Y; msg_cell{i}.Pose.Orientation.Z; msg_cell{i}.Pose.Orientation.W];
                        end
                    end
                
                case 'geometry_msgs/TransformStamped'
                    for i = 1:num_msgs
                        se3_data = msg_cell{i}.Transform;
                        msg_data(:, i) = [se3_data.Translation.X; se3_data.Translation.Y; se3_data.Translation.Z;
                                        se3_data.Rotation.W; se3_data.Rotation.X; se3_data.Rotation.Y; se3_data.Rotation.Z];
                    end

                case 'geometry_msgs/PointStamped'
                    % msg_data = msg_cell;
                    for i = 1:num_msgs
                        msg_data(:, i) = [msg_cell{i}.Point.X; msg_cell{i}.Point.Y; msg_cell{i}.Point.Z];
                    end

                case 'geometry_msgs/WrenchStamped'
                    % msg_data = msg_cell;
                    for i = 1:num_msgs
                        msg_data(:, i) = [msg_cell{i}.Wrench.Force.X; msg_cell{i}.Wrench.Force.Y; msg_cell{i}.Wrench.Force.Z;
                                          msg_cell{i}.Wrench.Torque.X; msg_cell{i}.Wrench.Torque.Y; msg_cell{i}.Wrench.Torque.Z];
                    end

                 case 'vicon_bridge/Markers'
                     [~, obj.marker_dictionary] = marker_management(msg_cell, "skip_unknown", true);

                case 'sensor_msgs/PointCloud'
                    % Read only the first instant the number of markers.
                    for i = 1:num_msgs
                        n_points = length(msg_cell{i}.Points);
                        % Fill vector
                        points_pos = [];
                        for j = 1:n_points
                            single_point = double([msg_cell{i}.Points(j).X; msg_cell{i}.Points(j).Y; msg_cell{i}.Points(j).Z]);
                            points_pos = [points_pos; single_point];
                        end
                        msg_data(:, i) = points_pos;   % [m]
                    end

                case 'sensor_msgs/PointCloud2'
                    % Read only the first instant the number of markers.
                    for i = 1:num_msgs
                        points = double(rosReadXYZ(msg_cell{i})');
                        points = reshape(points, size(points, 1)*size(points, 2), 1);
                        msg_data(:, i) = points;   % [m]
                    end

                case 'dynamic_manipulation_dlo/MarkerRigidBodyPoses'
                    % msg_data = msg_cell;
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
                            % Reuse marker_management converting in VICON
                            % Format
                            n_markers = length(msg_cell{i}.MarkerIds);
                            
                            % for each marker
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

                    % Marker Management
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

        % Synchronization
        function [merged_time, merged_dataset, sync_marker_dic, topics] = synchronization(obj, resampling_period, mask, options)
            arguments
                obj
                resampling_period = 1.0e+2;
                mask = true*ones(1, obj.n_topics);
                options.interpolation_method = 'linear';
            end

            % Merged Time Definition
            merged_time = 0:resampling_period:obj.bag_duration;

            % Interpolate
            for i = 1:obj.n_topics
                % Temporarirly skip for unsupported msg types
                if (length(obj.topics_ts{i}) ~= 1) || (~mask(i))
                    continue;
                end

                % Merge Dataset
                if (obj.msg_type{i} == "sensor_msgs/Image") || (obj.msg_type{i} == "sensor_msgs/CompressedImage")
                    % Due to the unsupport of interp1() for 4d matrix,
                    % I tried this alternative solution:
                    % Just reordering the samples index and after assign
                    % it. Of course, we have to manage the NaN case.
                    fake_data = (1:length(obj.topics_ts{i}.Time))';
                    
                    % hardcoded previous method for images
                    fake_dataset = interp1(obj.topics_ts{i}.Time', fake_data, ...
                                                    merged_time', 'previous');
                    for j = 1:length(fake_dataset)
                        if isnan(fake_dataset(j))
                            % Fill Image with NaN
                            merged_dataset{i}(:, :, :, j) = uint8(NaN*ones(size(obj.topics_ts{i}.Data(:, :, :, 1))));
                        else
                            merged_dataset{i}(:, :, :, j) = obj.topics_ts{i}.Data(:, :, :, fake_dataset(j));
                        end
                    end

                    clear fake_data fake_dataset
                else
                    merged_dataset{i} = interp1(obj.topics_ts{i}.Time', obj.topics_ts{i}.Data', ...
                                                    merged_time', options.method);

                    % Transposing, I like more column notation
                    merged_dataset{i} = merged_dataset{i}';
                end

                % Store topic names
                topics{i} = obj.topic_names{i};
            end

            %% Synchronize Marker Dictionary
            vicon_idx = find(strcmp(obj.msg_type, "vicon_bridge/Markers"), 1, 'last');
            optitrack_idx = find(strcmp(obj.msg_type, "dynamic_manipulation_dlo/MarkerRigidBodyPoses"), 1, 'last');

            if(~isempty(vicon_idx) || ~isempty(optitrack_idx))
                % Find idx
                idx = [vicon_idx, optitrack_idx];

                % Synchronize markers' Dictionary
                marker_names = string(fieldnames(obj.marker_dictionary));

                for j = 1:length(marker_names)
                    obj.marker_dictionary.(marker_names(j)) = interp1(obj.topics_ts{idx}.Time', obj.marker_dictionary.(marker_names(j))', merged_time', method)';
                end
            end
        
            % Remove Skipped Topics
            merged_dataset = merged_dataset(~cellfun('isempty', merged_dataset));

            % Updated Marker Dictionary
            sync_marker_dic = obj.marker_dictionary;
        end
    end
end