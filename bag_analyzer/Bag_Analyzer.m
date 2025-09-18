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
    end

    %% Methods
    methods
        % Constructor
        function obj = Bag_Analyzer(bag_name)
            % Init Bag Object
            obj.bag_obj = rosbag(bag_name);

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

                % case 'tf2_msgs/TFMessage'
                %     % msg_data = msg_cell;
                %     for i = 1:num_msgs
                %         % msg_data(:, i) = [msg_cell{i}.Point.X; msg_cell{i}.Point.Y; msg_cell{i}.Point.Z];
                %         if(isscalar(msg_cell{i}.Transforms))
                %             transform_obj =  msg_cell{i}.Transforms.Transform;
                %             msg_data(:, i) = [transform_obj.Translation.X; 
                %                                 transform_obj.Translation.Y;
                %                                 transform_obj.Translation.Z;
                %                                 transform_obj.Rotation.W;
                %                                 transform_obj.Rotation.X;
                %                                 transform_obj.Rotation.Y;
                %                                 transform_obj.Rotation.Z];
                %         end
                %     end

                 case 'vicon_bridge/Markers'
                     [msg_data, obj.marker_dictionary] = marker_management(msg_cell, "skip_unknown", true);

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

                % case 'tf'
                %     msg_data = msg_cell;

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
        function [merged_time, merged_dataset, topics] = synchronization(obj, resampling_period, mask)
            method = 'previous'; % hardcoded for the image

            if nargin <= 2
                mask = true*ones(1, obj.n_topics);
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
                    fake_dataset = interp1(obj.topics_ts{i}.Time', fake_data, ...
                                                    merged_time', method);
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
                                                    merged_time', method);

                    % Transposing, I like more column notation
                    merged_dataset{i} = merged_dataset{i}';
                end

                % Store topic names
                topics{i} = obj.topic_names{i};
            end
        
            % Remove Skipped Topics
            merged_dataset = merged_dataset(~cellfun('isempty', merged_dataset));
        end
    
    end
end 