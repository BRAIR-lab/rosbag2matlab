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

            % Extract Topics & Msgs
            obj.extractMsgs();
        end

        function msg_data = extractData(obj, msg_cell)
            
            switch msg_cell{1}.MessageType
                case 'sensor_msgs/Image'
                    % Extract Image
                    for i = 1:length(msg_cell)
                        msg_data(:, :, :, i) = rosReadImage(msg_cell{i});
                    end
                 
                case 'std_msgs/Float32MultiArray'
                    % msg_data = msg_cell;
                    for i = 1:length(msg_cell)
                        msg_data(:, i) = double(msg_cell{i}.Data);
                    end

                case 'std_msgs/Float64MultiArray'
                    % msg_data = msg_cell;
                    for i = 1:length(msg_cell)
                        msg_data(:, i) = double(msg_cell{i}.Data);
                    end
                
                case 'geometry_msgs/TransformStamped'
                    for i = 1:length(msg_cell)
                        se3_data = msg_cell{i}.Transform;
                        msg_data(:, i) = [se3_data.Translation.X; se3_data.Translation.Y; se3_data.Translation.Z;
                                        se3_data.Rotation.W; se3_data.Rotation.X; se3_data.Rotation.Y; se3_data.Rotation.Z];
                    end

                 case 'vicon_bridge/Markers'
                     % msg_data = msg_cell;
                    % for i = 1:length(msg_cell)
                    %     se3_data = msg_cell{i}.Transform;
                    %     msg_data(:, i) = [se3_data.Translation.X; se3_data.Translation.Y; se3_data.Translation.Z;
                    %                     se3_data.Rotation.W; se3_data.Rotation.X; se3_data.Rotation.Y; se3_data.Rotation.Z];
                    for i = 1:length(msg_cell)
                        n_markers = length(msg_cell{i}.Markers_);
                        % Fill vector
                        markers_pos = [];
                        for j = 1:n_markers
                            markers_pos = [markers_pos; msg_cell{i}.Markers_(j).Translation.X; msg_cell{i}.Markers_(j).Translation.Y; msg_cell{i}.Markers_(j).Translation.Z];
                        end
                        msg_data(:, i) = markers_pos./1000;
                    end
                    % end
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
        function [merged_time, merged_dataset] = synchronization(obj, resampling_period)
            method = 'previous'; % hardcoded for the image 

            % Merged Time Definition
            merged_time = 0:resampling_period:obj.bag_duration;

            % Interpolate
            for i = 1:obj.n_topics
                % Temporarirly skip for unsupported msg types
                if length(obj.topics_ts{i}) ~= 1
                    continue;
                end

                % Merge Dataset
                if obj.msg_type{i} == "sensor_msgs/Image"
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

            end
        end
    
    end
end 