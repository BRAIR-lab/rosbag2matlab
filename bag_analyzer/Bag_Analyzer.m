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
        topic_cell
        msg_cell

        % Time & Msgs
        topic_time
        topic_data
        msg_type
        n_msgs

        %% Timeseries
        topic_ts
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

            % Extract Data
            obj.ros2matlab();

            % Get Timeseries
            obj.ros2ts();
        end

        % Extract Topics & Msgs
        function extractMsgs(obj)
            for i = 1:obj.n_topics
                % Topic
                obj.topic_cell{i} = select(obj.bag_obj, 'Topic', obj.topic_names{i});
                % msgs
                obj.msg_cell{i} = readMessages(obj.topic_cell{i},'DataFormat','struct');
                % Time
                obj.topic_time{i} = obj.topic_cell{i}.MessageList.Time - obj.start_time;
                % Type
                obj.msg_type{i} = obj.msg_cell{i}{1}.MessageType;
                % N° of Msgs
                obj.n_msgs(i) = length(obj.msg_cell{i});
            end
        end

        % Useful function from ros2cell
        function [new_data, skip_flag] = ros2cell(obj, data, msg_type)

            % Different Operation w.r.t. type of msg
            switch msg_type
                case 'sensor_msgs/Image'
                    new_data = rosReadImage(data);
                    skip_flag = false;
    
                case 'std_msgs/Float32MultiArray'
                    new_data = data.Data;
                    skip_flag = false;
                   
                case 'std_msgs/Float64MultiArray'
                    new_data = data.Data;
                    skip_flag = false;
                
                case 'geometry_msgs/TransformStamped'
                    % Translation (Cartesian)
                    new_data.Translation    =       [data.Transform.Translation.X; 
                                                    data.Transform.Translation.Y;
                                                    data.Transform.Translation.Z];
    
                    % Rotation (Quaternion)
                    new_data.Rotation       =       [data.Transform.Rotation.W;
                                                    data.Transform.Rotation.X; 
                                                    data.Transform.Rotation.Y;
                                                    data.Transform.Rotation.Z];
                   
                    skip_flag = false;
                case 'vicon_bridge/Markers'
                    n_markers = length(data.Markers_);

                    % Fill vector
                    new_data = [];
                    for i = 1:n_markers
                        new_data = [new_data; data.Markers_(i).Translation.X; data.Markers_(i).Translation.Y; data.Markers_(i).Translation.Z];
                    end
                    skip_flag = false;
               
                otherwise
                    new_data = [];
                    % disp("Unsupported msg type.")
                    skip_flag = true;
            end
        end

        % Extract Data
        function ros2matlab(obj)
            for i = 1:obj.n_topics
                for j = 1:obj.n_msgs(i)
                    % Assign Data
                    [obj.topic_data{i}{j}, skip_flag] = obj.ros2cell(obj.msg_cell{i}{j}, obj.msg_type{i});
                    
                    % % Skip to avoid useless iterations
                    % if skip_flag
                    %     break;
                    % end
                end
            end
        end

        % Convert in Time Series
        function ros2ts(obj)
            for i = 1:obj.n_topics
                switch obj.msg_type{i}
                    case 'sensor_msgs/Image'
                        % Convert in image matrix (4d array)
                        for j = 1:length(obj.topic_data{i})
                            imageMatrix(:, :, :, j) = obj.topic_data{i}{j};
                        end
                        % Time series object
                        obj.topic_ts{i} = timeseries(imageMatrix, obj.topic_time{i}, ...
                                                        "Name", obj.topic_names{i});

                    case 'std_msgs/Float32MultiArray'
                        obj.topic_ts{i} = timeseries(double(cell2mat(obj.topic_data{i})'), obj.topic_time{i}, ...
                                                        "Name", obj.topic_names{i});
        
                    case 'std_msgs/Float64MultiArray'
                        obj.topic_ts{i} = timeseries(double(cell2mat(obj.topic_data{i})'), obj.topic_time{i}, ...
                                                        "Name", obj.topic_names{i});

                    case 'geometry_msgs/TransformStamped'
                        obj.topic_ts{i} = timeseries(cell2mat(struct2cell(cell2mat(obj.topic_data{i})'))', obj.topic_time{i}, ...
                                                        "Name", obj.topic_names{i});
                    
                    case 'vicon_bridge/Markers'
                        obj.topic_ts{i} = timeseries(cell2mat(obj.topic_data{i})', obj.topic_time{i}, ...
                                                        "Name", obj.topic_names{i});
                    
                    otherwise
                        obj.topic_ts{i} = timeseries(NaN*ones(size(obj.topic_time{i})), obj.topic_time{i});
                end
            end
        end
        
        % Synchronize Topics
        function synchronization(obj, resampling_period)
            for i = 1:obj.n_topics
                for j = 1:obj.n_topics
                    if i ~= j
                        [obj.topic_ts{i}, obj.topic_ts{j}] = synchronize(obj.topic_ts{i}, obj.topic_ts{j}, ...
                                                                        'uniform', 'Interval', resampling_period, ...
                                                                        'KeepOriginalTimes',true, ...
                                                                        'InterpMethod', 'zoh');
                        
                    end
                end
            end
        end
    
    end
end 