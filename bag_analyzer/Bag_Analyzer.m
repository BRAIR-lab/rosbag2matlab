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
        neff_topics
        topic_cell
        msg_cell

        % Time & Msgs
        topic_time
        topic_data
        msg_type
        n_msgs
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
            obj.topic_names = obj.bag_obj.AvailableTopics.Row;
            obj.n_topics = length(obj.topic_names);

            % Extract Topics & Msgs
            obj.extractMsgs();

            % Extract Data
            obj.ros2matlab();
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

        function initialized_ds = init_ds(obj, topic_id)
            % Different Operation w.r.t. type of msg
            switch obj.msg_type{topic_id}
                case 'sensor_msgs/Image'
                    initialized_ds = NaN*ones(size(obj.topic_data{topic_id}{1}));

                case 'std_msgs/Float32MultiArray'
                    initialized_ds = NaN*ones(length(obj.topic_data{topic_id}{1}), 1);
                   
                case 'std_msgs/Float64MultiArray'
                    initialized_ds = NaN*ones(length(obj.topic_data{topic_id}{1}), 1);
                
                case 'geometry_msgs/TransformStamped'
                    % Translation (Cartesian)
                    initialized_ds.Translation    = NaN*ones(3, 1);
    
                    % Rotation (Quaternion)
                    initialized_ds.Rotation       = NaN*ones(4, 1);

                case 'vicon_bridge/Markers'
                    n_markers = length(obj.topic_data{topic_id}{1});

                    % Fill vector
                    initialized_ds = NaN*ones(n_markers, 1);
               
                otherwise
                    disp("Unsupported msg type.")
            end
        end

        % Useful function from ros2cell
        function new_data = ros2cell(obj, data, msg_type)
            % new_data = [];

            % Different Operation w.r.t. type of msg
            switch msg_type
                case 'sensor_msgs/Image'
                    new_data = rosReadImage(data);
    
                case 'std_msgs/Float32MultiArray'
                    new_data = data.Data;
                   
                case 'std_msgs/Float64MultiArray'
                    new_data = data.Data;
                
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
                case 'vicon_bridge/Markers'
                    n_markers = length(data.Markers_);

                    % Fill vector
                    new_data = [];
                    for i = 1:n_markers
                        new_data = [new_data; data.Markers_(i).Translation.X; data.Markers_(i).Translation.Y; data.Markers_(i).Translation.Z];
                    end
               
                otherwise
                    disp("Unsupported msg type.")
            end
        end

        % Extract Data
        function ros2matlab(obj)
            for i = 1:obj.n_topics
                for j = 1:obj.n_msgs(i)
                    % Assign Data
                    obj.topic_data{i}{j} = obj.ros2cell(obj.msg_cell{i}{j}, obj.msg_type{i});
                end
            end

            % Init neff_topics
            obj.neff_topics = length(obj.topic_data);
        end

        % Merge Dataset
        function merged_dataset = merge_dataset(obj)
            %% Merge Time
            % Concatenate and Sort
            total_time = [];
            obj.neff_topics = length(obj.topic_data);
        
            % Append
            for i = 1:obj.neff_topics
                total_time = [total_time; obj.topic_time{i}];
            end

            % Sort and Merge
            [merged_time, idxs] = sort(total_time);
            merged_dataset.time = merged_time;
            
            %% Merge Dataset
            % Init Merged Dataset
            merged_dataset.data = cell(1, obj.neff_topics);

            for(i = 1:obj.neff_topics)
                merged_dataset.data{i}{1} = obj.init_ds(i);
            end

            %% Fill New Dataset
            % Find Bounds
            idx_bounds = [0];
            for k = 1:obj.n_topics
                idx_bounds = [idx_bounds, sum(obj.n_msgs(1:k))];
            end

            % Update new data & ZOH
            % Cycle for all time samples
            for i = 1:length(merged_dataset.time)
                % for all topics
                for j = 1:(obj.n_topics)
                    % if the sample belongs to the j-th topic
                    if( (idxs(i) > idx_bounds(j)) && (idxs(i) <= idx_bounds(j + 1)) )
                        % Update New Data
                        merged_dataset.data{j}{i} = obj.topic_data{j}{idxs(i) - idx_bounds(j)};

                        % Assign to the other topics the previous value
                        % (ZOH)
                        for k = 1:obj.n_topics
                            if((k ~= j) && (i ~= 1))
                                % Double check on the index.
                                % This is because is already initialized
                                merged_dataset.data{k}{i} = merged_dataset.data{k}{i - 1};
                            end
                        end
                    
                        % Continue to the next topic
                        continue;
                    end
                end
            end
        end
    end
end 