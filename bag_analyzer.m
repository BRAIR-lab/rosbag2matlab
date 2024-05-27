%%% bag analyzer %%%
clear all 
close all
clc

%% Load Bagfile
bagdir_name = "bags";
bagname = "formazione";
bag_ext = ".bag";

% Add bag dir
addpath(bagdir_name);

% fullfile makes compatible with windows or ubuntu paths
% Ubuntu "/" | Windows "\"
bag = rosbag(fullfile(bagdir_name, bagname + bag_ext));

%% Time Information
start_time = bag.StartTime;
end_time = bag.EndTime;
bag_duration = end_time - start_time;

%% Define useful topic names
topic_names = bag.AvailableTopics.Row;
n_topics = length(topic_names);

%% Extract Topics & Msgs
for i = 1:n_topics
    % Topic
    topic_cell{i} = select(bag, 'Topic', topic_names{i});
    % msgs
    msg_cell{i} = readMessages(topic_cell{i},'DataFormat','struct');
    % Time
    topic_time{i} = topic_cell{i}.MessageList.Time - start_time;
end

%% Convert in MATLAB types
for i = 1:n_topics
    % Find msg type
    msg_type = msg_cell{i}{1}.MessageType;
    n_msgs = length(msg_cell{i});

    for j = 1:n_msgs
        switch msg_type
            case 'sensor_msgs/Image'
                topic_data{i}{j} = rosReadImage(msg_cell{i}{j});

            case 'std_msgs/Float32MultiArray'
                topic_data{i}(:, j) = msg_cell{i}{j}.Data;
               
            case 'std_msgs/Float64MultiArray'
                topic_data{i}(:, j) = msg_cell{i}{j}.Data;
            
            case 'geometry_msgs/TransformStamped'
                % Translation (cartesian)
                topic_data{i}.Translation(1, j) = msg_cell{i}{j}.Transform.Translation.X;
                topic_data{i}.Translation(2, j) = msg_cell{i}{j}.Transform.Translation.Y;
                topic_data{i}.Translation(3, j) = msg_cell{i}{j}.Transform.Translation.Z;

                % Rotation (quaternion)
                topic_data{i}.Rotation(1, j) = msg_cell{i}{j}.Transform.Rotation.W;
                topic_data{i}.Rotation(2, j) = msg_cell{i}{j}.Transform.Rotation.X;
                topic_data{i}.Rotation(3, j) = msg_cell{i}{j}.Transform.Rotation.Y;
                topic_data{i}.Rotation(4, j) = msg_cell{i}{j}.Transform.Rotation.Z;
           
            otherwise
                % expty cell array
                topic_data{i} = {};
                disp("Unsupported msg type.")
        end
    end        

end

%% Merge Dataset
for i = 1:(n_topics - 1)
    ds_cell{i}.data = topic_data{i};
    ds_cell{i}.time = topic_time{i};
    ds_cell{i}.type = msg_cell{i}{1}.MessageType;
end

merged_dataset = merge_dataset(ds_cell);

%% Functions
function merged_dataset = merge_dataset(ds_cell)    
    %% Merge Time
    % Concatenate and Sort
    total_time = [];
    n_topics = length(ds_cell);

    for i = 1:n_topics
        n_msgs(i) = length(ds_cell{i}.time);
        total_time = [total_time; ds_cell{i}.time];
    end
    [merged_time, idxs] = sort(total_time);
    merged_dataset.time = merged_time;
    
    %% Merge Dataset
    % Init Merged Dataset
    merged_dataset.data = cell(1, n_topics);
    
    % Init
    for i = 1:n_topics
        switch ds_cell{i}.type
            case 'sensor_msgs/Image'
                merged_dataset.data{i}{1} = NaN*ones(size(ds_cell{i}.data{1}));

            case 'std_msgs/Float32MultiArray'
                merged_dataset.data{i}(:, 1) =  NaN*ones(length(ds_cell{i}.data(:, 1)), 1);
               
            case 'std_msgs/Float64MultiArray'
                merged_dataset.data{i}(:, 1) =  NaN*ones(length(ds_cell{i}.data(:, 1)), 1);
            
            case 'geometry_msgs/TransformStamped'
                merged_dataset.data{i}.Translation(:, 1) = NaN*ones(3, 1);
                merged_dataset.data{i}.Rotation(:, 1) = NaN*ones(4, 1);
        end
    end

    %% Fill New Dataset
    % % Cycle on all time samples
    % for i = 1:length(merged_dataset.time)
    %     for j = 1:(n_topics)
    % 
    % 
    %     end
    % end
end




