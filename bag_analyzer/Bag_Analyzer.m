%%%  Bag Analyzer (Optimized) %%%
classdef Bag_Analyzer < handle
    %% Attributes
    properties
        % Bag Object
        bag_obj
        % Time Information
        start_time
        end_time
        bag_duration
        % Topics Information
        topic_names
        n_topics
        msg_type
        n_msgs
        % Timeseries
        topics_ts
        % Marker & VICON Utilities
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
            
            %% OPTIMIZATION: Pre-allocate cell arrays and vectors.
            % This prevents MATLAB from resizing them in the loop, which is slow.
            obj.msg_type = cell(1, obj.n_topics);
            obj.n_msgs = zeros(1, obj.n_topics);
            obj.topics_ts = cell(1, obj.n_topics);

            % Extract Topics & Msgs
            obj.extractMsgs();
        end

        function msg_data = extractData(obj, msg_cell)
            num_msgs = length(msg_cell);
            if num_msgs == 0
                msg_data = [];
                return;
            end

            %% OPTIMIZATION: Use a struct array for vectorized access.
            % This conversion is fast and lets us access fields from all
            % messages at once (e.g., [s.Point.X]) instead of looping.
            msg_structs = [msg_cell{:}];
            
            switch msg_structs(1).MessageType
                %% OPTIMIZATION: Combine identical cases.
                case {'sensor_msgs/Image', 'sensor_msgs/CompressedImage'}
                    %% OPTIMIZATION: Pre-allocate the 4D image matrix.
                    % Reading the first image to get dimensions avoids resizing the
                    % matrix on every iteration, which is a major performance boost.
                    first_img = rosReadImage(msg_structs(1));
                    [h, w, c] = size(first_img);
                    % Use 'like' to automatically get the correct data type (e.g., uint8).
                    msg_data = zeros(h, w, c, num_msgs, 'like', first_img);
                    msg_data(:,:,:,1) = first_img;
                    for i = 2:num_msgs
                        msg_data(:, :, :, i) = rosReadImage(msg_structs(i));
                    end
                 
                %% OPTIMIZATION: Combine identical cases.
                case {'std_msgs/Float32MultiArray', 'std_msgs/Float64MultiArray'}
                    %% OPTIMIZATION: Use cellfun for faster processing than a loop.
                    % This extracts all 'Data' fields into a cell array at once.
                    all_data = arrayfun(@(s) double(s.Data), msg_structs, 'UniformOutput', false);
                    % Handle cases where 'Data' might be empty.
                    is_empty = cellfun('isempty', all_data);
                    all_data(is_empty) = {NaN};
                    % Pad with NaNs if lengths are inconsistent, then convert to matrix.
                    max_len = max(cellfun(@length, all_data));
                    padded_data = cellfun(@(d) [d(:); NaN(max_len - length(d), 1)], all_data, 'UniformOutput', false);
                    msg_data = [padded_data{:}];

                case 'geometry_msgs/TransformStamped'
                    %% OPTIMIZATION: Fully vectorized extraction. No loops.
                    transforms = [msg_structs.Transform];
                    translations = [transforms.Translation];
                    rotations = [transforms.Rotation];
                    msg_data = [
                        [translations.X]; [translations.Y]; [translations.Z];
                        [rotations.W]; [rotations.X]; [rotations.Y]; [rotations.Z]
                    ];

                case 'geometry_msgs/PointStamped'
                    %% OPTIMIZATION: Fully vectorized extraction. No loops.
                    points = [msg_structs.Point];
                    msg_data = [[points.X]; [points.Y]; [points.Z]];

                case 'geometry_msgs/WrenchStamped'
                    %% OPTIMIZATION: Fully vectorized extraction. No loops.
                    wrenches = [msg_structs.Wrench];
                    forces = [wrenches.Force];
                    torques = [wrenches.Torque];
                    msg_data = [
                        [forces.X]; [forces.Y]; [forces.Z];
                        [torques.X]; [torques.Y]; [torques.Z]
                    ];

                 case 'vicon_bridge/Markers'
                     [msg_data, obj.marker_dictionary] = marker_management(msg_cell, "skip_unknown", true);

                case 'sensor_msgs/PointCloud'
                    %% OPTIMIZATION: Pre-allocate and fill.
                    % Assuming the number of points is constant. If not, the logic
                    % becomes more complex and might require a cell array output.
                    n_points = length(msg_structs(1).Points);
                    msg_data = zeros(3 * n_points, num_msgs);
                    for i = 1:num_msgs
                        points = [msg_structs(i).Points];
                        % The reshape ensures column-major stacking [x1;y1;z1;x2;y2;z2;...]
                        msg_data(:, i) = reshape([[points.X]; [points.Y]; [points.Z]], [], 1);
                    end

                case 'sensor_msgs/PointCloud2'
                    %% OPTIMIZATION: Pre-allocate and fill.
                    first_cloud = rosReadXYZ(msg_structs(1));
                    [n_points, ~] = size(first_cloud);
                    msg_data = zeros(n_points * 3, num_msgs);
                    % We already have the first cloud, so we start the loop from 2.
                    msg_data(:, 1) = reshape(first_cloud', [], 1);
                    for i = 2:num_msgs
                        points = rosReadXYZ(msg_structs(i));
                        msg_data(:, i) = reshape(points', [], 1);
                    end

                otherwise
                    % Fallback for unhandled types
                    msg_data = msg_cell;
            end
        end
        
        % Extract Topics & Msgs
        function extractMsgs(obj)
            for i = 1:obj.n_topics
                topic_cell = select(obj.bag_obj, 'Topic', obj.topic_names{i});
                msg_cell = readMessages(topic_cell, 'DataFormat', 'struct');
                
                if isempty(msg_cell)
                    continue;
                end
                
                topic_time = topic_cell.MessageList.Time - obj.start_time;
                obj.msg_type{i} = msg_cell{1}.MessageType;
                obj.n_msgs(i) = length(msg_cell);
                
                % Call the optimized data extraction method
                data = obj.extractData(msg_cell);
                
                obj.topics_ts{i} = struct('Time', topic_time', 'Data', data);
            end
        end
        
        % Synchronization
        function [merged_time, merged_dataset, sync_marker_dic, topics] = synchronization(obj, resampling_period, mask)
            method = 'previous'; % hardcoded for the image
            if nargin <= 2
                mask = true(1, obj.n_topics);
            end
            
            merged_time = (0:resampling_period:obj.bag_duration)';
            num_merged_samples = length(merged_time);
            merged_dataset = cell(1, obj.n_topics);
            topics = cell(1, obj.n_topics);

            for i = 1:obj.n_topics
                if isempty(obj.topics_ts{i}) || ~mask(i)
                    continue;
                end
                
                ts = obj.topics_ts{i};
                msg_type = obj.msg_type{i};
                
                if contains(msg_type, "Image")
                    %% OPTIMIZATION: Vectorized image synchronization.
                    % Avoids a slow loop by finding source indices and using a
                    % single, powerful indexing operation to assign all images at once.
                    
                    % 1. Interpolate an index, not the data itself.
                    time_indices = (1:length(ts.Time))';
                    interp_indices = interp1(ts.Time, time_indices, merged_time, method);
                    
                    % 2. Pre-allocate the final 4D matrix with a fill value (zeros).
                    [h, w, c, ~] = size(ts.Data);
                    % Using uint8(0) as a fill value instead of NaN for integer types.
                    synced_images = zeros(h, w, c, num_merged_samples, 'like', ts.Data);
                    
                    % 3. Find where the valid (non-NaN) indices are.
                    valid_mask = ~isnan(interp_indices);
                    source_indices = interp_indices(valid_mask);
                    
                    % 4. Assign all valid images in one go.
                    synced_images(:, :, :, valid_mask) = ts.Data(:, :, :, source_indices);
                    merged_dataset{i} = synced_images;

                else
                    % Standard interpolation for numeric data.
                    resampled_data = interp1(ts.Time, ts.Data', merged_time, method);
                    merged_dataset{i} = resampled_data'; % Transpose back to column format
                    
                    if msg_type == "vicon_bridge/Markers"
                        % Synchronize markers' dictionary
                        marker_names = fieldnames(obj.marker_dictionary);
                        for j = 1:length(marker_names)
                            marker_data = obj.marker_dictionary.(marker_names{j});
                            synced_marker_data = interp1(ts.Time, marker_data', merged_time, method);
                            obj.marker_dictionary.(marker_names{j}) = synced_marker_data';
                        end
                    end
                end
                topics{i} = obj.topic_names{i};
            end
        
            % Remove empty cells from skipped topics
            is_empty_cell = cellfun('isempty', merged_dataset);
            merged_dataset = merged_dataset(~is_empty_cell);
            topics = topics(~is_empty_cell);
            
            sync_marker_dic = obj.marker_dictionary;
        end
    end
end