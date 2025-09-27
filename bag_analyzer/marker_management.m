%%% Marker Data Extraction Management %%%
function [msg_data, marker_dict] = marker_management(msg_cell, options)
    arguments
        msg_cell;
        options.skip_unknown = true; % Default to true as it's common practice
    end

    % --- Step 1: Discover all unique marker names and create a stable order ---
    all_marker_names = {};
    for i = 1:length(msg_cell)
        for j = 1:length(msg_cell{i}.Markers_)
            % Marker Name
            marker_name = msg_cell{i}.Markers_(j).MarkerName;

            % Add name if it's not empty or if we are not skipping unknowns
            if ~isempty(marker_name) || ~options.skip_unknown
                all_marker_names{end+1} = convertStringsToChars(string(msg_cell{i}.Markers_(j).SubjectName) + "/" + string(marker_name));
            end
        end
    end
    
    % Get the sorted, unique list of marker names
    unique_names = sort(unique(all_marker_names));
    if options.skip_unknown
        % Ensure empty string "" is not in our list if we are skipping unknowns
        unique_names = unique_names(~cellfun('isempty', unique_names));
    end
    num_unique_markers = length(unique_names);
    
    % Create a map for quick name-to-index lookup
    marker_map = containers.Map(unique_names, 1:num_unique_markers);

    % --- Step 2: Pre-allocate the output matrix with NaNs ---
    % Rows will be [marker1_x, marker1_y, marker1_z, marker2_x, ...]'
    msg_data = NaN(3 * num_unique_markers, length(msg_cell));

    % --- Step 3: Populate the matrix using the stable map ---
    for i = 1:length(msg_cell) % Iterate through each time step (message)
        for j = 1:length(msg_cell{i}.Markers_) % Iterate through markers present at this time
            
            marker = msg_cell{i}.Markers_(j);
            % marker_name = marker.MarkerName;
            marker_name = convertStringsToChars(string(marker.SubjectName) + "/" + string(marker.MarkerName));
            
            % If the marker name exists in our map, place its data in the correct rows
            if isKey(marker_map, marker_name)
                % Get the fixed index for this marker (e.g., 1 for 'C7', 2 for 'L_Heel')
                marker_idx = marker_map(marker_name);
                
                % Calculate the corresponding rows in the data matrix
                row_indices = (marker_idx - 1) * 3 + (1:3);

                if(marker.Occluded == 1)
                    msg_data(row_indices, i) = NaN*ones(3, 1);
                else
                    % Extract coordinates, convert from mm to meters, and assign
                    msg_data(row_indices, i) = [marker.Translation.X; ...
                                                  marker.Translation.Y; ...
                                                  marker.Translation.Z] / 1000;
                end
                
            end
        end
    end
    
    % --- Step 4 (Optional but helpful): Create a dictionary (struct) for easy access ---
    marker_dict = struct();
    for i = 1:num_unique_markers
        name = unique_names{i};
        % Sanitize name to be a valid MATLAB field name (e.g., "Marker 1" -> "Marker1")
        valid_name = matlab.lang.makeValidName(name);
        row_indices = (i-1)*3 + (1:3);
        marker_dict.(valid_name) = msg_data(row_indices, :);
    end
end