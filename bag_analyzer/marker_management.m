function msg_data = marker_management(msg_cell, marker_dict)
    % marker_dict is a dictionary (string -> index)
    
    n_msgs = length(msg_cell);
    msg_data = []; % dynamic allocation
    
    % For each message
    for i = 1:n_msgs
        % Initialize a temporary row with NaN for all known markers
        temp_data = NaN(3*length(marker_dict),1);
        
        % For each marker in this message
        for j = 1:length(msg_cell{i}.Markers_)
            m = msg_cell{i}.Markers_(j);
            m_name = string(m.MarkerName);
            
            % If new marker name appears, expand dictionary
            if ~isKey(marker_dict, m_name)
                marker_dict(m_name) = length(marker_dict) + 1;
                
                % Expand msg_data for all past samples with NaN
                n_old = size(msg_data,2);
                new_block = NaN(3, n_old);
                msg_data = [msg_data; new_block];
                
                % Expand current row as well
                temp_data = [temp_data; NaN(3,1)];
            end
            
            % Row index for this marker
            idx = marker_dict(m_name);
            row_range = (3*(idx-1)+1):(3*idx);
            
            % Store coordinates (converted to meters)
            temp_data(row_range,1) = [m.Translation.X; m.Translation.Y; m.Translation.Z] ./ 1000;
        end
        
        % Append this message’s vector
        msg_data(:,i) = temp_data;
    end
end