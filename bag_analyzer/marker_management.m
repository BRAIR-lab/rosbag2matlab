%%% Marker Data Extraction Management %%%
function msg_data = marker_management(msg_cell)
    % Take maximum number of Markers
    for i = 1:length(msg_cell)
        n_markers(i) = length(msg_cell{i}.Markers_);
    end

    % Max Numbers
    max_markers = max(n_markers);
    % Init
    msg_data = NaN*ones(3*max_markers, length(msg_cell));
    
    % Update Marker Dictionary
    for i = 1:length(msg_cell)
        % Fill vector
        markers_pos = [];
        for j = 1:n_markers(i)
            % % Skip if is a unlabeled marker
            % if(msg_cell{i}.Markers_(j).MarkerName == "")
            %     continue
            % end

            single_marker = [msg_cell{i}.Markers_(j).Translation.X; msg_cell{i}.Markers_(j).Translation.Y; msg_cell{i}.Markers_(j).Translation.Z];
            markers_pos = [markers_pos; single_marker];
        end
        
        % Fill with unrecognized markers
        markers_pos = [markers_pos; NaN*ones(3*(max_markers - n_markers(i)), 1)];

        % Update New Markers
        if ~isempty(markers_pos)
            msg_data(:, i) = markers_pos./1000; % markers are expressed in [mm]
        end
    end
end