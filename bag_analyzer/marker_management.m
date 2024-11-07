%%% Marker Data Extraction Management %%%
function msg_data = marker_management(msg_cell)
    % Update Marker Dictionary
    for i = 1:length(msg_cell)
        n_markers = length(msg_cell{i}.Markers_);
        
        % Fill vector
        markers_pos = [];
        for j = 1:n_markers
            % Skip if is a unlabeled marker
            if(msg_cell{i}.Markers_(j).MarkerName == "")
                continue
            end

            single_marker = [msg_cell{i}.Markers_(j).Translation.X; msg_cell{i}.Markers_(j).Translation.Y; msg_cell{i}.Markers_(j).Translation.Z];
            markers_pos = [markers_pos; single_marker];
        end
        
        % Finally Update Markers
        msg_data(:, i) = markers_pos./1000; % markers are expressed in [mm]
    end    

end