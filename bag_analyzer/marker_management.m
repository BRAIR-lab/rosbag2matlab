%%% Marker Data Extraction Management %%%
function [msg_data, marker_dict] = marker_management(msg_cell, options)
    arguments
        msg_cell;
        options.skip_unknown   = true;          % Default to true as it's common practice
        options.replace_style  = 'underscore';
        options.preserve_order = false;
    end

    % --- Helper: build the canonical key for a marker exactly ONE way ---------
    % Used in BOTH the discovery pass and the fill pass so the two can never
    % drift apart (this was Bug B: keys built with SubjectName prefix in Step 1
    % but looked up bare in Step 3, leaving those markers all-NaN forever).
    build_key = @(mk) local_build_key(mk);

    % --- Step 1: Discover all unique marker names across ALL messages --------
    all_marker_names = {};
    for i = 1:length(msg_cell)
        for j = 1:length(msg_cell{i}.Markers_)
            mk   = msg_cell{i}.Markers_(j);
            name = build_key(mk);

            % Add name if it's not empty, or if we are not skipping unknowns
            if ~isempty(name) || ~options.skip_unknown
                all_marker_names{end+1} = name; %#ok<AGROW>
            end
        end
    end

    % Sorted, unique list of marker names
    unique_names = sort(unique(all_marker_names));

    % Drop empty names when skipping unknowns
    if options.skip_unknown
        unique_names = unique_names(~cellfun('isempty', unique_names));
    end

    % --- Preserve order (built from the UNION across all messages) -----------
    % Bug A/C fix: the desired order is now discovered across every message,
    % not just msg_cell{1}, and any name not found is dropped safely instead
    % of producing a zero index that errors or silently deletes markers.
    if options.preserve_order
        marker_order = {};
        seen = containers.Map('KeyType', 'char', 'ValueType', 'logical');
        for i = 1:length(msg_cell)
            for j = 1:length(msg_cell{i}.Markers_)
                name = build_key(msg_cell{i}.Markers_(j));
                if ~isempty(name) && ~isKey(seen, name)
                    seen(name) = true;
                    marker_order{end+1} = name; %#ok<AGROW>
                end
            end
        end
        % Keep only names that survived the skip_unknown filtering, in
        % first-seen order.
        marker_order = marker_order(ismember(marker_order, unique_names));
        unique_names = marker_order;
    end

    num_unique_markers = length(unique_names);

    % Map for quick name -> index lookup
    marker_map = containers.Map(unique_names, 1:num_unique_markers);

    % --- Step 2: Pre-allocate the output matrix with NaNs --------------------
    % Rows are [marker1_x, marker1_y, marker1_z, marker2_x, ...]'
    msg_data = NaN(3 * num_unique_markers, length(msg_cell));

    % --- Step 3: Populate the matrix using the stable map -------------------
    for i = 1:length(msg_cell)                      % each time step (message)
        for j = 1:length(msg_cell{i}.Markers_)      % each marker in this message
            marker = msg_cell{i}.Markers_(j);
            name   = build_key(marker);             % SAME key builder as Step 1

            if ~isempty(name) && isKey(marker_map, name)
                marker_idx  = marker_map(name);
                row_indices = (marker_idx - 1) * 3 + (1:3);

                if marker.Occluded == 1
                    % Leave as NaN (occluded -> no valid measurement)
                    msg_data(row_indices, i) = NaN;
                else
                    % Extract coordinates, convert mm -> meters
                    msg_data(row_indices, i) = [marker.Translation.X; ...
                                                marker.Translation.Y; ...
                                                marker.Translation.Z] / 1000;
                end
            end
        end
    end

    % --- Step 4: Struct dictionary for easy field access --------------------
    marker_dict = struct();
    for i = 1:num_unique_markers
        name        = unique_names{i};
        valid_name  = matlab.lang.makeValidName(name, ...
                          'ReplacementStyle', options.replace_style);
        row_indices = (i - 1) * 3 + (1:3);
        marker_dict.(valid_name) = msg_data(row_indices, :);
    end
end

% -------------------------------------------------------------------------
function key = local_build_key(mk)
    % Canonical key for a single marker struct.
    % Always returns a char row vector (containers.Map keys must be char,
    % and mixing char keys with string lookups silently fails to match).
    marker_name = mk.MarkerName;

    if isempty(mk.SubjectName)
        key = convertStringsToChars(string(marker_name));
    else
        key = convertStringsToChars( ...
                  string(mk.SubjectName) + "/" + string(marker_name));
    end

    % Normalise empty results to '' so isempty() behaves as expected
    if isempty(key)
        key = '';
    end
end


function new_dict = rename_markers(old_dict, options)
    arguments
        old_dict
        options.preserve_order = true;
    end

    % Field names of the input structure
    fields = fieldnames(old_dict);

    % Extract numeric IDs from field names (expected format 'marker_number')
    ids = zeros(size(fields));
    for i = 1:numel(fields)
        tokens = regexp(fields{i}, 'marker_(\d+)', 'tokens');
        if isempty(tokens)
            error('Field %s does not match expected format marker_number.', fields{i});
        end
        ids(i) = str2double(tokens{1}{1});
    end

    % preserve_order = true  -> keep numeric-ID order (sorted by ID)
    % preserve_order = false -> keep original fieldnames() insertion order
    % (Bug E fix: "preserve order" now means the meaningful marker-ID order,
    %  which is what downstream sequential renumbering expects.)
    if options.preserve_order
        [~, order] = sort(ids);
    else
        order = 1:length(ids);
    end

    % Build new structure with renamed fields marker1, marker2, ...
    new_dict = struct();
    for i = 1:numel(order)
        newField = sprintf('marker%d', i);
        new_dict.(newField) = old_dict.(fields{order(i)});
    end
end