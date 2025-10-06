function new_dict = rename_markers(old_dict, options)
    arguments
        old_dict
        options.preserve_order = true;
    end
    % Get field names of the input structure
    fields = fieldnames(old_dict);

    % Extract numeric IDs from field names (assuming format 'marker_number')
    ids = zeros(size(fields));
    for i = 1:numel(fields)
        tokens = regexp(fields{i}, 'marker_(\d+)', 'tokens');
        if isempty(tokens)
            error('Field %s does not match expected format marker_number.', fields{i});
        end
        ids(i) = str2double(tokens{1}{1});
    end

    % Sort by IDs or Preserve the order
    order = 1:length(ids);
    if ~options.preserve_order
        [~, order] = sort(ids);        
    end

    % Build new structure with renamed fields marker1, marker2, ...
    new_dict = struct();
    for i = 1:numel(order)
        newField = sprintf('marker%d', i);
        new_dict.(newField) = old_dict.(fields{order(i)});
    end
end