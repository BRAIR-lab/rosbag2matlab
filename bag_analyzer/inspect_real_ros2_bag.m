%% inspect_real_ros2_bag.m -- read-only inspection of a real ROS2 bag
% Reports topics, message types, message counts, and (for the first
% message on each topic) the actual struct field layout -- so extractData
% can be fixed against REAL field names instead of guesses.

bag_path = fullfile("bags", "ros2bags", "collocated_cycle_2026_06_26-19_09_51");
bag_obj = ros2bagreader(bag_path);

fprintf("=== bag info ===\n");
fprintf("StartTime=%.3f EndTime=%.3f duration=%.3f s\n", bag_obj.StartTime, bag_obj.EndTime, bag_obj.EndTime - bag_obj.StartTime);
fprintf("NumMessages=%d\n\n", bag_obj.NumMessages);

topics = bag_obj.AvailableTopics;
disp(topics(:, {'NumMessages', 'MessageType'}));

topic_names = topics.Row;
for i = 1:numel(topic_names)
    tname = topic_names{i};
    try
        fprintf("\n=== topic: %s ===\n", tname);
        sel = select(bag_obj, 'Topic', tname);
        fprintf("MessageType (from AvailableTopics): %s\n", string(topics.MessageType(i)));
        fprintf("NumMessages: %d\n", sel.NumMessages);
        if sel.NumMessages == 0
            continue
        end
        msgs = readMessages(sel, 1);
        m = msgs{1};
        fprintf("Top-level fields: %s\n", strjoin(fieldnames(m), ', '));
        print_struct_recursive(m, "  ", 0);
    catch ME
        fprintf("  [ERROR on topic %s]: %s\n", tname, ME.message);
    end
end
fprintf("\n=== done, all topics processed ===\n");

function print_struct_recursive(s, prefix, depth)
    if depth > 3
        fprintf("%s...(depth limit)\n", prefix);
        return
    end
    fns = fieldnames(s);
    for k = 1:numel(fns)
        f = fns{k};
        v = s.(f);
        if isstruct(v)
            if numel(v) > 1
                fprintf("%s%s: struct array [1x%d]\n", prefix, f, numel(v));
                if numel(v) >= 1
                    print_struct_recursive(v(1), prefix + "  ", depth + 1);
                end
            else
                fprintf("%s%s: struct\n", prefix, f);
                print_struct_recursive(v, prefix + "  ", depth + 1);
            end
        elseif ischar(v) || isstring(v)
            fprintf("%s%s: %s = '%s'\n", prefix, f, class(v), char(v));
        elseif isnumeric(v) || islogical(v)
            if numel(v) <= 6
                fprintf("%s%s: %s = %s\n", prefix, f, class(v), mat2str(v));
            else
                fprintf("%s%s: %s [%s]\n", prefix, f, class(v), mat2str(size(v)));
            end
        else
            fprintf("%s%s: %s\n", prefix, f, class(v));
        end
    end
end
