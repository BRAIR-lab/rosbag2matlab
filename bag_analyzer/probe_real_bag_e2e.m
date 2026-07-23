%% probe_real_bag_e2e.m -- run Bag_Analyzer against the real ROS2 bag
bag_path = fullfile("bags", "ros2bags", "collocated_cycle_2026_06_26-19_09_51");
try
    ba = Bag_Analyzer(bag_path);
    fprintf("[PASS] Bag_Analyzer constructed. n_topics=%d\n", ba.n_topics);
    for i = 1:ba.n_topics
        fprintf("  topic %-25s type=%-45s n_msgs=%d\n", ba.topic_names{i}, ba.msg_type{i}, ba.n_msgs(i));
    end
catch ME
    fprintf("[FAIL] Bag_Analyzer threw: %s\n", ME.message);
    fprintf("stack:\n");
    for k = 1:numel(ME.stack)
        fprintf("  %s (line %d)\n", ME.stack(k).name, ME.stack(k).line);
    end
end
