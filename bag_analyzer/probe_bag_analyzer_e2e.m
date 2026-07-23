%PROBE_BAG_ANALYZER_E2E End-to-end smoke test of Bag_Analyzer against a
% synthetic ROS2 bag: confirms the constructor + extractMsgs + extractData
% pipeline runs without error and decodes PoseStamped correctly.
%
% EXPECTED RESULT (as of this commit): [PASS], with decoded data
% [1;2;3;0.7071;0;0;0] in both columns (matching the two identical
% messages written below). This used to [FAIL] with "Unrecognized field
% name 'Pose'" -- see Bag_Analyzer.gf() and extractData's header comment
% for the fix (also verified against a real ROS2 bag, see
% inspect_real_ros2_bag.m / verify_field_case_fix.m).

tmp_root = fullfile(tempdir, "ros2_e2e_" + string(datetime("now"), "yyyyMMdd_HHmmssSSS"));
bag_dir = fullfile(tmp_root, "e2e_bag");

writer = ros2bagwriter(bag_dir);
msg1 = ros2message("geometry_msgs/PoseStamped");
msg1.pose.position.x = 1.0;
msg1.pose.position.y = 2.0;
msg1.pose.position.z = 3.0;
msg1.pose.orientation.w = 0.7071;
write(writer, "/probe_topic", 1000.0, msg1);
write(writer, "/probe_topic", 1000.1, msg1);
clear writer;

try
    ba = Bag_Analyzer(bag_dir);
    fprintf("[PASS] Bag_Analyzer(bag_dir) constructed without error. n_topics=%d\n", ba.n_topics);
    fprintf("       topics_ts{1}.Data (PoseStamped decode, expect [1;2;3;0.7071;0;0;0] per column):\n");
    disp(ba.topics_ts{1}.Data);
catch ME
    fprintf("[FAIL] Bag_Analyzer(bag_dir) threw: %s\n", ME.message);
end

try
    rmdir(tmp_root, 's');
catch
end
