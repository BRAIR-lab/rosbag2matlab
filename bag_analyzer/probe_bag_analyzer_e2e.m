%PROBE_BAG_ANALYZER_E2E End-to-end smoke test of Bag_Analyzer against a
% synthetic ROS2 bag: confirms the constructor + extractMsgs pipeline
% (dispatch points 1-5: open/time/topics/select+readMessages/timestamps)
% runs without error after the readMessages dispatch fix.
%
% EXPECTED RESULT (as of this commit): [FAIL] Bag_Analyzer(bag_dir) threw:
% Unrecognized field name "Pose". This is not a bug in this probe -- it is
% the CONFIRMED extractData field-case gap documented at the top of
% Bag_Analyzer.extractData (ROS2 struct fields are lowercase, e.g.
% pose.position.x, not Pose.Position.X). Requires ODO/PR-scoped work to
% fix; this script exists to make the gap reproducible and re-checkable
% once that fix lands (it should then print [PASS] and the decoded data).

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
    fprintf("       topics_ts{1}.Data (PoseStamped decode, expect WRONG per flagged gap):\n");
    disp(ba.topics_ts{1}.Data);
catch ME
    fprintf("[FAIL] Bag_Analyzer(bag_dir) threw: %s\n", ME.message);
end

try
    rmdir(tmp_root, 's');
catch
end
