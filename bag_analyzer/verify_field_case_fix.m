%% verify_field_case_fix.m -- confirm extracted DATA (not just no-crash) is
% correct for the two message types verified against the real bag.

bag_path = fullfile("bags", "ros2bags", "collocated_cycle_2026_06_26-19_09_51");
ba = Bag_Analyzer(bag_path);

n_pass = 0; n_fail = 0;

% --- sensor_msgs/JointState: /md80/joint_states ---
idx = find(strcmp(ba.topic_names, "/md80/joint_states"));
ts = ba.topics_ts{idx};

bag_obj = ros2bagreader(bag_path);
sel = select(bag_obj, 'Topic', '/md80/joint_states');
raw = readMessages(sel, 1);
m1 = raw{1};
expected_col1 = [m1.position; m1.velocity; m1.effort];

[n_pass, n_fail] = check(n_pass, n_fail, 'JointState first column == [position;velocity;effort]', ...
    isequal(ts.Data(:,1), expected_col1));

% --- std_msgs/Float64MultiArray: /Lc ---
idx = find(strcmp(ba.topic_names, "/Lc"));
ts = ba.topics_ts{idx};
sel = select(bag_obj, 'Topic', '/Lc');
raw = readMessages(sel, 1);
expected_col1 = double(raw{1}.data);
[n_pass, n_fail] = check(n_pass, n_fail, 'Float64MultiArray first column == data', ...
    isequal(ts.Data(1:numel(expected_col1), 1), expected_col1));

fprintf('\n%d passed, %d failed\n', n_pass, n_fail);

function [n_pass, n_fail] = check(n_pass, n_fail, label, ok)
    if ok
        fprintf('[PASS] %s\n', label);
        n_pass = n_pass + 1;
    else
        fprintf('[FAIL] %s\n', label);
        n_fail = n_fail + 1;
    end
end
