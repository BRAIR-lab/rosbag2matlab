%PROBE_ROS2_ASSUMPTIONS_REAL_BAG Re-run of probe_ros2_assumptions.m's
% checks against the real bag instead of a synthetic ros2bagwriter one,
% to catch anything the synthetic bag couldn't (real timestamps produced
% by an actual recorder, real multi-topic AvailableTopics table, etc.).

bag_path = fullfile("bags", "ros2bags", "collocated_cycle_2026_06_26-19_09_51");
bag_obj = ros2bagreader(bag_path);

results = struct('name', {}, 'pass', {}, 'note', {});

% StartTime/EndTime
st = bag_obj.StartTime; et = bag_obj.EndTime;
note = sprintf("class(StartTime)=%s class(EndTime)=%s values=[%.3f %.3f] duration=%.3fs", ...
    class(st), class(et), double(st), double(et), double(et) - double(st));
results(end+1) = struct('name', 'StartTime/EndTime plain double seconds', ...
    'pass', isa(st,'double') && isa(et,'double'), 'note', note); %#ok<SAGROW>

% AvailableTopics.Row on a real, 19-topic bag (not 1 synthetic topic)
row_names = bag_obj.AvailableTopics.Row;
ok = (iscell(row_names) || isstring(row_names)) && numel(row_names) == 19 ...
    && any(strcmp(string(row_names), "/md80/joint_states"));
note = sprintf("class=%s numel=%d (expected 19) contains /md80/joint_states=%d", ...
    class(row_names), numel(row_names), any(strcmp(string(row_names), "/md80/joint_states")));
results(end+1) = struct('name', 'AvailableTopics.Row on a real multi-topic bag', 'pass', ok, 'note', note); %#ok<SAGROW>

% select + readMessages(no DataFormat arg) on a real, high-count topic
sel = select(bag_obj, 'Topic', '/md80/joint_states');
msgs = readMessages(sel);
ok = iscell(msgs) && numel(msgs) == sel.NumMessages && isstruct(msgs{1});
note = sprintf("numel(msgs)=%d NumMessages=%d class(msgs{1})=%s", numel(msgs), sel.NumMessages, class(msgs{1}));
results(end+1) = struct('name', 'select+readMessages on real high-volume topic (6781 msgs)', 'pass', ok, 'note', note); %#ok<SAGROW>

% MessageList.Time: real recorder timestamps, not synthetic round numbers
tt = sel.MessageList.Time;
ok = isnumeric(tt) && numel(tt) == sel.NumMessages && all(diff(tt) >= 0);
note = sprintf("class=%s numel=%d monotonic-nondecreasing=%d first=%.6f last=%.6f", ...
    class(tt), numel(tt), all(diff(tt) >= 0), tt(1), tt(end));
results(end+1) = struct('name', 'MessageList.Time on real recorder-produced timestamps', 'pass', ok, 'note', note); %#ok<SAGROW>

fprintf("\n=== Results (real bag) ===\n");
for i = 1:numel(results)
    r = results(i);
    if r.pass, status = "VERIFIED"; else, status = "CONTRADICTED/ERROR"; end
    fprintf("[%s] %s\n    %s\n", status, r.name, r.note);
end
