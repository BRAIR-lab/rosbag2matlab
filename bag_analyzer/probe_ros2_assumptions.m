%PROBE_ROS2_ASSUMPTIONS Empirically verify the ROS2 assumptions flagged in
% Bag_Analyzer.m, against a synthetic ros2bagwriter-produced bag (no real
% robot bag needed / available). Read-only inspection, writes only to a
% scratch temp folder. Prints VERIFIED/CONTRADICTED per assumption.

tmp_root = fullfile(tempdir, "ros2_probe_" + string(datetime("now"), "yyyyMMdd_HHmmssSSS"));
bag_dir = fullfile(tmp_root, "probe_bag");

fprintf("=== probe_ros2_assumptions ===\n");
fprintf("writing synthetic ros2 bag to: %s\n", bag_dir);

writer = ros2bagwriter(bag_dir);
msg1 = ros2message("geometry_msgs/PoseStamped");
msg1.pose.position.x = 1.0;
msg1.pose.position.y = 2.0;
msg1.pose.position.z = 3.0;
msg1.pose.orientation.w = 0.7071;
msg1.pose.orientation.x = 0.0;
msg1.pose.orientation.y = 0.7071;
msg1.pose.orientation.z = 0.0;

msg2 = msg1;
msg2.pose.position.x = 4.0;

t0 = 1000.0;
dt = 0.1;
write(writer, "/probe_topic", t0,      msg1);
write(writer, "/probe_topic", t0 + dt, msg2);
clear writer; % flush/close

fprintf("bag written. contents:\n");
dir(bag_dir)

%% --- Open with ros2bagreader, exactly as Bag_Analyzer does ---
bag_obj = ros2bagreader(bag_dir);

results = struct('name', {}, 'pass', {}, 'note', {});

% Assumption: StartTime/EndTime numeric-seconds (or at least normalizable)
try
    st = bag_obj.StartTime;
    et = bag_obj.EndTime;
    note = sprintf("StartTime class=%s EndTime class=%s values=[%s %s]", ...
        class(st), class(et), mat2str(double(st)), mat2str(double(et)));
    results(end+1) = struct('name', 'StartTime/EndTime class+normalizable', 'pass', true, 'note', note); %#ok<SAGROW>
catch ME
    results(end+1) = struct('name', 'StartTime/EndTime class+normalizable', 'pass', false, 'note', ME.message); %#ok<SAGROW>
end

% Assumption 1: AvailableTopics.Row gives topic names (like ROS1 BagSelection)
try
    row_names = bag_obj.AvailableTopics.Row;
    ok = iscell(row_names) || isstring(row_names);
    ok = ok && any(strcmp(string(row_names), "/probe_topic"));
    note = sprintf("AvailableTopics.Row = %s (class %s)", mat2str(string(row_names)), class(row_names));
    results(end+1) = struct('name', 'AvailableTopics.Row exposes topic names', 'pass', ok, 'note', note); %#ok<SAGROW>
catch ME
    results(end+1) = struct('name', 'AvailableTopics.Row exposes topic names', 'pass', false, 'note', ME.message); %#ok<SAGROW>
end

% Assumption 2a: select(...,'Topic',...) works the same as ROS1
topic_sel = [];
try
    topic_sel = select(bag_obj, 'Topic', '/probe_topic');
    note = sprintf("class(select(...))=%s", class(topic_sel));
    results(end+1) = struct('name', 'select(bag,''Topic'',name) syntax parity', 'pass', true, 'note', note); %#ok<SAGROW>
catch ME
    results(end+1) = struct('name', 'select(bag,''Topic'',name) syntax parity', 'pass', false, 'note', ME.message); %#ok<SAGROW>
end

% Assumption 2b: readMessages(...,'DataFormat','struct') -- AS WRITTEN IN
% Bag_Analyzer.m today -- does this exact call work for ROS2?
try
    msg_cell = readMessages(topic_sel, 'DataFormat', 'struct'); %#ok<NASGU>
    results(end+1) = struct('name', 'readMessages(sel,''DataFormat'',''struct'') AS-WRITTEN', 'pass', true, 'note', 'accepted'); %#ok<SAGROW>
catch ME
    results(end+1) = struct('name', 'readMessages(sel,''DataFormat'',''struct'') AS-WRITTEN', 'pass', false, 'note', ME.message); %#ok<SAGROW>
end

% What DOES work for ROS2, per ros2bagreader/readMessages doc (no
% DataFormat param; always returns cell array of structs)?
try
    msg_cell = readMessages(topic_sel);
    ok = iscell(msg_cell) && numel(msg_cell) == 2 && isstruct(msg_cell{1});
    note = sprintf("class(msg_cell)=%s numel=%d class(msg_cell{1})=%s", ...
        class(msg_cell), numel(msg_cell), class(msg_cell{1}));
    results(end+1) = struct('name', 'readMessages(sel) [no DataFormat arg] -- ROS2-correct form', 'pass', ok, 'note', note); %#ok<SAGROW>
catch ME
    results(end+1) = struct('name', 'readMessages(sel) [no DataFormat arg] -- ROS2-correct form', 'pass', false, 'note', ME.message); %#ok<SAGROW>
end

% Check field-layout parity used by Bag_Analyzer.extractData /
% decodePoseStamped: does it use msg_cell{i}.Pose.Position.X (ROS1-style
% capitalized) or msg_cell{i}.pose.position.x (ROS2 struct convention)?
try
    if ~isempty(topic_sel) && exist('msg_cell', 'var') && ~isempty(msg_cell)
        m = msg_cell{1};
        has_capitalized = isfield(m, 'Pose') && isfield(m.Pose, 'Position') && isfield(m.Pose.Position, 'X');
        has_lowercase   = isfield(m, 'pose') && isfield(m.pose, 'position') && isfield(m.pose, 'position') && isfield(m.pose.position, 'x');
        note = sprintf("fieldnames(msg)=%s | has ROS1-style Pose.Position.X=%d | has ROS2-style pose.position.x=%d", ...
            strjoin(fieldnames(m), ','), has_capitalized, has_lowercase);
        results(end+1) = struct('name', 'struct field CASE parity vs ROS1 (Bag_Analyzer assumes capitalized)', ...
            'pass', has_capitalized, 'note', note); %#ok<SAGROW>
    end
catch ME
    results(end+1) = struct('name', 'struct field CASE parity vs ROS1', 'pass', false, 'note', ME.message); %#ok<SAGROW>
end

% Assumption 3: MessageList.Time units (numeric seconds, same basis as StartTime)
try
    tt = topic_sel.MessageList.Time;
    ok = isnumeric(tt) && numel(tt) == 2;
    note = sprintf("class(MessageList.Time)=%s values=%s bag.StartTime(normalized)=%s diff-from-t0=%s", ...
        class(tt), mat2str(double(tt)), mat2str(double(bag_obj.StartTime)), mat2str(double(tt) - t0));
    results(end+1) = struct('name', 'MessageList.Time numeric-seconds, same basis as write() timestamps', 'pass', ok, 'note', note); %#ok<SAGROW>
catch ME
    results(end+1) = struct('name', 'MessageList.Time numeric-seconds, same basis as write() timestamps', 'pass', false, 'note', ME.message); %#ok<SAGROW>
end

fprintf("\n=== Results ===\n");
for i = 1:numel(results)
    r = results(i);
    if r.pass, status = "VERIFIED"; else, status = "CONTRADICTED/ERROR"; end
    fprintf("[%s] %s\n    %s\n", status, r.name, r.note);
end

% Cleanup
clear bag_obj topic_sel
try
    rmdir(tmp_root, 's');
catch
    fprintf("(non-fatal: could not remove temp folder %s)\n", tmp_root);
end
