%%% bag analyzer %%%
clear all 
close all
clc

%% Load Bagfile
bagdir_name = "bags";
bagname = "formazione";
bag_ext = ".bag";

% Add bag dir
addpath("bag_analyzer");
addpath(bagdir_name);

% fullfile makes compatible with windows or ubuntu paths
% Ubuntu "/" | Windows "\"
% bag = rosbag(fullfile(bagdir_name, bagname + bag_ext));
bag_analyzer_obj = Bag_Analyzer(fullfile(bagdir_name, bagname + bag_ext));

%% Merge Dataset
merged_dataset = bag_analyzer_obj.merge_dataset();

%% Cell2Array
% Pressures
for i = 1:length(merged_dataset.data{2})
    pressures(:, i) = merged_dataset.data{2}{i};
    tip_position(:, i) = merged_dataset.data{5}{i}.Translation;
    tip_orientation(:, i) = merged_dataset.data{5}{i}.Rotation;
end

%% Plot Synchronized Data
% Init Figure
fig  = figure;
fig.WindowState = 'fullscreen';
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Rec or just show
rec = true;

% Open Video
if rec
    v = VideoWriter("video_exp.mp4", 'MPEG-4');
    open(v);
end

for i = 1:length(merged_dataset.time)
    % Plot Pressures
    nexttile(1)
    plot(merged_dataset.time(1:i), pressures(:, 1:i), 'LineWidth', 2.0)
    grid on
    title("Pressures")
    xlabel("Time [s]")
    ylabel("Pressures [bar]")
    legend("Chamber 1", "Chamber 2", "Chamber 3", ...
            "Chamber 4", "Chamber 5", "Chamber 6", ...
            "Chamber 7", "Chamber 8", "Chamber 9")
    if(i ~= 1)
        xlim([merged_dataset.time(1), merged_dataset.time(i)])
    end

    % Show Frame
    nexttile(2)
    imshow(merged_dataset.data{1}{i})
    title("Video of the Experience")

    % Tip Position
    nexttile(3)
    plot(merged_dataset.time(1:i), tip_position(:, 1:i),  'LineWidth', 2.0)
    grid on
    title("Tip Position")
    xlabel("Time [s]")
    ylabel("Tip Position [m]")
    legend("x", "y", "z")
    if(i ~= 1)
        xlim([merged_dataset.time(1), merged_dataset.time(i)])
    end

    % Tip Orientation
    nexttile(4)
    plot(merged_dataset.time(1:i), tip_orientation(:, 1:i),  'LineWidth', 2.0)
    grid on
    title("Tip Orientation")
    xlabel("Time [s]")
    ylabel("Tip Orientation (Quaternion)")
    legend("w", "x", "y", "z")
    if(i ~= 1)
        xlim([merged_dataset.time(1), merged_dataset.time(i)])
    end

    if rec
        frame = getframe(gcf);
        writeVideo(v, frame);
    else
        drawnow
    end
end

if rec
    close(v);
end



