% === Data Loading & Setup ===
clear;

% Load CSV and format timestamps to UTC
data = readtable('data/braking_all.csv');
data.Timestamp = datetime(data.Timestamp/1000, 'ConvertFrom', 'posixtime', 'TimeZone', 'UTC');

% Convert tilt references from radians to degrees
data.tilt_xref = data.tilt_xref * (180 / pi);
data.tilt_yref = data.tilt_yref * (180 / pi);

% Calculate Required Output Torque (ROS)
ROS_scaler = 5;
data.ROS_1 = (data.t_ff_1 / 0.125 * 2) ./ (data.F_1 * 1.38 * 9.8) ./ROS_scaler;
data.ROS_2 = (data.t_ff_2 / 0.125 * 2) ./ (data.F_2 * 1.38 * 9.8) ./ROS_scaler;
data.ROS_3 = (data.t_ff_3 / 0.125 * 2) ./ (data.F_3 * 1.38 * 9.8) ./ROS_scaler;
data.ROS_4 = (data.t_ff_4 / 0.125 * 2) ./ (data.F_4 * 1.38 * 9.8) ./ROS_scaler;

% === Section Definitions ===
t_sec1_end = datetime('2026-02-27 02:26:00', 'InputFormat', 'yyyy-MM-dd HH:mm:ss', 'TimeZone', 'UTC');
t_sec2_end = datetime('2026-02-27 02:28:40', 'InputFormat', 'yyyy-MM-dd HH:mm:ss', 'TimeZone', 'UTC');

sections = {
    data.Timestamp < t_sec1_end, ...
    data.Timestamp >= t_sec1_end & data.Timestamp < t_sec2_end, ...
    data.Timestamp >= t_sec2_end
};
section_names = {'Section 1 (y-tilt)', 'Section 2 (x-tilt)', 'Section 3 (45-deg tilt)'};

ref_signals = {data.tilt_yref, data.tilt_xref, data.tilt_xref}; 

% === Processing Parameters & Selectors ===
filter_window = 3; 
ros_upper_limit = 1.0; 
ros_lower_limit = -1.0;

% MANUAL SELECTORS: 1 = Keep, 0 = Discard
% Adjust the lengths and values of these arrays based on the valid moves in your data
% Example assumes roughly 8-9 moves per section for a total of 25
selectors = {
    [0, 1, 1, 1, 0], ...       % Section 1
    [0, 0, 1, 0, 1, 0, 1, 0, 0], ...       % Section 2
    [0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1]         % Section 3
};

% Initialize storage
total_move_count = 0;
stored_results = {}; % Cell array to handle varying-length arrays

% === Move Extraction & Processing ===
for s = 1:3
    fprintf('\n--- %s ---\n', section_names{s});
    
    sec_data = data(sections{s}, :);
    sec_ref = ref_signals{s}(sections{s});
    
    % Find rising edges (>1.0 deg threshold)
    rising_edges = find(diff(sec_ref) > 1.0) + 1; 
    section_valid_peaks = []; % Track valid peak values for the section average
    
    for m = 1:2:(length(rising_edges) - 1)
        move_num = (m + 1) / 2; 
        total_move_count = total_move_count + 1;
        
        % Determine if this move is flagged to be kept
        if move_num <= length(selectors{s})
            keep_move = selectors{s}(move_num);
        else
            keep_move = 1; % Default to keeping it if the selector array is short
        end
        
        start_idx = rising_edges(m);
        end_idx = rising_edges(m+1);
        
        % Extract ROS data matrix for this move
        ROS_matrix = [sec_data.ROS_1(start_idx:end_idx), sec_data.ROS_2(start_idx:end_idx), ...
                      sec_data.ROS_3(start_idx:end_idx), sec_data.ROS_4(start_idx:end_idx)];
        
        % Clip spikes and apply moving average filter
        ROS_matrix = min(max(ROS_matrix, ros_lower_limit), ros_upper_limit);
        filtered_ROS = smoothdata(ROS_matrix, 'movmean', filter_window);
        
        % Extract max ROS array over time (row-wise maximum)
        move_max_ros_array = max(filtered_ROS, [], 2);
        
        % Extract the absolute peak
        peak_ROS = max(move_max_ros_array);
        
        % Add to average calculation and print status based on selector
        if keep_move
            section_valid_peaks = [section_valid_peaks; peak_ROS];
            fprintf('Move %02d (Overall %02d): Peak ROS = %.4f [Logged]\n', move_num, total_move_count, peak_ROS);
        else
            fprintf('Move %02d (Overall %02d): Peak ROS = %.4f [SKIPPED]\n', move_num, total_move_count, peak_ROS);
        end
        
        % Append to cell array storage (Storing EVERYTHING, plus the 'keep_move' flag)
        stored_results(end+1, :) = {total_move_count, s, move_num, keep_move, peak_ROS, move_max_ros_array};
    end
    
    % Calculate and display average peak ROS for kept moves
    if ~isempty(section_valid_peaks)
        avg_max_ros = mean(section_valid_peaks);
        fprintf('>> Average Peak ROS for Kept Moves in %s = %.4f\n', section_names{s}, avg_max_ros);
    else
        fprintf('>> No valid moves kept for %s.\n', section_names{s});
    end
end

% === Final Output Generation & Storage ===
% Convert cell array to a structured table
results_table = cell2table(stored_results, 'VariableNames', {'Overall_Move', 'Section', 'Section_Move', 'Is_Kept', 'Peak_ROS', 'ROS_TimeSeries_Array'});

% Save the table as a .mat file in the same directory
save('data/braking_processed_results.mat', 'results_table');
fprintf('\nData successfully saved to data/braking_processed_results.mat\n');

