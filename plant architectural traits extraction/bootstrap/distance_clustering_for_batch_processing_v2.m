clear
clc
close all

% 文件夹路径
folder_path = 'E:\Tomato_data\seg_organs\p6\';
code_path = 'E:\Tomato_data\seg_organs\';

% 初始化间距
interval_width = 0.005;

% 设置平滑系数，可以根据需要调整
smoothing_factor = 0.1;

% 设置寻峰的参数，可以根据需要调整
min_peak_height = 0.15;
% min_peak_distance = 15;
min_peak_distance = 4.99;

% 遍历1到144的变量
for N = 1:144
    % 构造文件路径
    file_path = fullfile(folder_path, [num2str(N) '_filtered_traits_no_empty.csv']);
    csv_file_path = fullfile(code_path, [num2str(N) '_filtered_traits_no_empty_v2.csv']);
    
    % 检查文件是否存在
    if exist(file_path, 'file') ~= 2
        disp(['File not found: ' file_path]);
        continue;  % 跳过不存在的文件
    end
    
    % 读取CSV文件
    data = csvread(file_path);

    % 初始化容器
    unique_values_in_second_column = unique(data(data(:, 1) == 1, 2));
    cell_array_of_clusters = cell(1, length(unique_values_in_second_column));

    % 遍历每个不同的值
    for j = 1:length(unique_values_in_second_column)
        current_value = unique_values_in_second_column(j);

        % 初始化当前值的容器
        current_cluster_values = [];

        % 遍历每一行
        for i = 1:size(data, 1)
            % 判断第一列为1且第二列的值与当前值相等
            if data(i, 1) == 1 && data(i, 2) == current_value
                % 将该行的第三列及以后的非零值存入当前值的容器
                cluster_values = data(i, 3:end);
                non_zero_values = cluster_values(cluster_values ~= 0);

                % 只将非零值添加到容器
                if ~isempty(non_zero_values)
                    current_cluster_values = [current_cluster_values, non_zero_values];
                end
            end
        end

        % 将当前值的容器存储在cell数组中的相应位置
        cell_array_of_clusters{j} = current_cluster_values;
    end

    % CSV 文件用于写入数据
    fileID = fopen(csv_file_path, 'w');

    % 遍历每个 cell 中的数组
    for j = 1:length(cell_array_of_clusters)
        current_cluster_values = cell_array_of_clusters{j};

        % 获取当前数组的最大值和最小值
        array_min = min(current_cluster_values);
        array_max = max(current_cluster_values);

        % 初始化区间和频数统计容器
        all_intervals = array_min:interval_width:array_max;
        frequency_counts = zeros(1, length(all_intervals) - 1);

        % 将数组中的元素按照间距划分到对应的区间
        if numel(all_intervals) >= 2
            assigned_intervals = discretize(current_cluster_values, all_intervals);
        
            % 统计每个区间的频数
            frequency_counts = histcounts(assigned_intervals, 1:length(all_intervals));
        else
            disp('Warning: Insufficient elements in all_intervals for discretization.');
            frequency_counts = zeros(size(all_intervals) - 1);
        end

        % 画出适度平滑处理后的折线图
        interval_centers = (all_intervals(1:end-1) + all_intervals(2:end)) / 2;
        if numel(interval_centers) > 1
            smoothed_counts = smooth(interval_centers, frequency_counts, smoothing_factor, 'lowess');
        else
            smoothed_counts = interval_centers;
        end

        % 寻找峰值
        if numel(smoothed_counts) >= 3
            [peaks, locs] = findpeaks(smoothed_counts, 'MinPeakHeight', min_peak_height, 'MinPeakDistance', min_peak_distance);
            if numel(peaks) ~= 0
                % 保存峰值横轴值到 CSV 文件
                fprintf(fileID, '1,%d', j); % 写入第一部分数据
            
                % 逐个写入峰值横轴值到不同的单元格
                for k = 1:length(locs)
                    fprintf(fileID, ',%s', num2str(interval_centers(locs(k))));
                end
                fprintf(fileID, '\n'); % 换行                
            end

        else
            disp('Warning: smoothed_counts is empty. Skipping peak detection.');
            peaks = [];
            locs = [];
        end
    end

    % 关闭 CSV 文件
    fclose(fileID);

    % 读取原始数据
    original_data = csvread(file_path);

    % 过滤第一列为1的行
    filtered_data = original_data(original_data(:, 1) ~= 1, :);

    % 追加数据
    fileID = fopen(csv_file_path, 'a');

    % 将非零数据追加到 CSV 文件
    for i = 1:size(filtered_data, 1)
        non_zero_values = filtered_data(i, 3:end) ~= 0;

        % 写入第一部分数据
        fprintf(fileID, '%d,%d', filtered_data(i, 1), filtered_data(i, 2));

        % 逐个写入非零值到不同的单元格
        non_zero_indices = find(non_zero_values);
        for k = 1:length(non_zero_indices)
            fprintf(fileID, ',%s', num2str(filtered_data(i, 2 + non_zero_indices(k))));
        end

        fprintf(fileID, '\n'); % 换行
    end

    % 关闭 CSV 文件
    fclose(fileID);
end
