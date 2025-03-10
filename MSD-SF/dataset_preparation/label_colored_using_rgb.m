clear all
clc

% 定义标签颜色
label_colors = [
    0, 0, 0;        % 标签 0
    170, 85, 0;     % 标签 1
    170, 0, 255;    % 标签 2
    255, 170, 150;    % 标签 3   
    255, 0, 127     % 标签 4
];

% 输入文件夹路径
input_folder = 'E:\Tomato_data\dataset_preparation\Dataset_medium\PredFolder\data_eval_all';

% 输出文件夹路径
output_folder = 'E:\Tomato_data\dataset_preparation\Dataset_medium\PredFolder\data_eval_all\_color';

% 获取输入文件夹中所有的png文件
file_list = dir(fullfile(input_folder, '*.png'));

% 遍历每个文件
for i = 1:length(file_list)
    % 读取标签图片
    label_image = imread(fullfile(input_folder, file_list(i).name));
    
    % 初始化输出图片
    colored_label = zeros(size(label_image, 1), size(label_image, 2), 3, 'uint8');
    
    % 根据标签颜色映射进行颜色替换
    for j = 1:size(label_colors, 1)
        mask = label_image == (j - 1); % 标签值从0开始
        colored_label(:, :, 1) = colored_label(:, :, 1) + uint8(mask) * label_colors(j, 1);
        colored_label(:, :, 2) = colored_label(:, :, 2) + uint8(mask) * label_colors(j, 2);
        colored_label(:, :, 3) = colored_label(:, :, 3) + uint8(mask) * label_colors(j, 3);
    end
    
    % 保存可视化的标签图片到输出文件夹
    [~, base_name, ~] = fileparts(file_list(i).name);
    % output_path = fullfile(output_folder, [base_name, '_visualized.png']);
    output_path = fullfile(output_folder, [base_name, '.png']);
    imwrite(colored_label, output_path);
end

disp('可视化标签生成完成。');
