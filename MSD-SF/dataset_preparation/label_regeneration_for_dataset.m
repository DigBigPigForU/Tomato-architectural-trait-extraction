close all
clear all
clc

% Specify the base path
base_path = 'E:\Tomato_data\dataset_preparation\Dataset_medium';

% Define the file pattern
file_pattern = '*.png';

% Specify the range of numbers (1 to N) for image processing
N = 100;  % You can adjust this number accordingly

% Create a folder to store the processed images
result_folder = fullfile(base_path, 'label_processed');
if ~exist(result_folder, 'dir')
    mkdir(result_folder);
end

for label_number = 1:N
    % Construct the full file paths
    leaves_file_path = fullfile(base_path, 'label_leaves', sprintf('%d.png', label_number));
    others_file_path = fullfile(base_path, 'label_others', sprintf('%d.png', label_number));
    rgb_file_path = fullfile(base_path, 'RGBFolder', sprintf('%d.png', label_number));
    
    % Read the images
    original_leaves = imread(leaves_file_path);
    original_others = imread(others_file_path);
    original_rgb = imread(rgb_file_path);

    % Your existing processing code here...
    mask = (original_rgb(:,:,1) == 0) & (original_rgb(:,:,2) == 0) & (original_rgb(:,:,3) == 0);
    original_leaves(mask) = 0;
    mask_others = (original_others ~= 0);
    original_leaves(mask_others) = 0;
    result_label = original_leaves + original_others;

    % Define RGB values for each label
    label_colors = [
        0, 0, 0;        % Label 0
        170, 85, 0;     % Label 1
        170, 0, 255;    % Label 2
        255, 170, 0;    % Label 3
        255, 0, 127     % Label 4
    ];

    % Create label_rgb using the specified color mapping
    label_rgb = zeros(size(result_label, 1), size(result_label, 2), 3, 'uint8');
    for label = 0:4
        mask_label = (result_label == label);
        for channel = 1:3
            label_rgb(:, :, channel) = label_rgb(:, :, channel) + uint8(mask_label) * label_colors(label + 1, channel);
        end
    end

    % Save the results to the specified paths
    result_label_path = fullfile(result_folder, sprintf('%d.png', label_number));
    label_rgb_path = fullfile(result_folder, sprintf('%d_label_rgb.png', label_number));

    imwrite(result_label, result_label_path);
    imwrite(label_rgb, label_rgb_path);

    disp(['Result label for image ' num2str(label_number) ' saved to ' result_label_path]);
    disp(['Label RGB for image ' num2str(label_number) ' saved to ' label_rgb_path]);
end
