clear all
clc

% 设置原始路径和新路径的对应关系文件
mappingFile = 'E:\Tomato_data\dataset_preparation\Dataset_20231027_112455\file_records.csv';

% 读取对应关系文件
mappingData = readtable(mappingFile, 'Delimiter', ',');

% 遍历每一行，进行复制操作
for i = 1:size(mappingData, 1)
    % 获取原始路径和新路径
    originalPath = mappingData{i, 1};
    newPath = mappingData{i, 2};
    
    % 获取文件名
    [~, filename, ext] = fileparts(newPath);
    
    % 构建新的目标路径
    targetPath = fullfile(fileparts(originalPath), ['raw_label', ext]);
    targetColoredPath = fullfile(fileparts(originalPath), ['colored_label', ext]);
    
    % 构建 PredFolder 和 ColoredPredFolder 的路径
    predFolderPath = fullfile(fileparts(fileparts(newPath)), 'PredFolder', [filename, ext]);
    coloredPredFolderPath = fullfile(fileparts(fileparts(newPath)), 'ColoredPredFolder', [filename, ext]);
    
    try
        % 复制 PredFolder 中的文件到原始路径的上级目录
        copyfile(predFolderPath, targetPath);
        
        % 复制 ColoredPredFolder 中的文件到原始路径的上级目录
        copyfile(coloredPredFolderPath, targetColoredPath);
        
        disp(['复制成功: ', targetPath, ' 和 ', targetColoredPath]);
    catch
        disp(['复制失败: ', targetPath, ' 或 ', targetColoredPath]);
    end
end


