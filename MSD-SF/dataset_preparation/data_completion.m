clear all
clc

%% 用户指定的绝对路径、数字 N 和补全图片的大小
path = 'E:\Tomato_data\dataset_preparation\Dataset_all\LabelFolder'; % 替换为你的绝对路径
N = 2880; % 替换为用户指定的数字
imageSize = [720, 1280]; % 替换为用户指定的大小

% 遍历指定路径下的所有 PNG 图片
files = dir(fullfile(path, '*.png'));

% 提取所有文件的文件名
fileNames = {files.name};

% 提取文件名中的数字部分
fileNumbers = cellfun(@(x) str2double(regexp(x, '\d+', 'match', 'once')), fileNames);

% 计算缺失的图片编号
missingNumbers = setdiff(1:N, fileNumbers);

% 补全缺失的图片
for i = 1:length(missingNumbers)
    missingNumber = missingNumbers(i);
    newFileName = fullfile(path, [num2str(missingNumber) '.png']);
    
    % 生成全0的图片
    newImage = zeros(imageSize);

    % 保存图片
    imwrite(newImage, newFileName);

    fprintf('生成缺失的图片：%s\n', newFileName);
end

%% 复制现有的图片到另一个指定路径
destinationPath = 'E:\Tomato_data\dataset_preparation\Dataset_medium\label_others_v1'; % 修改为新的路径

for i = 1:N
    sourceFileName = fullfile(path, [num2str(i) '.png']);
    destinationFileName = fullfile(destinationPath, [num2str(i) '.png']);
    
    % 复制文件
    copyfile(sourceFileName, destinationFileName);
    
    % fprintf('复制图片：%s 到 %s\n', sourceFileName, destinationFileName);
end

fprintf('图片复制完成！\n');
