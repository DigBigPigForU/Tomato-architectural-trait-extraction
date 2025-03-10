clear all
clc

% 设置目录
mainDir = 'E:\Tomato_data\';
prefix1 = '20230';
prefix2 = 'p';
N = 147;

% 创建数据集文件夹
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
datasetDir = fullfile(mainDir, 'dataset_preparation', ['Dataset_' timestamp]);
mkdir(datasetDir);

% 创建RGB和Depth文件夹
outputRGBDir = fullfile(datasetDir, 'RGBFolder');
outputDepthDir = fullfile(datasetDir, 'ModalXFolder');
labelDir = fullfile(datasetDir, 'LabelFolder');
mkdir(outputRGBDir);
mkdir(outputDepthDir);
mkdir(labelDir);

% 创建CSV文件
csvFilePath = fullfile(datasetDir, 'file_records.csv');
csvFile = fopen(csvFilePath, 'w');
fprintf(csvFile, 'OriginalPath,NewPath\n');

% 找出所有以20230开头的文件夹
dirs1 = dir(fullfile(mainDir, [prefix1 '*']));
dirs1 = dirs1([dirs1.isdir]);  % 仅选择文件夹

fileCounter = 1;

% 遍历每个以20230开头的文件夹
for i = 1:length(dirs1)
    currentDir1 = fullfile(mainDir, dirs1(i).name);
    
    % 找出每个以p开头的子文件夹
    dirs2 = dir(fullfile(currentDir1, [prefix2 '*']));
    dirs2 = dirs2([dirs2.isdir]);  % 仅选择文件夹
    
    % 在每个以p开头的文件夹中选择N个文件夹
    for j = 1:length(dirs2)
        currentDir2 = fullfile(currentDir1, dirs2(j).name);
        
        % 找出当前文件夹下的所有文件夹
        subdirs = dir(fullfile(currentDir2, '*'));
        subdirs = subdirs([subdirs.isdir]);  % 仅选择文件夹
        
        % 如果文件夹数量小于N，就选择全部文件夹
        if N > length(subdirs)
            selectedIndices = 1:length(subdirs);
        else
            % 随机选择N个文件夹编号
            selectedIndices = randperm(length(subdirs), N);
        end
        
        % 对选定的文件夹执行操作
        for k = 1:length(selectedIndices)
            currentSubDir = fullfile(currentDir2, subdirs(selectedIndices(k)).name);
            
            % 处理RGB图片
            rgbFileName = fullfile(currentSubDir, [subdirs(selectedIndices(k)).name '_color_uint8_segmented.png']);
            if exist(rgbFileName, 'file') == 2
                newRGBFileName = fullfile(outputRGBDir, sprintf('%d.png', fileCounter));
                copyfile(rgbFileName, newRGBFileName);
                
                % 写入CSV记录
                fprintf(csvFile, '%s,%s\n', rgbFileName, newRGBFileName);
                
                % 处理深度图片
                depthFileName = fullfile(currentSubDir, [subdirs(selectedIndices(k)).name '_depth_uint16.png']);
                if exist(depthFileName, 'file') == 2
                    newDepthFileName = fullfile(outputDepthDir, sprintf('%d.png', fileCounter));
                    copyfile(depthFileName, newDepthFileName);
                    
                    % 写入CSV记录
                    fprintf(csvFile, '%s,%s\n', depthFileName, newDepthFileName);
                    
                    fileCounter = fileCounter + 1;
                end
            end
        end
    end
end

% 关闭CSV文件
fclose(csvFile);

disp('操作完成。');
