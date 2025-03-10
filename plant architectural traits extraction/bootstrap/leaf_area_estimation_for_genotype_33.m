close all
clear all
clc

%%
% 文件夹路径
parentFolder = 'E:\Tomato_data\seg_organs';

% 获取所有文件夹名
allFolderNames = {'instances_and_traits_20230315_p1', 'instances_and_traits_20230315_p2', ...
                   'instances_and_traits_20230403_p3', 'instances_and_traits_20230403_p4', ...
                   'instances_and_traits_20230501_p5', 'instances_and_traits_20230501_p6', ...
                   'instances_and_traits_20230501_p7', 'instances_and_traits_20230501_p8', ...
                   'instances_and_traits_20230525_p5', 'instances_and_traits_20230525_p6', ...
                   'instances_and_traits_20230525_p7', 'instances_and_traits_20230525_p8', ...
                   'instances_and_traits_20230628_p5', 'instances_and_traits_20230628_p6', ...
                   'instances_and_traits_20230628_p9', 'instances_and_traits_20230628_p10', ...
                   'instances_and_traits_20230704_p5', 'instances_and_traits_20230704_p6', ...
                   'instances_and_traits_20230705_p5', 'instances_and_traits_20230705_p6'};

% 定义要匹配的值集合
validValues = [1, 4, 5, 7, 9, 11, 13, 15, 17, 19];

% 初始化总容器
allPointCloudCounts = cell(1, length(allFolderNames));

% 循环遍历不同的文件夹
for folderIndex = 1:length(allFolderNames)
    folderName = allFolderNames{folderIndex};
    folderPath = fullfile(parentFolder, folderName);
    
    % 初始化当前文件夹的容器
    pointCloudCounts = zeros(144, 10);
    
    % 循环遍历当前文件夹中的文件
    for M = 1:144
        for N = 1:10
            % 构造文件名
            fileName = sprintf('%d_plant_%d_leaves_pc.pcd', M, N);
            
            % 构造完整路径
            filePath = fullfile(folderPath, fileName);
            
            % 检查文件是否存在
            if exist(filePath, 'file') == 2
                % 读取点云文件
                ptCloud = pcread(filePath);
                
                % 获取点的数量并存储到容器中
                pointCloudCounts(M, N) = ptCloud.Count;
            end
        end
    end
    
    % 将当前文件夹的容器存入总容器中
    allPointCloudCounts{folderIndex} = pointCloudCounts;

    disp([folderName,' finished processing!']);
end

%%
tb = table();
    
times = 3;

for g = 1:33
    % 提取值并按列装入容器
    nonZeroValues = [];
    
    % 循环遍历每个文件夹
    for folderIndex = 1:length(allFolderNames)
        % 获取当前文件夹的点云计数
        currentPointCloudCounts = allPointCloudCounts{folderIndex};
        
        % 提取值并转为单列
        if g > 0 && g <= 11
            % 根据 flag 设置 N 的值
            if ~any(folderIndex == validValues)
                flag = 1;
                N = [2*g-1, 2*g, 47-2*g, 48-2*g];
            else
                flag = 0;
                N = [2*g+1, 2*g+2, 49-2*g, 50-2*g];
            end
            currentValues = currentPointCloudCounts(N, :);
        elseif g > 11 && g <= 22
            % 根据 flag 设置 N 的值
            if ~any(folderIndex == validValues)
                flag = 1;
                N = [2*(g-11)-1+48, 2*(g-11)+48, 47-2*(g-11)+48, 48-2*(g-11)+48];
            else
                flag = 0;
                N = [2*(g-11)+1+48, 2*(g-11)+2+48, 49-2*(g-11)+48, 50-2*(g-11)+48];
            end
            currentValues = currentPointCloudCounts(N, :);
        else
            % 根据 flag 设置 N 的值
            if ~any(folderIndex == validValues)
                flag = 1;
                N = [2*(g-22)-1+96, 2*(g-22)+96, 47-2*(g-22)+96, 48-2*(g-22)+96];
            else
                flag = 0;
                N = [2*(g-22)+1+96, 2*(g-22)+2+96, 49-2*(g-22)+96, 50-2*(g-22)+96];
            end
            currentValues = currentPointCloudCounts(N, :);
        end

        
        currentValues = currentValues(:);
    
        % currentValues = currentPointCloudCounts(:);
        
        % 将当前文件夹的值列添加到总容器中
        nonZeroValues = [nonZeroValues, currentValues];
    end
    
    nonZeroValues(nonZeroValues==0)=NaN;
    
    % 计算每列的均值和标准差
    columnMeans = nanmean(nonZeroValues);
    columnStd = nanstd(nonZeroValues);
    
    % 循环遍历每列
    for col = 1:size(nonZeroValues, 2)
        % 计算上下界限，以一倍标准差为界限
        upperBound = columnMeans(col) + times*columnStd(col);
        lowerBound = columnMeans(col) - times*columnStd(col);
        
        % 将超过界限的数据置为NaN
        nonZeroValues(nonZeroValues(:, col) > upperBound, col) = NaN;
        nonZeroValues(nonZeroValues(:, col) < lowerBound, col) = NaN;
    end
    
    
    valuesContainer = [];
    folderNum = [];
    valuesContainerSize = 0;
    
    for r = 1:40
        for c = 1:6
            if ~isnan(nonZeroValues(r,c))
                valuesContainer = [valuesContainer;nonZeroValues(r,c)];
                folderNum = [folderNum;c];
                valuesContainerSize = valuesContainerSize + 1;
            end
        end
    end
    
    tb_g = table(valuesContainer/20000, ones(valuesContainerSize, 1) * g, ...
        folderNum, 'VariableNames', {'values', 'g', 'folder_num'});
    
    tb = [tb; tb_g];
end

% Display the resulting table
disp(tb);

folderOrder = [1,2,3,4,5,6];
tb.folder_num = categorical(tb.folder_num,folderOrder);

boxchart(tb.folder_num,tb.values,'GroupByColor',tb.g)
ylabel('Total leaf area (m^2)')
legend