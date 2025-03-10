close all
clear all
clc
%%
load('leaf_inclination_angle.mat')

% 定义要匹配的值集合
validValues = [1, 4, 5, 7, 9, 11, 13, 15, 17, 19];

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
    
    for r = 1:(4*4)
        for c = 1:6
            if ~isnan(nonZeroValues(r,c))
                valuesContainer = [valuesContainer;nonZeroValues(r,c)];
                folderNum = [folderNum;c];
                valuesContainerSize = valuesContainerSize + 1;
            end
        end
    end
    
    tb_g = table(valuesContainer, ones(valuesContainerSize, 1) * g, ...
        folderNum, 'VariableNames', {'values', 'g', 'folder_num'});
    
    tb = [tb; tb_g];
end

% Display the resulting table
disp(tb);

folderOrder = [1,2,3,4,5,6];
tb.folder_num = categorical(tb.folder_num,folderOrder);

boxchart(tb.folder_num,tb.values,'GroupByColor',tb.g)
ylabel('Leaf inclination angle (deg)')
legend