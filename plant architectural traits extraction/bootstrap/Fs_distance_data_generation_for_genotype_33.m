clear
clc
close all

DataSize = 20;
times = 3;
FolderNum = 4;

% Initialize container for the table
tb = table();

foldersPrefixes = {'instances_and_traits_20230501', 'instances_and_traits_20230525', ...
    'instances_and_traits_20230628', 'instances_and_traits_20230704'};

for g = 1:33  % Loop over different values of g
    % Initialize container for the current g
    valuesContainer = zeros(DataSize * FolderNum, 1);  % Reshape to a single column

    currentFolderNum = [];

    for folderIndex = 1:length(foldersPrefixes)
        currentFolderPrefix = foldersPrefixes{folderIndex};

        currentColumn = [];

        xh_num = 0;

        while numel(currentColumn) < DataSize

            xh_num = xh_num +1;

            if(xh_num > 10000)
                currentColumn = nan(1,DataSize);
                currentFolderNum = [currentFolderNum,folderIndex*ones(1,DataSize)];
                break;
            end

            % 获取以currentFolderPrefix开头的文件夹列表
            folders = dir([currentFolderPrefix '*']);

            % 随机选择一个文件夹
            selectedFolder = folders(randi(length(folders))).name;

            % 构建文件夹路径
            folderPath = fullfile(pwd, selectedFolder);

            % 在 while 循环内的 selectedFolder 之后添加
            lastDigit = str2double(selectedFolder(end));  % 获取文件夹名称的最后一个数字
            
            if g > 0 && g <= 11
                % 根据 flag 设置 N 的值
                if ~ismember(lastDigit, [1, 4, 5, 7, 9])
                    flag = 1;
                    N = [2*g-1, 2*g, 47-2*g, 48-2*g];
                else
                    flag = 0;
                    N = [2*g+1, 2*g+2, 49-2*g, 50-2*g];
                end
            elseif g > 11 && g <= 22
                % 根据 flag 设置 N 的值
                if ~ismember(lastDigit, [1, 4, 5, 7, 9])
                    flag = 1;
                    N = [2*(g-11)-1+48, 2*(g-11)+48, 47-2*(g-11)+48, 48-2*(g-11)+48];
                else
                    flag = 0;
                    N = [2*(g-11)+1+48, 2*(g-11)+2+48, 49-2*(g-11)+48, 50-2*(g-11)+48];
                end
            else
                % 根据 flag 设置 N 的值
                if ~ismember(lastDigit, [1, 4, 5, 7, 9])
                    flag = 1;
                    N = [2*(g-22)-1+96, 2*(g-22)+96, 47-2*(g-22)+96, 48-2*(g-22)+96];
                else
                    flag = 0;
                    N = [2*(g-22)+1+96, 2*(g-22)+2+96, 49-2*(g-22)+96, 50-2*(g-22)+96];
                end
            end
            
            % 初始化 files 结构体数组
            files = struct('name', {});
            
            % 构建文件路径，并读取CSV文件
            for i = 1:4
                currentFile = sprintf('%d_filtered_traits_no_empty_v3.csv', N(i));
                filePath = fullfile(folderPath, currentFile);
            
                % 保存到结构体数组
                files(i).name = currentFile;
            end

            if ~isempty(files)
                % 随机选择一个csv文件
                selectedFile = files(randi(length(files))).name;

                % 构建文件路径
                filePath = fullfile(folderPath, selectedFile);

                % 读取csv文件
                if exist(filePath, 'file')
                    data = csvread(filePath);
                else
                    continue;
                end

                % 随机选择一个第一列为0的行
                rowsWith3 = find(data(:, 1) == 0);
                if ~isempty(rowsWith3)
                    selectedRow = rowsWith3(randi(length(rowsWith3)));

                    % 将该行第三列及以后的所有非零且非空值的平均值放在容器中的当前列
                    nonZeroValues = data(selectedRow, 3:end);  % 获取第三列及以后的所有值
                    nonZeroValues = nonZeroValues(nonZeroValues ~= 0);  % 去除零值
                    if ~isempty(nonZeroValues)
                        averageValue = mean(nonZeroValues);  % 计算平均值
                        currentColumn = [currentColumn, averageValue];
                        currentFolderNum = [currentFolderNum,folderIndex];
                    end
                end
            end
        end

        % Store valuesContainer for the current g and folderIndex
        valuesContainer((folderIndex - 1) * DataSize + 1 : folderIndex * DataSize) = filterOutliers(currentColumn',times)';

    end

    % Create a table for the current g
    tb_g = table(valuesContainer, ones(DataSize * FolderNum, 1) * g, ...
        currentFolderNum', 'VariableNames', {'values', 'g', 'folder_num'});

    tb = [tb; tb_g];  % Concatenate tables for different values of 'g'
end

% Display the resulting table
disp(tb);

folderOrder = [1,2,3,4];
tb.folder_num = categorical(tb.folder_num,folderOrder);

boxchart(tb.folder_num,tb.values,'GroupByColor',tb.g)
ylabel('Fruit spacing (m)')
legend

function filteredData = filterOutliers(data,times)
    % Remove values beyond 1 standard deviation
    meanValue = mean(data);
    stdValue = std(data);
    
    % Replace values beyond 1 standard deviation with NaN
    data(data < meanValue - times*stdValue | data > meanValue + times*stdValue) = NaN;

    % Do not Filter out NaN values
    filteredData = data;
end