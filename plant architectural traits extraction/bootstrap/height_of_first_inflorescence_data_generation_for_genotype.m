clear
clc
close all

DataSize = 50;
times = 1;
FolderNum = 5;

% Initialize container for the table
tb = table();

foldersPrefixes = {'instances_and_traits_20230315', 'instances_and_traits_20230403', ...
    'instances_and_traits_20230501', 'instances_and_traits_20230525', ...
    'instances_and_traits_20230628'};

for g = 1:3  % Loop over different values of g
    % Initialize container for the current g
    valuesContainer = zeros(DataSize * FolderNum, 1);  % Reshape to a single column

    currentFolderNum = [];

    for folderIndex = 1:length(foldersPrefixes)
        currentFolderPrefix = foldersPrefixes{folderIndex};

        currentColumn = [];

        while numel(currentColumn) < DataSize
            % 获取以currentFolderPrefix开头的文件夹列表
            folders = dir([currentFolderPrefix '*']);

            % 计算g对应的文件范围
            filesRangeStart = (g - 1) * 48 + 1;
            filesRangeEnd = g * 48;

            % 随机选择一个文件夹
            selectedFolder = folders(randi(length(folders))).name;

            % 构建文件夹路径
            folderPath = fullfile(pwd, selectedFolder);

            % 获取以_filtered_traits_no_empty_v3结尾的csv文件列表
            files = dir(fullfile(folderPath, '*_filtered_traits_no_empty_v3.csv'));

            % 限制文件选择范围
            files = files(filesRangeStart:min(filesRangeEnd, length(files)));

            if ~isempty(files)
                % 随机选择一个csv文件
                selectedFile = files(randi(length(files))).name;

                % 构建文件路径
                filePath = fullfile(folderPath, selectedFile);

                % 读取csv文件
                data = csvread(filePath);

                % 随机选择一个第一列为0的行
                rowsWith3 = find(data(:, 1) == 4);
                if ~isempty(rowsWith3)
                    selectedRow = rowsWith3(randi(length(rowsWith3)));

                    % 将该行第三列的值存放在容器中的当前列
                    currentColumn = [currentColumn, data(selectedRow, 3)];

                    currentFolderNum = [currentFolderNum,folderIndex];
                end
            end
        end

        % Store valuesContainer for the current g and folderIndex
        valuesContainer((folderIndex - 1) * DataSize + 1 : folderIndex * DataSize) = filterOutliers(currentColumn',times)';

    end

    % % Create a table for the current g
    % tb_g = table(valuesContainer, ones(DataSize * FolderNum, 1) * g, ...
    %     repmat((1:FolderNum)', DataSize, 1), 'VariableNames', {'values', 'g', 'folder_num'});

    % Create a table for the current g
    tb_g = table(valuesContainer, ones(DataSize * FolderNum, 1) * g, ...
        currentFolderNum', 'VariableNames', {'values', 'g', 'folder_num'});

    tb = [tb; tb_g];  % Concatenate tables for different values of 'g'
end

% Display the resulting table
disp(tb);

folderOrder = [1,2,3,4,5];
tb.folder_num = categorical(tb.folder_num,folderOrder);

boxchart(tb.folder_num,tb.values,'GroupByColor',tb.g)
ylabel('Inflorescence height (m)')
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