% 假设你已经有一个名为tb的Table，并且有三列values、g和folder_num
% 如果你的Table变量名不是tb，请替换成实际的变量名

% 找到values列中含有NaN值的行
rows_with_nan = any(isnan(tb.values), 2);

% 删除含有NaN值的行
tb_cleaned = tb(~rows_with_nan, :);

% 将values列转换为float
tb_cleaned.values = double(tb_cleaned.values);

% 将g列转换为int
tb_cleaned.g = int32(tb_cleaned.g);

% 将folder_num列转换为int
tb_cleaned.folder_num = int32(tb_cleaned.folder_num);

% 获取不同的folder_num值
unique_folder_nums = unique(tb_cleaned.folder_num);

% 创建一个cell数组来存储不同folder_num对应的子表
sub_tables = cell(length(unique_folder_nums), 1);
mean_values = zeros(length(unique_folder_nums), 1);
sum_squared_diff = zeros(length(unique_folder_nums), 1);
sum_squared_diff_for_g_sub_tables = zeros(length(unique_folder_nums), 1);
percentage_for_g_sub_tables = zeros(length(unique_folder_nums), 1);
broad_sense_heritability = zeros(length(unique_folder_nums), 1);

% 遍历所有folder_num的可能值
for i = 1:length(unique_folder_nums)
    current_folder_num = unique_folder_nums(i);
    
    % 找出folder_num是current_folder_num的所有行
    sub_table = tb_cleaned(tb_cleaned.folder_num == current_folder_num, :);
    
    % 计算values列的平均值
    mean_values(i) = mean(sub_table.values);
    
    % 计算values值和平均值的差的平方和
    sum_squared_diff(i) = sum((sub_table.values - mean_values(i)).^2);
    
    % 进一步提取子表，每个子表的g不同
    unique_g_values = unique(sub_table.g);
    g_sub_tables_sum_squared_diff = 0;  % 初始化每个folder_num的g_sub_tables平方和
    
    for j = 1:length(unique_g_values)
        current_g_value = unique_g_values(j);
        
        % 提取具有相同g值的子表
        g_sub_table = sub_table(sub_table.g == current_g_value, :);
        
        % 计算g_sub_table中所有values和其均值之差的平方和
        g_sub_tables_sum_squared_diff = g_sub_tables_sum_squared_diff + sum((g_sub_table.values - mean(g_sub_table.values)).^2);
    end
    
    % 存储子表及其g_sub_tables平方和
    sub_tables{i} = struct('folder_num', current_folder_num, 'sub_table', sub_table, 'g_sub_tables_sum_squared_diff', g_sub_tables_sum_squared_diff);
    
    % 存储每个folder_num的g_sub_tables平方和
    sum_squared_diff_for_g_sub_tables(i) = g_sub_tables_sum_squared_diff;
    
    % 计算百分比
    percentage_for_g_sub_tables(i) = g_sub_tables_sum_squared_diff / sum_squared_diff(i) * 100;
    
    % 计算广义遗传力
    broad_sense_heritability(i) = 1 - percentage_for_g_sub_tables(i) / 100;
end

% 输出结果
for i = 1:length(unique_folder_nums)
    disp(['Folder_num: ', num2str(sub_tables{i}.folder_num)]);
    disp(['Mean values: ', num2str(mean_values(i))]);
    disp(['Sum of squared differences: ', num2str(sum_squared_diff(i))]);
    disp(['Sum of squared differences for g_sub_tables: ', num2str(sub_tables{i}.g_sub_tables_sum_squared_diff)]);
    disp(['Percentage for g_sub_tables: ', num2str(percentage_for_g_sub_tables(i)), '%']);
    disp(['Broad sense heritability: ', num2str(broad_sense_heritability(i))]);
    disp('---');
end

% 绘制折线图
figure;
plot(unique_folder_nums, broad_sense_heritability, '-o');
title('Broad Sense Heritability vs. Stages');
xlabel('Stages');
ylabel('Broad Sense Heritability');
grid on;

% 转置
broad_sense_heritability = broad_sense_heritability';