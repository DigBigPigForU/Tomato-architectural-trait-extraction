%% random

clear all
clc

% 获取用户输入
train_count = input('请输入训练集的数量: ');
test_count = input('请输入测试集的数量: ');

% 生成编号范围
total_count = train_count + test_count;
all_numbers = 1:total_count;

% 随机打乱编号顺序
shuffled_numbers = randperm(total_count);

% 分配编号给训练集和测试集
train_numbers = shuffled_numbers(1:train_count);
test_numbers = shuffled_numbers(train_count+1:end);

% 保存为txt文件
dlmwrite('train.txt', train_numbers, 'delimiter', '\n');
dlmwrite('test.txt', test_numbers, 'delimiter', '\n');

disp('训练集编号已保存到train.txt');
disp('测试集编号已保存到test.txt');

%% in order

clear all
clc

% 获取用户输入
train_count = input('请输入训练集的数量: ');
test_count = input('请输入测试集的数量: ');

% 生成编号范围
total_count = train_count + test_count;
all_numbers = 1:total_count;

% 分配编号给训练集和测试集
train_numbers = all_numbers(1:train_count);
test_numbers = all_numbers(train_count+1:end);

% 保存为txt文件
dlmwrite('train.txt', train_numbers, 'delimiter', '\n');
dlmwrite('test.txt', test_numbers, 'delimiter', '\n');

disp('训练集编号已保存到train.txt');
disp('测试集编号已保存到test.txt');
