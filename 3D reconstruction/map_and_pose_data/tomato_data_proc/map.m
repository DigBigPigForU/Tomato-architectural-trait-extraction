clear all
clc
% PNG_name = 'wenshi0705.png';
% filename = 'wenshi0705point-data.xlsx';
% dataname = 'T.xlsx';
% H = 68.8;
% W = 46.4;
% PNG_name = 'wenshi230315.png';
% filename = 'wenshi230315point-data.xlsx';
% dataname = 'T.xlsx';
% H = 73.6;
% W = 68.8;
PNG_name = 'wenshi0607.png';
filename = 'wenshi0607point-data.xlsx';
dataname = 'T_220607.xlsx';
H = 75.2;
W = 64.0;

GM = imread(PNG_name);
h = size(GM,1);
w = size(GM,2);

[num,txt,raw] = xlsread(filename);
% X = num(4:end,1);
% Y = num(4:end,2);
% A = num(4:end,3);

X = num(3:end,1);
Y = num(3:end,2);
A = num(3:end,3);

% quiver(X,Y,cos(A./180.*pi),sin(A./180.*pi),0.25)
quiver(X./w.*W,Y./h.*H,cos(A./180.*pi),sin(A./180.*pi),0.3)

%%
T = [];
for i = 1:size(num,1)
    % T(:,:,i) = [cos(0.5*pi-A(i)./180.*pi),-sin(0.5*pi-A(i)./180.*pi),0,X(i)./w.*W;
    %             sin(0.5*pi-A(i)./180.*pi),cos(0.5*pi-A(i)./180.*pi),0,Y(i)./h.*H;
    %             0,0,1,0.92;
    %             0,0,0,1];
    T(:,:,i) = [cos(A(i)./180.*pi),-sin(A(i)./180.*pi),0,X(i)./w.*W;
            sin(A(i)./180.*pi),cos(A(i)./180.*pi),0,Y(i)./h.*H;
            0,0,1,0.92;
            0,0,0,1];
end

data = [];
for i = 1:size(num,1)
    data(i,1:4) = T(1,:,i);
    data(i,5:8) = T(2,:,i);
    data(i,9:12) = T(3,:,i);
    data(i,13:16) = T(4,:,i);
end
xlswrite(dataname,data)