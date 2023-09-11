clear; close all; clc;
%%%
% MATLAB 2022b 之前采用 rigid3d，使用右乘
%%%

%%% 生成点云
num_pw = 200;
z = linspace(0, 2, round(num_pw/2)).';
x = zeros(size(z));
y = 0.05*ones(round(num_pw/2),1);
pw = [[x, y, z];[x, -y, z]];
fprintf('Num of points of model: %i\n', length(pw));
pw_ptc = pointCloud(pw);

%%% 输入点云
ln = [-0.05875661,  0.12769788, -1.03835467];
lf = [-0.04536033, -0.14304574, -1.16413067];
rn = [0.04076717,  0.12842873, -1.04517329];
rf = [0.05255912, -0.14377659, -1.16978621];
ps = [ln;lf;rn;rf];
ps_ptc = pointCloud(ps);

%%% Initial Pose
ini_eul = deg2rad([90, 90, 0]);
ini_rotm = eul2rotm(ini_eul, 'XYZ');
ini_tform = rigid3d(ini_rotm.', zeros(1,3));

%%% real reg
real_eul = [0.11639231, 1.13684745, 1.40421912];
rrotm = eul2rotm(real_eul,'XYZ');
rt = [1, -0.0500000, 2.009999999];
rtform = rigid3d(rrotm.', rt);
ps_real_reg_ptc = pctransform(ps_ptc, rtform);

%%% ICP
[tform,ps_reg_ptc,rmse] = pcregistericp(ps_ptc,pw_ptc, "InitialTransform", ini_tform);

%%% extract eul
est_eul = rotm2eul(tform.Rotation.','XYZ');

%%% print results
fprintf('Est eul angle X Y Z: %f,%f,%f\n',est_eul(1),est_eul(2),est_eul(3));
fprintf('Real eul angle X Y Z: %f,%f,%f\n',real_eul(1),real_eul(2),real_eul(3));
fprintf('Est translation X Y Z: %f,%f,%f\n', tform.Translation(1),tform.Translation(2),tform.Translation(3));
fprintf('Real translation X Y Z: %f,%f,%f\n', rt(1),rt(2),rt(3));

%%% visualization
pcshow(pw_ptc.Location,'r','MarkerSize',100);
hold on;
pcshow(ps_reg_ptc.Location,'g','MarkerSize',200);
pcshow(ps_real_reg_ptc.Location,'y','MarkerSize',200);