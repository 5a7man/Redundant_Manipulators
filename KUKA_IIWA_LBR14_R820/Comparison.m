clc
clear all
close all

%% Deviation from Desired Trajectory
path = [-0.6:0.05:0.6];
actual_path =  sqrt(path.^2+(-0.4)^2+(0)^2);;
plot(actual_path,'LineWidth',2)
hold on
load p_wpi.mat
plot(obtained_p,'LineWidth',2)
disp('WPI Mean Error')
disp(mean((obtained_p-actual_path)))
load p_dls.mat
plot(obtained_p,'LineWidth',2)
disp('DLS Mean Error')
disp(mean((obtained_p-actual_path)))
load p_nsm.mat
plot(obtained_p,'LineWidth',2)

legend('Actual Path','WPI','DLS','NSM')
title('Deviation from Desired Path')


%% Calcualting Mean
disp('WPI Mean Error')
load error_wpi.mat
disp(mean(error))
disp('DLS Mean Error')
load error_dls.mat
disp(mean(error))
disp('NSM Mean Error')
load error_nsm.mat
disp(mean(error))