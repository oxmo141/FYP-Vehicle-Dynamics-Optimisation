%% LATERAL LOAD TRANSFER FROM MOTEC I2PRO DATA

clear;clc
paramR26

% Extract Data
scriptPath = fileparts(mfilename('fullpath'));
folder = fullfile(scriptPath, 'MoTeC data');

data = load(fullfile(folder, 'S1_#2882_20250620_140834.mat'));