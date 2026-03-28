%% Extract Data
scriptPath = fileparts(mfilename('fullpath'));
folder = fullfile(scriptPath, 'MoTeC data');

data = load(fullfile(folder, 'compskidpad.mat'));