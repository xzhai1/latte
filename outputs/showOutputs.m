%% Script for display montage of outputs
fileFolder = fullfile(pwd);
dirOutput = dir(fullfile(fileFolder,'channel*.txt'));
fileNames = {dirOutput.name}';

X = [];
for i = 1:size(fileNames,1)
    [A,delimiterOut] = importdata(fileNames{i});
    X = cat(3, X, A);
end

% Compute width and height
width = 10;
height = ceil(size(X, 3) / width);

figure
for i = 1:size(X,3)
    subplot(height, width, i);
    axis off;
    imagesc(X(:,:,i));
    set(gca,'xtick',[],'ytick',[])
end