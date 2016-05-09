%% Script for display montage of outputs
fileFolder = fullfile(pwd);
dirOutput = dir(fullfile(fileFolder,'channel*.txt'));
fileNames = {dirOutput.name}';

X = [];
for i = 1:length(fileNames)
    [A,delimiterOut] = importdata(fileNames{i});
    X = cat(3, X, A);
end

% % Compute width and height
% width = 10;
% height = ceil(size(X, 3) / width);
% 
% figure
% for i = 1:size(X,3)
%     subplot(height, width, i);
%     axis off;
%     imagesc(X(:,:,i));
%     set(gca,'xtick',[],'ytick',[])
% end
% 
% colormap default;

[~, I] = max(X, [], 3);

% Color maps
Color = cell(21, 1);

Color{1} = [0 0 0];
Color{2} = [255 102 178];
Color{3} = [204 204 0];
Color{4} = [153 0 0];
Color{5} = [0 153 0];
Color{6} = [102 0 204];
Color{7} = [255 51 153];
Color{8} = [0 0 255];
Color{9} = [255 0 0];
Color{10} = [0 255 0];
Color{11} = [0 0 204];
Color{12} = [0 255 255];
Color{13} = [255 255 153];
Color{14} = [0 153 153];
Color{15} = [224 224 224];
Color{16} = [255 0 255];
Color{17} = [102 0 0];
Color{18} = [255 153 51];
Color{19} = [255 153 153];
Color{20} = [102 178 255];
Color{21} = [102 255 102];

Y = zeros([size(X, 1), size(X, 2), 3]);
for k = 1:size(X, 3)
    for j = 1:size(X, 2)
        for i = 1:size(X, 1)
            Y(i, j, :) = reshape(Color{I(i, j)},[1 1 3]);
        end
    end
end

Y = uint8(Y);
% figure;
imwrite(Y, 'cat.png');

% person
% bird, cat, cow, dog, horse, sheep
% aeroplane, bicycle, boat, bus, car, motorbike, train
% bottle, chair, dining table, potted plant, sofa, tv/monitor