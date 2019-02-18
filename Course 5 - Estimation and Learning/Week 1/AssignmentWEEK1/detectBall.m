% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
% thre = 

data = load('sample_data');
Samples = data.Samples;
D = 2;
[mu, sig] = MLE(Samples(:,1:D));
thre = 0.9;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 

HSV = rgb2hsv(I);
H = HSV(:,:,1);
S = HSV(:,:,2);
V = HSV(:,:,3);
segI = zeros(size(HSV,1),size(HSV,2));
for i=1:size(HSV,1)
    for j=1:size(HSV,2)
        input = [H(i,j),S(i,j)];
        prob = gaussian(input, mu,sig);
        if prob > thre
            segI(i,j) = 1;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

% create new empty binary image
bw_biggest = false(size(segI));

CC = bwconncomp(segI);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
%figure,imshow(bw_biggest);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
%plot(loc(1), loc(2),'r+');


end
