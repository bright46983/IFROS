function [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(image1, image2, distRatio, drawMatches,useSiftOption)

% [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(image1, image2, distRatio, drawMatches, useSiftMatlabInternal)
%
% This is as a modified version of the original match.m by Lowe.
%   This function reads two images, finds their SIFT features, and creates
%   associations between SIFT features using theri descriptors.
%   A match is accepted only if its distance is less than distRatio times the distance to the
%   second closest match. If drawMatches is true, then it displays lines connecting the matched keypoints.
% It returns two list of matched points, using the uv coordinate convention.
% It also returns the original SIFT locations and descriptors for both
% images, as computed by the SIFT.M function
%
% Example: [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif('scene.pgm','book.pgm',0.6,true);

if nargin < 5
    % The default is to use the internal Matlab SIFT implementation, adapted to be similar to Lowe's
    %     useSiftOption = 'Original';  % Use Lowe's original windows executable SIFTs keypoints
    %     useSiftOption = 'Precomputed';  % Use precomputed SIFTs keypoints
    %     useSiftOption = 'Internal';  % Use internal SIFTs keypoints for each image. ONLY WORKS ON VERSION 2021b or later !

    % Set the default: 
    useSiftOption = 'Precomputed';
    disp(['Using option ' useSiftOption ' in matchsiftmodif.m']);
else
    useSiftPrecomputed = true;
end;

if nargin < 4
    drawMatches = true;
end;

if nargin < 3
    distRatio = 0.6;
end;

switch (useSiftOption)
    case 'Internal'
        % Use internal SIFTs keypoints for each image. ONLY WORKS ON VERSION 2021b or later !
        [im1, des1, loc1] = siftmatlabinternal(image1);
        [im2, des2, loc2] = siftmatlabinternal(image2);

    case 'Precomputed'
        % Use precomputed SIFTs keypoints for each image
        [im1, des1, loc1] = siftprecomputed(image1);
        [im2, des2, loc2] = siftprecomputed(image2);

    case 'Original'
        % Find SIFT keypoints for each image using original Lowe's executable.
        [im1, des1, loc1] = sift(image1);
        [im2, des2, loc2] = sift(image2);

    otherwise
        warning('Invalid SiftOption');
end;


% For efficiency in Matlab, it is cheaper to compute dot products between
%  unit vectors rather than Euclidean distances.  Note that the ratio of
%  angles (acos of dot products of unit vectors) is a close approximation
%  to the ratio of Euclidean distances for small angles.
%
% distRatio: Only keep matches in which the ratio of vector angles from the
%   nearest to second nearest neighbor is less than distRatio.
% distRatio = 0.6;

% For each descriptor in the first image, select its match to second image.
des2t = des2';                          % Precompute matrix transpose
for i = 1 : size(des1,1)
    dotprods = des1(i,:) * des2t;        % Computes vector of dot products
    [vals,indx] = sort(acos(dotprods));  % Take inverse cosine and sort results

    % Check if nearest neighbor has angle less than distRatio times 2nd.
    if (vals(1) < distRatio * vals(2))
        match(i) = indx(1);
    else
        match(i) = 0;
    end
end

% Check if we should draw the matches
if drawMatches
    % Create a new image showing the two images side by side.
    im3 = appendimages(im1,im2);

    % Show a figure with lines joining the accepted matches.
    figure('Position', [100 100 size(im3,2) size(im3,1)]);
    colormap('gray');
    imagesc(im3);
    hold on;
    cols1 = size(im1,2);
    for i = 1: size(des1,1)
        if (match(i) > 0)
            line([loc1(i,2) loc2(match(i),2)+cols1], ...
                [loc1(i,1) loc2(match(i),1)], 'Color', 'c');
        end
    end
    hold off;
    num = sum(match > 0);
    fprintf('Found %d matches.\n', num);
end;


% Create the lists of prospective matched points
numPoints = sum(match>0);
CL1uv = zeros(numPoints,2);
CL2uv = zeros(numPoints,2);
clCounter = 1;

for i = 1:size(des1,1)
    if (match(i) > 0)
        CL1uv(clCounter,:) = [loc1(i,2) loc1(i,1)];
        CL2uv(clCounter,:) = [loc2(match(i),2) loc2(match(i),1)];
        clCounter = clCounter + 1;
    end
end




