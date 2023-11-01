% Helper script for lab 2 of MVG section 2
clear all;
close all;
clc;

% image1filename = 'imgl01366.jpg';
% image2filename = 'imgl01386.jpg';

% image1filename = 'imgl01386.jpg';
% image2filename = 'imgl01451.jpg';
% 
% image1filename = 'imgl01366.jpg';
% image2filename = 'imgl01451.jpg';
% 
image1filename = 'imgl01311.jpg';
image2filename = 'imgl01396.jpg';
% 
% image1filename = 'imgl01311.jpg';
% image2filename = 'imgl01386.jpg';
% 
% image1filename = 'imgl01366.jpg';
% image2filename = 'imgl01396.jpg';
% 
% image1filename = 'IMG_1253_small.JPG';
% image2filename = 'IMG_1254_small.JPG';
% 
image1filename = 'Img00001_small.jpg';
image2filename = 'Img00025_small.jpg';
% 
% image1filename = 'Img00.4:0.05:0.9
    [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(image1filename, image2filename, distRatio, drawMatches);
       
    errorVec = projectionerrorvec(H12,CL1uv,CL2uv);
    
    
    if isempty(errorVec)
        disp('No features were associated');
    else0025_small.jpg';
% image2filename = 'Img00320_small.jpg';

[image1, descriptors1, loc1] = siftprecomputed(image1filename);
showkeys(image1, loc1)

[image2, descriptors2, loc2] = siftprecomputed(image2filename);
showkeys(image2, loc2)


