% Test for helper script for lab 2 of MVG section 4
clear all;
close all;
clc;

% Read the first pair of synthetic images and the given coordinates of
% matched points. These matched points are error free thus ideal for
% testing
image1filename = 'DataSet01/00.png';
image2filename = 'DataSet01/01.png';
load('DataSet01/Features.mat');
CL1uv = Features(1).xy;
CL2uv = Features(2).xy;

I1 = imread(image1filename);
I2 = imread(image2filename);

% This is a function that you have to populate. 
% Change the name to H12 = computeHomography(CL1uv,CL2uv, Model) after you added your own code inside
Model = 'Projective';
H12 = computeHomographyEmpty(CL1uv,CL2uv, Model);

% This is an alternative way of showing the associations, using a toolbox function
figure; showMatchedFeatures(I1,I2,CL1uv,CL2uv,'Montage');

% This is a fuction that warps image I2 into the frame of image I1 and
% shows the result with red and green colors
figure; showwarpedimages(I1,I2,H12);


return;

% After testing with the synthetic images you can test with the underwater
% images, by replacing the lines at the beginning of this script by the
% following ones. 

% image1filename = 'imgl01311.jpg';
% image2filename = 'imgl01396.jpg';
% H12 = [0.923963035362317 0.105144470821343 -41.6461418047369;-0.105144470821343 0.923963035362317 -224.712186142836;0 0 1];
% distRatio = 0.5; [CL1uv,CL2uv] = matchsiftmodif(image1filename, image2filename, distRatio,false);