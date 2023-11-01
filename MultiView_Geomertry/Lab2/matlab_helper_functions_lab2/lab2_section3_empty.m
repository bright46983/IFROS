% Helper script for lab 2 of MVG section 3
% You have to complete where needed.
% This is helper file to help you get started. You may use it or not. If
% you use it, then understand the code well before asking questions.


clear all;
close all;
clc;

% example pair 1
image1filename = 'imgl01311.jpg';
image2filename = 'imgl01396.jpg';
H12 = [0.923963035362317 0.105144470821343 -41.6461418047369;-0.105144470821343 0.923963035362317 -224.712186142836;0 0 1];

% example pair 2 - Uncomment to use
% image1filename = 'IMG_1253_small.JPG';
% image2filename = 'IMG_1254_small.JPG';
% H12 = [0.991966 0.135529 -85.4565;-0.135529 0.991966 -372.3004;0 0 1];

I1 = imread(image1filename);
I2 = imread(image2filename);

% Do feature association with the modified match.m function
distRatio = 0.8;
drawMatches = true;
[CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(image1filename, image2filename, distRatio, drawMatches);


% This is an alternative way of showing the associations, using a toolbox function
figure; showMatchedFeatures(I1,I2,CL1uv,CL2uv,'Montage');



drawMatches = false;
distThreshold = 50;

% Initialize stacks for plotting. 
% This can be implemented more elegantly using matlab's table structure
distRatioStack = [];
avgRepErrorStack = [];
maxRepErrorStack = [];
numPointsStack = [];
numPointsOverThresholdStack = []; 

for distRatio = 0.4:0.05:0.9
    [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(image1filename, image2filename, distRatio, drawMatches);
       
    errorVec = projectionerrorvec(H12,CL1uv,CL2uv);
    
    
    if isempty(errorVec)
        disp('No features were associated');
    else
        distRatioStack = [distRatioStack; distRatio];
        avgRepErrorStack = [avgRepErrorStack; mean(errorVec)];
        maxRepErrorStack = [maxRepErrorStack; max(errorVec)];
        numPointsStack = [numPointsStack; size(CL1uv,1)];
        numPointsOverThresholdStack = [numPointsOverThresholdStack; sum(errorVec > distThreshold)];
    end;
    
end;


% Draw two versions of the same plot, one with linear y-axis and the other
% with log y-axis, to better see the small vs large values
figure;
subplot(1,2,1); 
plot(distRatioStack,avgRepErrorStack,distRatioStack,maxRepErrorStack,distRatioStack,numPointsStack,distRatioStack,numPointsOverThresholdStack);
xlabel('Distance Ratio');
ylabel('Error and Num Matches (linear scale)');
legend('Avg Rep Error','Max Rep Error','Num Matches','Num matches > dist threshold');

subplot(1,2,2); 
semilogy(distRatioStack,avgRepErrorStack,distRatioStack,maxRepErrorStack,distRatioStack,numPointsStack,distRatioStack,numPointsOverThresholdStack);
xlabel('Distance Ratio');
ylabel('Error and Num Matches (log scale)');
legend('Avg Rep Error','Max Rep Error','Num Matches','Num matches > dist threshold');


