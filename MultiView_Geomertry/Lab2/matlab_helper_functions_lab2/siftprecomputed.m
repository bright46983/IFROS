function [image, descriptors, locs] = siftprecomputed(imageFile)
% [image, descriptors, locs] = siftprecomputed(imageFile)
%
% Compute or read pre-existing SIFT file
%

% Nuno Gracias Nov 2021

[pathstr,name,ext] = fileparts(imageFile);

if ~isempty(pathstr)
    precomputedSiftFile = [pathstr filesep name '.rawsift'];
else
    precomputedSiftFile = [name '.rawsift'];
end;

if exist(precomputedSiftFile,'file')
    disp(['Reading pre-existing SIFT file: ' precomputedSiftFile]);
    load(precomputedSiftFile,'image','descriptors','locs','-mat');
else
    [image, descriptors, locs] = sift(imageFile);
    disp(['Saving SIFT file: ' precomputedSiftFile]);
    save(precomputedSiftFile,'image','descriptors','locs','-mat');
end;

