function H12_final = computeHomographyRANSAC(CL1uv,CL2uv, Model)
%% computeHomography : estimate the Homography between two images according to Model using RANSAC algorithm
%           Cluv1    set of points on image1 . Each row represents a 2-D point
%                       (u,v). Size: Nx2, with N number of points.
%           Cluv2    set of points on image2 . Each row represents a 2-D point
%                       (u,v). Size: Nx2, with N number of points.
%           Model       type of Homography to estimate. It has to be egual
%                       to one of the following strings: 'Translation',
%                       'Rigid', 'Similarity', 'Affine', 'Projective'.
%   Output
%           H12_final           estimated Homography of Model type. 3x3 matrix.
%
N = 100
for i=1:N
    % random the pairs from images according to model DOF and instantiate
    % the model from those pairs
    [CL1uv_ran,CL2uv_ran] = randomMatch(CL1uv,CL1uv,Model)
    H12 = computeHomography(CL1uv_ran,CL1uv_ran,Model)

    % determine the number of pairs that are within the threshold of the model 
    for i = 1:length(CL2uv)
        err = projectionerrorvec(H12, CL1uv(i,:), CL2uv(i,:))
        d
    end

end

end



    





