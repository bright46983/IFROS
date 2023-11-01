function H12 = computeHomographyEmpty(CL1uv,CL2uv, Model)
%% computeHomography : estimate the Homography between two images according to Model 
%           Cluv1    set of points on image1 . Each row represents a 2-D point
%                       (u,v). Size: Nx2, with N number of points.
%           Cluv2    set of points on image2 . Each row represents a 2-D point
%                       (u,v). Size: Nx2, with N number of points.
%           Model       type of Homography to estimate. It has to be egual
%                       to one of the following strings: 'Translation',
%                       'Rigid', 'Similarity', 'Affine', 'Projective'.
%   Output
%           H12           estimated Homography of Model type. 3x3 matrix.
%


warning('This is an empty function just to help you with the switch command.')

    switch (Model)

        case 'Translation'
            
            % Compute here your 'Translation' homography
            
            H12 = eye(3);              % 3x3 matrix used to store the Homography
            
        case 'Similarity'
            
            % Compute here your 'Similarity' homography
            
            H12 = eye(3);              % 3x3 matrix used to store the Homography

            
        case 'Affine'
            
            % Compute here your 'Affine' homography
            
            H12 = eye(3);              % 3x3 matrix used to store the Homography

        
        case 'Projective'
            
            % Compute here your 'Projective' homography
            
            H12 = eye(3);              % 3x3 matrix used to store the Homography

        
        
        otherwise
            warning('Invalid model, returning identity homography');
            H12 = eye(3);
            
    end
    
    
end





