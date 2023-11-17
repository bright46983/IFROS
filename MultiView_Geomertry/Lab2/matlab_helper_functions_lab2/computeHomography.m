function H12 = computeHomography(CL1uv,CL2uv, Model)
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
            Q = [];
            B = [];
            for i = 1:size(CL1uv,1)
                p1 = [CL1uv(i,:)';1];
                p2 = [CL2uv(i,:)';1];
            
                Q1 = [p1(1), p1(2), 1, 0, 0, 0, -p1(1)*p2(1), -p1(2)*p2(1)];
                Q2 = [0, 0, 0, p1(1), p1(2), 1, -p1(1)*p2(2), -p1(2)*p2(2)];
            
                B1 = [p2(1)];
                B2 = [p2(2)];
            
                Q(2*i-1,:) = Q1;
                Q(2*i,:)   = Q2;
                B(2*i-1,:) = B1;
                B(2*i,:)   = B2;
            
            end
            A = inv(Q.'*Q)*Q.'*B;
            H12 = [A(1), A(2), A(3), A(4);
                     A(5), A(6), A(7), A(8);
                     A(9), A(10),A(11), 1];

        
        case 'Projective'
            
            % Compute here your 'Projective' homography
            
            H12 = eye(3);              % 3x3 matrix used to store the Homography
            Q = [];
            B = [];
            for i = 1:size(CL1uv,1)
                p1 = [CL1uv(i,:)';1]
                p2 = [CL2uv(i,:)';1]
            
                Q1 = [p1(1), p1(2), 1, 0, 0, 0, -p1(1)*p2(1), -p1(2)*p2(1)]
                Q2 = [0, 0, 0, p1(1), p1(2), 1, -p1(1)*p2(2), -p1(2)*p2(2)]
            
                B1 = [p2(1)];
                B2 = [p2(2)];
            
                Q(2*i-1,:) = Q1;
                Q(2*i,:)   = Q2;
                B(2*i-1,:) = B1;
                B(2*i,:)   = B2;
            
            end
            A = inv(Q.'*Q)*Q.'*B;
            H12 = [A(1), A(2), A(3);
                   A(4), A(5), A(6);
                   A(7), A(8),  1];
                
        
        otherwise
            warning('Invalid model, returning identity homography');
            H12 = eye(3);
            
    end
    
    
end



    





