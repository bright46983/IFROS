function errorVec = projectionerrorvec(H12,CL1uv,CL2uv)
% errorVec = projectionerrorvec(H12,CL1uv,CL2uv)
% 
% Given two list of coordinates (CL1uv and CL2uv) on two images and the
% homography that relates them (H12) this function will compute an error vector
% (errorVec).  This vector will contain the Euclidean distance between each
% point in CL1uv and its corresponding point in CL2uv after applying the
% homography  H12.errorVec = []
    errorVec = []
    for i=1:size(CL1uv,1)
        p1 = [CL1uv(i,:)';1] 
        p1 = p1/ p1(3)
        p2 = [CL2uv(i,:)';1]
        p2 = p2/ p2(3)
        p1_projected = H12 * p2 %homogeneous coordinate for each point 
        p1_projected = p1_projected/p1_projected(3)
        err = p1 - p1_projected
        euc_dis = sqrt(err(1)^2 + err(2)^2)
        errorVec(end+1) = euc_dis
    end 
 