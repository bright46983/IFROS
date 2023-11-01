function overlay = showwarpedimages(I1,I2,H12)

% Warp the Homography in a projective struct. It's necessary to use imwarp
% function
t_struct = projective2d(H12');


% Apply Homography to the moving image
If = I1;
Im = I2;
% Ir = imwarp(Im,t_struct,'OutputView',imref2d(size(If))); %'OutputView' options ensures that Ir will be of the same size of Im

Rfixed = imref2d(size(If));
[Ir, Rregistered] = imwarp(Im, t_struct); 

% Show overlape with different colors
overlay = imfuse(If,Rfixed,Ir,Rregistered,'falsecolor','Scaling','joint','ColorChannels',[1 2 0]);

imshow(overlay);

end

