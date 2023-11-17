function [CL1uv_s,CL2uv_s] = randomMatch(CL1uv,CL2uv,Model)
    switch (Model)
         case 'Translation'
           s = 2  
           idx1 = randperm(length(CL1uv),s/2)
           idx2 = randperm(length(CL2uv),s/2)
           CL1uv_s = CL1uv(idx1,:)
           CL2uv_s = CL2uv(idx2,:)

            
            
        case 'Similarity'
          s = 4 
          idx1 = randperm(length(CL1uv),s/2)
          idx2 = randperm(length(CL2uv),s/2)
          CL1uv_s = CL1uv(idx1,:)
          CL2uv_s = CL2uv(idx2,:)

            
        case 'Affine'
          s = 6
          idx1 = randperm(length(CL1uv),s/2)
          idx2 = randperm(length(CL2uv),s/2)
          CL1uv_s = CL1uv(idx1,:)
          CL2uv_s = CL2uv(idx2,:)

        
        case 'Projective'
          s= 8
          idx1 = randperm(length(CL1uv),s/2)
          idx2 = randperm(length(CL2uv),s/2)
          CL1uv_s = CL1uv(idx1,:)
          CL2uv_s = CL2uv(idx2,:)
        
        otherwise
            warning('Invalid model, returning the input');
            CL1uv_s = CL1uv
            CL2uv_s = CL2uv
    end
end