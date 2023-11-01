function [ fig1, fig2 ] = show_registration(Im,If,Ir)
%% show_registration : show a figure contaning Im, If and Ir and another figure containing the overlap between If and Ir
    fig1 = figure;
    subplot(1,3,1);
    imshow(Im,[]);
    title('Moving image');
    
    subplot(1,3,2);
    imshow(If,[]);
    title('Fixed image');

    subplot(1,3,3);
    imshow(Ir,[]);
    title('Registered image');
    
    fig2 = figure;
    
    % Show overlape with different colors
    overlay = imfuse(If,Ir,'falsecolor','Scaling','joint','ColorChannels',[1 2 0]);
    
    imshow(overlay);
    
    
end