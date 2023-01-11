function config_new = buildMatrix(corners,config,cdata,sdata)

board_width = config.board_width;
board_height = config.board_height;
config_new = config;

ACMat = extACMat(config,cdata);

if length(cdata) == 2
    [A_part,C_part,B_part] = JMatStereo(corners,config,cdata,sdata);
    ACMat_extend = zeros(size(ACMat) + [4*board_height*board_width,4*board_height*board_width]);
    config_new.A_part = A_part;
    config_new.C_part = C_part;
    config_new.B_part = B_part;

else
    [A_part,B_part] = JMat(corners,config,cdata,sdata);
    ACMat_extend = zeros(size(ACMat) + [2*board_height*board_width,2*board_height*board_width]);
    config_new.A_part = A_part;
    config_new.B_part = B_part;

end


ACMat_extend(1:size(ACMat,1), 1:size(ACMat,2)) = ACMat;  
config_new.ACMat = ACMat;
config_new.ACMat_extend = ACMat_extend;
