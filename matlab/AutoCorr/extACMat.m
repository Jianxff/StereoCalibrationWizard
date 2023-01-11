function ACMat = extACMat(config,cdata)

num_frame = config.num_frame;
% 
board_width = config.board_width;
board_height = config.board_height;

% %% Calculate the image points
% P_l = zeros(2,board_height*board_width, num_frame);
% P_r = zeros(2,board_height*board_width, num_frame);
% for serial = 1 : length(cdata):
%     for m = 1 : num_frame
%         for i = 1 : board_height
%             for j = 1 : board_width
%                 pos = j + (i - 1) * board_width;
%                 cdata(serial).P = 
%                 [P_l(1,pos,m),P_l(2,pos,m)] = project_points(LEFT.S(:,pos,m),LEFT);
%                 [P_r(1,pos,m),P_r(2,pos,m)] = project_points(RIGHT.S(:,pos,m),RIGHT);
%             end
%         end
%     end
% %%
if length(cdata) == 2
    ACMat = zeros(4*board_height*board_width*num_frame, 4*board_height*board_width*num_frame);
    for m = 1 : num_frame
        ACMat_cur_l = singleACMat(cdata(1).P(:,:,m), config);
        ACMat_cur_r = singleACMat(cdata(2).P(:,:,m), config);   
        n = board_height*board_width;
        left =  1 + (m-1)*4*n;
        right = m*4*n - 2 * n;
        ACMat(left : right,...
            left : right) = ACMat_cur_l;
        left = left + 2 * n;
        right = right + 2 * n;
        ACMat(left : right,...
            left : right) = ACMat_cur_r;
    end
    
else
    ACMat = zeros(2*board_height*board_width*num_frame, 2*board_height*board_width*num_frame);
    for m = 1 : num_frame
        ACMat_cur = singleACMat(cdata(1).P(:,:,m), config);
        n = board_height*board_width;
        left =  1 + (m-1)*2*n;
        right = m*2*n;
        ACMat(left : right,...
            left : right) = ACMat_cur;
    end
end
