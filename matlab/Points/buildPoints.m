function [corners,cdata_new] = buildPoints(config,cdata,sdata)

    board_width = config.board_width;
    board_height = config.board_height;
    square_size = config.square_size;
    num_frame = config.num_frame;
    cdata_new = cdata;

    %% Define 3D points in the world coordinate
    corners = zeros(board_width * board_height, 3);
    for i = 1 : board_height
        for j = 1 : board_width
            corners(j + (i - 1) * board_width, 1) = (j - 1) * square_size;
            corners(j + (i - 1) * board_width, 2) = (i - 1) * square_size;
        end
    end
    corners = corners';

    %% Transform all the 3D points from the world coordinate to the camera coordinate
    % S = R * Q + t
    for serial = 1:length(cdata)
        S = zeros(3, board_width * board_height,num_frame);
        P = zeros(2, board_width * board_height,num_frame);

        for m = 1 : num_frame
            for i = 1 : board_height
                for j = 1 : board_width
                    pos = j + (i - 1) * board_width;
                    
                    if serial == 1
                        S(:,pos,m) = cdata(1).R(:,:,m) * corners(:,pos) + cdata(1).T(:,m);
                    else
                        tmp = cdata(1).R(:,:,m) * corners(:,pos) + cdata(1).T(:,m);
                        S(:,pos,m) = sdata.R * tmp + sdata.T;
                    end

                    [P(1,pos,m),P(2,pos,m)] = projectPoints(S(:,pos,m),cdata(serial));
                end
            end
        end

        cdata_new(serial).S = S;
        cdata_new(serial).P = P;
    end
