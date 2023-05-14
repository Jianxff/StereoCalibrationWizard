function out = costFunction(par,corners,config,cdata,sdata)

    board_width = config.board_width;
    board_height = config.board_height;
    image_width = config.image_width;
    image_height = config.image_height;
    dist_border = config.dist_border;
    dist_neighbor = config.dist_neighbor;
    % 4-corners detection
    Edge = config.corner;
    Edge_side = config.corner_neighbor;
     
    % Define the rotation matrix and translation vector of next pose
    x = par;
    Rx = [1 0 0;
          0 cos(x(1)) -sin(x(1));    
          0 sin(x(1)) cos(x(1))];
    Ry = [cos(x(2)) 0 sin(x(2));
          0 1 0;
          -sin(x(2)) 0 cos(x(2))];
    Rz = [cos(x(3)) -sin(x(3)) 0;
          sin(x(3)) cos(x(3)) 0;
          0 0 1];
    R = Rz * Ry * Rx; % Rotation matrix in the next frame
    t = x(4:6)';
    
    objects = zeros(3,2);
    P = zeros(2,board_width * board_height,2);
    
    % Verify if all the points fall inside the image plane after projection
    for i = 1 : board_height
        for j = 1 : board_width
            pos = j + (i - 1) * board_width;
            objects(:,1) = R * corners(:,pos) + t;
            [P(1,pos,1),P(2,pos,1)] = projectPoints(objects(:,1),cdata(1));

            if length(cdata) == 2
                objects(:,2) = sdata.R * objects(:,1) + sdata.T;
                [P(1,pos,2),P(2,pos,2)] = projectPoints(objects(:,2),cdata(2));
            end
            
            for k = 1 : length(cdata)
                if P(1,pos,k) < dist_border || P(1,pos,k) > image_width - dist_border || P(2,pos,k) < dist_border || P(2,pos,k) > image_height - dist_border
                    out = realmax;
                    return;
                end
            end
    
            % if length(cdata) == 2
            %     % epi error
            %     EpiLine = sdata.F * ([P(1,pos,1), P(2,pos,1), 1]');
            %     Dist = ([P(1,pos,2), P(2,pos,2), 1] * EpiLine)^2 / (EpiLine(1) ^ 2 + EpiLine(2) ^ 2);
            %     epi_err = epi_err + Dist;
            % end
        end
    end
    
    % avg_epi_err = sqrt(epi_err / (board_height * board_width));
    
    % out_of_range check
    for i = 1 : 4
        for j = 1 : length(cdata)
            % one corner point should not be to close to the three neighbor ones
            for k = 1 : 3
                if distCount(P(:,Edge(i),j),P(:,Edge_side(i,k),j)) < dist_neighbor
                    out = realmax;
                    return;
                end
            end
        end
    end
    
    Sigma = buildSigma(corners,P,R,t,config,cdata,sdata);
    tr = trace(Sigma);

    % relatice_dist count
    min_dist = realmax;
    for i = 1 : length(cdata(1).T)
        new_dist = distCount(t,cdata(1).T(:,i));
        if new_dist < min_dist
            min_dist = new_dist;
        end
    end

    out = tr + 3600 / min_dist.^2;
    % fprintf(1,'tr: %.4f, dist: %.4f, out: %.4f\n',tr,min_dist,out);

    % fprintf(1,'tr: %.4f, epi: %.4f\n',tr,avg_epi_err);
    % if length(cdata) == 2
    %     out = tr * avg_epi_err;
    % else
    %     out = tr;
    % end