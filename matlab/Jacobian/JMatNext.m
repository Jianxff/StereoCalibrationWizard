function [A_new,B_new] = JMatNext(corners,R,T,config,cdata)

    board_width = config.board_width;
    board_height = config.board_height;
    CD = cdata;
    num_intrinsic = CD.intr_num;

    
    %% Build Jacobian for the next pose
    A_new = zeros(2 * board_width * board_height,num_intrinsic); % 2n * k
    B_new = zeros(2 * board_width * board_height, 6); % 2n * 6 <length(x)>
    
    for i = 1 : board_height
        for j = 1 : board_width
            pos = j + (i - 1) * board_width;
            Q = corners(:, j + (i - 1) * board_width);
            S = R * Q + T; 
            dSdr = R * [Q(1)  Q(3) -Q(2);
                       -Q(3)  Q(2)  Q(1);
                        Q(2) -Q(1)  Q(3)];

            switch num_intrinsic
                case 3
                    % intrinsic parameter part
                    Ax = [S(1) / S(3), 1, 0];
                    Ay = [S(2) / S(3), 0, 1]; 
                    
                    % extrinsic parameter part
                    Btx = [CD.f * (1/S(3)), 0, -CD.f * (S(1)/S(3)^2)];
                    Bty = [0, CD.f * (1/S(3)), -CD.f * (S(2)/S(3)^2)];

                case 4
                    % intrinsic parameter part
                    r = (1 / S(3)) * sqrt(S(1) ^ 2 + S(2) ^ 2);
                    Ax = [(1 + CD.k1 * r ^ 2) * S(1)/S(3), 1, 0, r ^ 2 * CD.f * (S(1) / S(3))];
                    Ay = [(1 + CD.k1 * r ^ 2) * S(2)/S(3), 0, 1, r ^ 2 * CD.f * (S(2) / S(3))];

                    % extrinsic parameter part
                    Btx = [CD.f / S(3) + CD.f * CD.k1 * (3 * S(1) ^ 2 + S(2) ^ 2) / S(3) ^ 3, 2 * CD.f * CD.k1 * S(1) * S(2) / S(3) ^ 3, -CD.f * (S(1) / S(3) ^ 2) - 3 * CD.f * CD.k1 * S(1) * (S(1) ^ 2 + S(2) ^ 2) / S(3) ^ 4];
                    Bty = [2 * CD.f * CD.k1 * S(1) * S(2) / S(3) ^ 3, CD.f / S(3) + CD.f * CD.k1 * (S(1) ^ 2 + 3 * S(2) ^ 2) / S(3) ^ 3, -CD.f * (S(2) / S(3) ^ 2) - 3 * CD.f * CD.k1 * S(2) * (S(1) ^ 2 + S(2) ^ 2) / S(3) ^ 4];

                case 5
                    % intrinsic parameter part
                    r = (1 / S(3)) * sqrt(S(1) ^ 2 + S(2) ^ 2);

                    Ate_1 = (1 + CD.k1 * r ^ 2 + CD.k2 * r ^ 4);
                    Ate_2 = (CD.k1 + 2 * CD.k2 * r ^ 2);
                    Ate_3 = (2 * CD.k1 + 4 * CD.k2 * r ^ 2);

                    Ax = [Ate_1 * S(1)/S(3), 1, 0, r ^ 2 * CD.f * (S(1) / S(3)), r ^ 4 * CD.f * (S(1) / S(3))];
                    Ay = [Ate_1 * S(2)/S(3), 0, 1, r ^ 2 * CD.f * (S(2) / S(3)), r ^ 4 * CD.f * (S(2) / S(3))];

                    % extrinsic parameter part
                    Btx = [CD.f * Ate_1 / S(3) + CD.f * S(1) ^ 2 * Ate_3 / S(3) ^ 3, 2 * CD.f * S(1) * S(2) * Ate_2 / S(3) ^ 3, -2 * S(1) * CD.f * r ^ 2 * Ate_2 / S(3) ^ 2 - S(1) * CD.f * Ate_1 / S(3)^2];
                    Bty = [2 * CD.f * S(1) * S(2) * Ate_2 / S(3) ^ 3, CD.f * Ate_1 / S(3) + CD.f * S(2) ^ 2 * Ate_3 / S(3) ^ 3, -2 * S(2) * CD.f * r ^ 2 * Ate_2 / S(3) ^ 2 - S(2) * CD.f * Ate_1 / S(3)^2]; 

            end

            BRx = Btx * dSdr;
            BRy = Bty * dSdr;
    
            A_new(pos,:) = Ax;
            A_new(pos + 1,:) = Ay;
            B_new(pos, :) = [BRx,Btx];
            B_new(pos + 1, :) = [BRy,Bty];
            
        end
    end