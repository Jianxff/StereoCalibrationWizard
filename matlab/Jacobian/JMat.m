function [A,B] = JMat(corners,config,cdata)
    board_width = config.board_width;
    board_height = config.board_height;
    num_frame = config.num_frame;
    CD = cdata(1);
    
    num_intrinsic = CD.intr_num;
    A = zeros(2 * board_width * board_height * num_frame,num_intrinsic); % (2n * m) * k
    B = zeros(2 * board_width * board_height * num_frame,6 * num_frame); % (2n * m) * 6m
    
    
    
    for m = 1 : num_frame
        for i = 1 : board_height
            for j = 1 : board_width
                pos = (m - 1) * 2 * board_width * board_height + 2 * (j + (i - 1) * board_width - 1) + 1;
                Q = corners(:, j + (i - 1) * board_width);
                S = CD.R(:,:,m) * Q + CD.T(:,m); 
                r = (1 / S(3)) * sqrt(S(1) ^ 2 + S(2) ^ 2);
                
                dSdr = CD.R(:,:,m) * [Q(1)  Q(3) -Q(2);
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
                        % % intrinsic parameter part
                        % r = (1 / S(3)) * sqrt(S(1) ^ 2 + S(2) ^ 2);
                        % Ax = [(1 + CD.k1 * r ^ 2 + CD.k2 * r ^ 4) * S(1)/S(3), 1, 0, r ^ 2 * CD.f * (S(1) / S(3)), r ^ 4 * CD.f * (S(1) / S(3))];
                        % Ay = [(1 + CD.k1 * r ^ 2 + CD.k2 * r ^ 4) * S(2)/S(3), 0, 1, r ^ 2 * CD.f * (S(2) / S(3)), r ^ 4 * CD.f * (S(2) / S(3))];

                        % % extrinsic parameter part
                        % Btx = [CD.f * (1 + CD.k1 * r ^ 2 + CD.k2 * r ^ 4) / S(3) + CD.f * S(1) ^ 2 * (2 * CD.k1 + 4 * CD.k2 * r ^ 2) / S(3) ^ 3, 2 * CD.f * S(1) * S(2) * (CD.k1 + 2 * CD.k2 * r ^ 2) / S(3) ^ 3, -2 * S(1) * CD.f * r ^ 2 * (CD.k1 + 2 * CD.k2 * r ^ 2) / S(3) ^ 2 - S(1) * CD.f * (1 + CD.k1 * r ^ 2 + CD.k2 * r ^ 4) / S(3)^2];
                        % Bty = [2 * CD.f * S(1) * S(2) * (CD.k1 + 2 * CD.k2 * r ^ 2) / S(3) ^ 3, CD.f * (1 + CD.k1 * r ^ 2 + CD.k2 * r ^ 4) / S(3) + CD.f * S(2) ^ 2 * (2 * CD.k1 + 4 * CD.k2 * r ^ 2) / S(3) ^ 3, -2 * S(2) * CD.f * r ^ 2 * (CD.k1 + 2 * CD.k2 * r ^ 2) / S(3) ^ 2 - S(2) * CD.f * (1 + CD.k1 * r ^ 2 + CD.k2 * r ^ 4) / S(3)^2]; 
                        Ate = 1 + CD.k1 * r ^ 2 + CD.k2 * r ^ 4;
                        Bte = CD.k1 + 2 * CD.k2 * r ^ 2;
                        Ax = [Ate * S(1)/S(3), 1, 0, r ^ 2 * CD.f * (S(1) / S(3)), r ^ 4 * CD.f * (S(1) / S(3))];
                        Ay = [Ate * S(2)/S(3), 0, 1, r ^ 2 * CD.f * (S(2) / S(3)), r ^ 4 * CD.f * (S(2) / S(3))];

                        % extrinsic parameter part
                        Btx = [CD.f * Ate / S(3) + CD.f * S(1) ^ 2 * (2 * CD.k1 + 4 * CD.k2 * r ^ 2) / S(3) ^ 3, 2 * CD.f * S(1) * S(2) * Bte / S(3) ^ 3, -2 * S(1) * CD.f * r ^ 2 * Bte / S(3) ^ 2 - S(1) * CD.f * Ate / S(3)^2];
                        Bty = [2 * CD.f * S(1) * S(2) * Bte/ S(3) ^ 3, CD.f * Ate / S(3) + CD.f * S(2) ^ 2 * (2 * CD.k1 + 4 * CD.k2 * r ^ 2) / S(3) ^ 3, -2 * S(2) * CD.f * r ^ 2 * Bte / S(3) ^ 2 - S(2) * CD.f * Ate / S(3)^2]; 

                end

                BRx = Btx * dSdr;
                BRy = Bty * dSdr;
    
                A(pos,:) = Ax;
                A(pos + 1,:) = Ay;
                B(pos, 6 * (m - 1) + 1: 6 * m) = [BRx,Btx];
                B(pos + 1, 6 * (m - 1) + 1: 6 * m) = [BRy,Bty];
            end
        end
    end