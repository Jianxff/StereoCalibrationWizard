function [A_new,C_new,B_new] = JMatNextStereo(corners,R,T,config,cdata,sdata)

    board_width = config.board_width;
    board_height = config.board_height;
    num_intrinsic = config.intr_num;
    CL = cdata(1);
    CR = cdata(2);
    
    % if isempty(R)
    %     x = T;
    %     Rx = [1 0 0;
    %           0 cos(x(1)) -sin(x(1));    
    %           0 sin(x(1)) cos(x(1))];
    %     Ry = [cos(x(2)) 0 sin(x(2));
    %           0 1 0;
    %           -sin(x(2)) 0 cos(x(2))];
    %     Rz = [cos(x(3)) -sin(x(3)) 0;
    %           sin(x(3)) cos(x(3)) 0;
    %           0 0 1];
    %     R = Rz * Ry * Rx; % Rotation matrix in the next frame
    %     T = x(4:6)';
    % end

    %% Build Jacobian for the next pose
    A_new = zeros(4 * board_width * board_height,num_intrinsic * 2); % 2n * k
    C_new = zeros(4 * board_width * board_height, 6);
    B_new = zeros(4 * board_width * board_height, 6); % 2n * 6 <length(x)>

    offset = 2 * board_height * board_width;

    for i = 1 : board_height
        for j = 1 : board_width
            pos = j + (i - 1) * board_width;
            Q = corners(:, j + (i - 1) * board_width);
            Sl = R * Q + T; 
            Sr = sdata.R * Sl + sdata.T;
            dSldr = R * [Q(1)  Q(3) -Q(2);
                        -Q(3)  Q(2)  Q(1);
                        Q(2) -Q(1)  Q(3)];
            dSrdR = sdata.R * [Sl(1)  Sl(3) -Sl(2);
                            -Sl(3)  Sl(2)  Sl(1);
                                Sl(2) -Sl(1)  Sl(3)];
            dSrdr = sdata.R * dSldr;
            switch num_intrinsic
                case 3
                    % intrinsic parameter part
                    Axl = [Sl(1) / Sl(3), 1, 0];
                    Ayl = [Sl(2) / Sl(3), 0, 1];
                    Axr = Axl;
                    Ayr = Ayl;

                    % stereo parameter part
                    Ctx = [CR.f * (1/Sr(3)), 0, -CR.f * (Sr(1)/Sr(3)^2)];  
                    Cty = [0, CR.f * (1/Sr(3)), -CR.f * (Sr(2)/Sr(3)^2)];   
                    
                    % extrinsic parameter part
                    Btxl = [CL.f * (1/Sl(3)), 0, -CL.f * (Sl(1)/Sl(3)^2)];
                    Btyl = [0, CL.f * (1/Sl(3)), -CL.f * (Sl(2)/Sl(3)^2)];
                    Btxr = Ctx;
                    Btyr = Cty;

                case 4
                    % intrinsic parameter part
                    rl = (1 / Sl(3)) * sqrt(Sl(1) ^ 2 + Sl(2) ^ 2);
                    Axl = [(1 + CL.k1 * rl ^ 2) * Sl(1)/Sl(3), 1, 0, rl ^ 2 * CL.f * (Sl(1) / Sl(3))];
                    Ayl = [(1 + CL.k1 * rl ^ 2) * Sl(2)/Sl(3), 0, 1, rl ^ 2 * CL.f * (Sl(2) / Sl(3))];

                    rr = (1 / Sr(3)) * sqrt(Sr(1) ^ 2 + Sr(2) ^ 2);
                    Axr = [(1 + CR.k1 * rr ^ 2) * Sr(1)/Sr(3), 1, 0, rr ^ 2 * CR.f * (Sr(1) / Sr(3))];
                    Ayr = [(1 + CR.k1 * rr ^ 2) * Sr(2)/Sr(3), 0, 1, rr ^ 2 * CR.f * (Sr(2) / Sr(3))];

                    % stereo parameter part
                    Ctx = [CR.f / Sr(3) + CR.f * CR.k1 * (3 * Sr(1) ^ 2 + Sr(2) ^ 2) / Sr(3) ^ 3, 2 * CR.f * CR.k1 * Sr(1) * Sr(2) / Sr(3) ^ 3, -CR.f * (Sr(1) / Sr(3) ^ 2) - 3 * CR.f * CR.k1 * CR.k2 * Sr(1) * (Sr(1) ^ 2 + Sr(2) ^ 2) / Sr(3) ^ 4];
                    Cty = [2 * CR.f * CR.k1 * Sr(1) * Sr(2) / Sr(3) ^ 3, CR.f / Sr(3) + CR.f * CR.k1 * (Sr(1) ^ 2 + 3 * Sr(2) ^ 2) / Sr(3) ^ 3, -CR.f * (Sr(2) / Sr(3) ^ 2) - 3 * CR.f * CR.k1 * CR.k2 * Sr(2) * (Sr(1) ^ 2 + Sr(2) ^ 2) / Sr(3) ^ 4];

                    % extrinsic parameter part
                    Btxl = [CL.f / Sl(3) + CL.f * CL.k1 * (3 * Sl(1) ^ 2 + Sl(2) ^ 2) / Sl(3) ^ 3, 2 * CL.f * CL.k1 * Sl(1) * Sl(2) / Sl(3) ^ 3, -CL.f * (Sl(1) / Sl(3) ^ 2) - 3 * CL.f * CL.k1 * Sl(1) * (Sl(1) ^ 2 + Sl(2) ^ 2) / Sl(3) ^ 4];
                    Btyl = [2 * CL.f * CL.k1 * Sl(1) * Sl(2) / Sl(3) ^ 3, CL.f / Sl(3) + CL.f * CL.k1 * (Sl(1) ^ 2 + 3 * Sl(2) ^ 2) / Sl(3) ^ 3, -CL.f * (Sl(2) / Sl(3) ^ 2) - 3 * CL.f * CL.k1 * Sl(2) * (Sl(1) ^ 2 + Sl(2) ^ 2) / Sl(3) ^ 4];
                    Btxr = Ctx;
                    Btyr = Cty;

                case 5
                    % intrinsic parameter part
                    rl = (1 / Sl(3)) * sqrt(Sl(1) ^ 2 + Sl(2) ^ 2);
                    Axl = [(1 + CL.k1 * rl ^ 2 + CL.k2 * rl ^ 4) * Sl(1)/Sl(3), 1, 0, rl ^ 2 * CL.f * (Sl(1) / Sl(3)), rl ^ 4 * CL.f * (Sl(1) / Sl(3))];
                    Ayl = [(1 + CL.k1 * rl ^ 2 + CL.k2 * rl ^ 4) * Sl(2)/Sl(3), 0, 1, rl ^ 2 * CL.f * (Sl(2) / Sl(3)), rl ^ 4 * CL.f * (Sl(2) / Sl(3))];

                    rr = (1 / Sr(3)) * sqrt(Sr(1) ^ 2 + Sr(2) ^ 2);
                    Axr = [(1 + CR.k1 * rr ^ 2 + CR.k2 * rr ^ 4) * Sr(1)/Sr(3), 1, 0, rr ^ 2 * CR.f * (Sr(1) / Sr(3)), rr ^ 4 * CR.f * (Sr(1) / Sr(3))];
                    Ayr = [(1 + CR.k1 * rr ^ 2 + CR.k2 * rr ^ 4) * Sr(2)/Sr(3), 0, 1, rr ^ 2 * CR.f * (Sr(2) / Sr(3)), rr ^ 4 * CR.f * (Sr(2) / Sr(3))];

                    % stereo parameter part 
                    Ctx = [CR.f * (1 + CR.k1 * rr ^ 2 + CR.k2 * rr ^ 4) / Sr(3) + CR.f * Sr(1) ^ 2 * (2 * CR.k1 + 4 * CR.k2 * rr ^ 2) / Sr(3) ^ 3, 2 * CR.f * Sr(1) * Sr(2) * (CR.k1 + 2 * CR.k2 * rr ^ 2) / Sr(3) ^ 3, -2 * Sr(1) * CR.f * rr ^ 2 * (CR.k1 + 2 * CR.k2 * rr ^ 2) / Sr(3) ^ 2 - Sr(1) * CR.f * (1 + CR.k1 * rr ^ 2 + CR.k2 * rr ^ 4) / Sr(3)^2];
                    Cty = [2 * CR.f * Sr(1) * Sr(2) * (CR.k1 + 2 * CR.k2 * rr ^ 2) / Sr(3) ^ 3, CR.f * (1 + CR.k1 * rr ^ 2 + CR.k2 * rr ^ 4) / Sr(3) + CR.f * Sr(2) ^ 2 * (2 * CR.k1 + 4 * CR.k2 * rr ^ 2) / Sr(3) ^ 3, -2 * Sr(2) * CR.f * rr ^ 2 * (CR.k1 + 2 * CR.k2 * rr ^ 2) / Sr(3) ^ 2 - Sr(2) * CR.f * (1 + CR.k1 * rr ^ 2 + CR.k2 * rr ^ 4) / Sr(3)^2]; 
                    
                    
                    % extrinsic parameter part
                    Btxl = [CL.f * (1 + CL.k1 * rl ^ 2 + CL.k2 * rl ^ 4) / Sl(3) + CL.f * Sl(1) ^ 2 * (2 * CL.k1 + 4 * CL.k2 * rl ^ 2) / Sl(3) ^ 3, 2 * CL.f * Sl(1) * Sl(2) * (CL.k1 + 2 * CL.k2 * rl ^ 2) / Sl(3) ^ 3, -2 * Sl(1) * CL.f * rl ^ 2 * (CL.k1 + 2 * CL.k2 * rl ^ 2) / Sl(3) ^ 2 - Sl(1) * CL.f * (1 + CL.k1 * rl ^ 2 + CL.k2 * rl ^ 4) / Sl(3)^2];
                    Btyl = [2 * CL.f * Sl(1) * Sl(2) * (CL.k1 + 2 * CL.k2 * rl ^ 2) / Sl(3) ^ 3, CL.f * (1 + CL.k1 * rl ^ 2 + CL.k2 * rl ^ 4) / Sl(3) + CL.f * Sl(2) ^ 2 * (2 * CL.k1 + 4 * CL.k2 * rl ^ 2) / Sl(3) ^ 3, -2 * Sl(2) * CL.f * rl ^ 2 * (CL.k1 + 2 * CL.k2 * rl ^ 2) / Sl(3) ^ 2 - Sl(2) * CL.f * (1 + CL.k1 * rl ^ 2 + CL.k2 * rl ^ 4) / Sl(3)^2]; 
                    Btxr = Ctx;
                    Btyr = Cty;
            end

            CRx = Ctx * dSrdR;
            CRy = Cty * dSrdR;
            BRxl = Btxl * dSldr;
            BRyl = Btyl * dSldr;
            BRxr = Btxr * dSrdr;
            BRyr = Btyr * dSrdr;

            A_zero = zeros(1,num_intrinsic);

            A_new(pos,:) = [Axl,A_zero];
            A_new(pos + 1,:) = [Ayl,A_zero];
            C_new(pos,:) = zeros(1,6);
            C_new(pos + 1,:) = zeros(1,6);
            B_new(pos, :) = [BRxl,Btxl];
            B_new(pos + 1, :) = [BRyl,Btyl];

            pos = offset + pos;
            A_new(pos,:) = [A_zero,Axr];
            A_new(pos + 1,:) = [A_zero,Ayr];
            C_new(pos,:) = [CRx,Ctx];
            C_new(pos + 1,:) = [CRy,Cty];
            B_new(pos, :) = [BRxr,Btxr];
            B_new(pos + 1, :) = [BRyr,Btyr];
        end
    end
