function [A,C,B] = JMatStereo(corners,config,cdata,sdata)

board_width = config.board_width;
board_height = config.board_height;
num_frame = config.num_frame;
CL = cdata(1);
CR = cdata(2);

num_intrinsic = CL.intr_num;
A = zeros(2 * 2 * board_width * board_height * num_frame,num_intrinsic * 2); % (4n * m) * 2k
C = zeros(2 * 2 * board_width * board_height * num_frame,6); % (4n * m) * 6
B = zeros(2 * 2 * board_width * board_height * num_frame,6 * num_frame); % (4n * m) * 6m

offset = 2 * board_height * board_width;

for m = 1 : num_frame
    for i = 1 : board_height
        for j = 1 : board_width
            pos = (m - 1) * 4 * board_width * board_height + 2 * (j + (i - 1) * board_width - 1) + 1;
            Q = corners(:, j + (i - 1) * board_width);
            Sl = CL.R(:,:,m) * Q + CL.T(:,m); 
            Sr = sdata.R * Sl + sdata.T;
            dSldr = CL.R(:,:,m) * [Q(1)  Q(3) -Q(2);
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
                    rr = (1 / Sr(3)) * sqrt(Sr(1) ^ 2 + Sr(2) ^ 2);

                    Axl = [(1 + CL.k1 * rl ^ 2) * Sl(1)/Sl(3), 1, 0, rl ^ 2 * CL.f * (Sl(1) / Sl(3))];
                    Ayl = [(1 + CL.k1 * rl ^ 2) * Sl(2)/Sl(3), 0, 1, rl ^ 2 * CL.f * (Sl(2) / Sl(3))];

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
                    rr = (1 / Sr(3)) * sqrt(Sr(1) ^ 2 + Sr(2) ^ 2);

                    Ate_1 = (1 + CL.k1 * rl ^ 2 + CL.k2 * rl ^ 4);
                    Ate_2 = (1 + CR.k1 * rr ^ 2 + CR.k2 * rr ^ 4);

                    Axl = [Ate_1 * Sl(1)/Sl(3), 1, 0, rl ^ 2 * CL.f * (Sl(1) / Sl(3)), rl ^ 4 * CL.f * (Sl(1) / Sl(3))];
                    Ayl = [Ate_1 * Sl(2)/Sl(3), 0, 1, rl ^ 2 * CL.f * (Sl(2) / Sl(3)), rl ^ 4 * CL.f * (Sl(2) / Sl(3))];

                    Axr = [Ate_2 * Sr(1)/Sr(3), 1, 0, rr ^ 2 * CR.f * (Sr(1) / Sr(3)), rr ^ 4 * CR.f * (Sr(1) / Sr(3))];
                    Ayr = [Ate_2 * Sr(2)/Sr(3), 0, 1, rr ^ 2 * CR.f * (Sr(2) / Sr(3)), rr ^ 4 * CR.f * (Sr(2) / Sr(3))];

                    Cte_1 = (2 * CR.k1 + 4 * CR.k2 * rr ^ 2);
                    Cte_2 = (CR.k1 + 2 * CR.k2 * rr ^ 2);

                    % stereo parameter part 
                    Ctx = [CR.f * Ate_2 / Sr(3) + CR.f * Sr(1) ^ 2 * Cte_1 / Sr(3) ^ 3, 2 * CR.f * Sr(1) * Sr(2) * Cte_2 / Sr(3) ^ 3, -2 * Sr(1) * CR.f * rr ^ 2 * Cte_2 / Sr(3) ^ 2 - Sr(1) * CR.f * Ate_2 / Sr(3)^2];
                    Cty = [2 * CR.f * Sr(1) * Sr(2) * (CR.k1 + 2 * CR.k2 * rr ^ 2) / Sr(3) ^ 3, CR.f * Ate_2 / Sr(3) + CR.f * Sr(2) ^ 2 * Cte_1 / Sr(3) ^ 3, -2 * Sr(2) * CR.f * rr ^ 2 * (CR.k1 + 2 * CR.k2 * rr ^ 2) / Sr(3) ^ 2 - Sr(2) * CR.f * Ate_2 / Sr(3)^2]; 
                     
                    Bte_1 = (2 * CL.k1 + 4 * CL.k2 * rl ^ 2);
                    Bte_2 = (CL.k1 + 2 * CL.k2 * rl ^ 2);
                    
                    % extrinsic parameter part
                    Btxl = [CL.f * Ate_1 / Sl(3) + CL.f * Sl(1) ^ 2 * Bte_1 / Sl(3) ^ 3, 2 * CL.f * Sl(1) * Sl(2) * Bte_2 / Sl(3) ^ 3, -2 * Sl(1) * CL.f * rl ^ 2 * Bte_2 / Sl(3) ^ 2 - Sl(1) * CL.f * Ate_1 / Sl(3)^2];
                    Btyl = [2 * CL.f * Sl(1) * Sl(2) * Bte_2 / Sl(3) ^ 3, CL.f * Ate_1 / Sl(3) + CL.f * Sl(2) ^ 2 * Bte_1 / Sl(3) ^ 3, -2 * Sl(2) * CL.f * rl ^ 2 * Bte_2 / Sl(3) ^ 2 - Sl(2) * CL.f * Ate_1 / Sl(3)^2]; 
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

            A(pos,:) = [Axl,A_zero];
            A(pos + 1,:) = [Ayl,A_zero];
            C(pos,:) = zeros(1,6);
            C(pos + 1,:) = zeros(1,6);
            B(pos, 6 * (m - 1) + 1: 6 * m) = [BRxl,Btxl];
            B(pos + 1, 6 * (m - 1) + 1: 6 * m) = [BRyl,Btyl];

            pos = offset + pos;
            A(pos,:) = [A_zero,Axr];
            A(pos + 1,:) = [A_zero,Ayr];
            C(pos,:) = [CRx,Ctx];
            C(pos + 1,:) = [CRy,Cty];
            B(pos, 6 * (m - 1) + 1: 6 * m) = [BRxr,Btxr];
            B(pos + 1, 6 * (m - 1) + 1: 6 * m) = [BRyr,Btyr];
        end
    end
end
