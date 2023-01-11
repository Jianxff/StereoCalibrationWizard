function out = traceCount(corners,P,R,T,config,cdata,sdata)

board_width = config.board_width;
board_height = config.board_height;

if length(cdata) == 2
    A = config.A_part;
    C = config.C_part;
    B = config.B_part;

    ACMat = config.ACMat_extend;
    P_l = P(:,:,1);
    P_r = P(:,:,2);

    %% Build Jacobian for the next pose
    [A_new,C_new,B_new] = JMatNextStereo(corners,R,T,config,cdata,sdata);
    A = [A;A_new];
    C = [C;C_new];
    B = [B, zeros(size(B,1),6);zeros(4*board_width*board_height,size(B,2)), B_new];
    J = sparse([A,C,B]);

    ACMat_new_l = singleACMat(P_l, config);
    ACMat_new_r = singleACMat(P_r, config);
    ACMat_new = [ACMat_new_l,zeros(size(ACMat_new_l));zeros(size(ACMat_new_r)),ACMat_new_r];
    ACMat (end-size(ACMat_new,1)+1 : end, end-size(ACMat_new,1)+1 : end) = ACMat_new;
    M = J' * sparse(ACMat)* J;

    num_intr = config.intr_num * 2 + 6;

    U = M(1:num_intr,1:num_intr);
    W = M(1:num_intr,num_intr+1:end);
    V = M(num_intr+1:end,num_intr+1:end); 

    out = trace(inv(U- W *inv(V)*W'));

else
    A = config.A_part;
    B = config.B_part;

    ACMat = config.ACMat_extend;
    P = P(:,:,1);

    %% Build Jacobian for the next pose
    [A_new,B_new] = JMatNext(corners,R,T,config,cdata,sdata);
    A = [A;A_new];
    B = [B, zeros(size(B,1),6);zeros(2*board_width*board_height,size(B,2)), B_new];
    J = sparse([A,B]);

    ACMat_new = singleACMat(P, config);
    ACMat (end-size(ACMat_new,1)+1 : end, end-size(ACMat_new,1)+1 : end) = ACMat_new;
    M = J' * sparse(ACMat)* J;

    num_intr = config.intr_num;

    U = M(1:num_intr,1:num_intr);
    W = M(1:num_intr,num_intr+1:end);
    V = M(num_intr+1:end,num_intr+1:end); 

    out = trace(inv(U- W *inv(V)*W'));

end