function plotUncertaintyMap(x,corners,config,Pl,Pr,cdata,sdata,pixel_gap)

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
T = x(4:6)';

config = buildMatrix(corners,config,cdata(1),[]);
sigma_p = buildSigma(corners,Pl,R,T,config,cdata(1),[]);
plotUncertaintyMap(sigma_p,config,cdata(1),pixel_gap,'primary');

if length(cdata) == 2
    R_s = sdata.R * R;
    T_s = sdata.T + sdata.R * T;

    config = buildMatrix(corners,config,cdata(2),[]);
    sigma_s = buildSigma(corners,Pr,R_s,T_s,config,cdata(2),[]);
    plotUncertaintyMap(sigma_s,config,cdata(2),pixel_gap,'secondary');
end