clc
close all;
clear;
addpath(genpath('.'));

xml_path = '../config.xml';

dist_border = 50;           % smallest distance between the next pose and border (in pixel)
dist_neighbor = 20;         % smallest distance of neighboring corner points (in pixel)
tranlation_bound = 1000;    % bound of translation in optimization, may need to change for different cameras
%cost functon control
use_pso = 1;                % 1: use pso() function  0: use sa() function 
cost_per_round = 100;       % per pso round control
cost_round_control = 1000;   % total pso round control
min_fval_control = 10;     % min_fval control

try
    [config,cdata,sdata] = importData(xml_path);
catch
    fprintf(1,'data files not found or broken\n');
    return
end
    config.dist_border = dist_border;
config.dist_neighbor = dist_neighbor;

[corners,cdata] = buildPoints(config, cdata, sdata);
config = buildMatrix(corners,config,cdata,sdata);

fprintf(1,'initialize compelete.\n');

% Set initial extrinsic parameters
x = [0, 0, 0, mean(cdata(1).T(1,:)),mean(cdata(1).T(2,:)),mean(cdata(1).T(3,:))];
fval = 0;

% Global optimization method
% lb = [-pi/3, -pi/3, -pi/2, -tranlation_bound, -tranlation_bound, 0]; % lower bound
% ub = [pi/3, pi/3, pi/2, tranlation_bound, tranlation_bound, mean(LEFT.T(3,:))]; % upper bound

lb = [-pi, -pi, -pi, -tranlation_bound, -tranlation_bound, 0]; % lower bound
ub = [pi, pi, pi, tranlation_bound, tranlation_bound, mean(cdata(1).T(3,:))]; % upper bound

sta= tic;
min_x = x;
min_fval = realmax;
round = [0,0];
min_limit = min_fval_control - config.num_frame * 0.4;
round_limit = cost_round_control - config.num_frame * 100;
if min_limit < 0.9
    min_limit = 0.9;
end
if round_limit < 500
    round_limit = 500;
end
while round(1) < round_limit && min_fval >= min_limit 
    round(2) = round(2) + 1;
    while round(1) < cost_per_round * round(2)
        if use_pso
            % [x,fval,exitflag,output] = particleswarm(@(x)cost_function(x,corners,config, LEFT,RIGHT,STEREO),6,lb,ub,optimoptions('particleswarm', 'Display', 'off'));
            [x,fval,exitflag,output] = pso(@(x)costFunction(x,corners,config,cdata,sdata),6,[],[],[],[],lb,ub,[],psooptimset('Display','off'));
        else
            [x,fval,exitflag,output] = simulannealbnd(@(x)costFunction(x,corners,config, cdata,sdata),x,lb,ub,saoptimset('Display', 'off'));
        end
        round(1) = round(1) + 1;
        if fval < min_fval
            min_fval = fval;
            min_x = x;
        end
        if fval < realmax
            fprintf(1,'%d: fval: %.4f\n',round(1),fval);
        end
    end
end
x = min_x;
fval = min_fval;
cost_time = toc(sta);

if fval > realmax/2
    fprintf(1,'*** cost funtion time out ***\n');
    return;
else
    fprintf(1,'[fval] %.6f\n',fval);
    fprintf(1,'[rotation] x:%.1f  y:%.1f  z:%.1f\n',x(1)*180/pi,x(2)*180/pi,x(3)*180/pi);
    fprintf(1,'[translation] %.3f  %.3f  %.3f\n',x(4),x(5),x(6));
end
fprintf(1,'[round] %d\n[time] %.6f\n',round(1),cost_time);

[Next,NextR] = nextPoints(x, corners, config,cdata,sdata);

plotNext( config, Next,'primary');
writePoints(config, Next, [config.storage_path,'/next/primary.txt'])
if length(cdata) == 2
    plotNext(config, NextR, 'secondary');
    writePoints(config, NextR, [config.storage_path,'/next/secondary.txt'])
end
