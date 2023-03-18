clc
close all;
clear;
addpath(genpath('.'));

xml_path = '../config.xml';

dist_border = 60;           % smallest distance between the next pose and border (in pixel)
dist_neighbor = 20;         % smallest distance of neighboring corner points (in pixel)
tranlation_bound = 600;     % bound of translation in optimization, may need to change for different cameras
%cost functon control
use_pso = 1;                % 1: use pso() function  0: use sa() function 
fast_mode = 1;              % fast calculation in cost function
fval_threshold = 5;         % threshold of fval
round_limit = 800;          % iteration round limit
[config,cdata,sdata] = importData(xml_path);

% try
%    [config,cdata,sdata] = importData(xml_path);
%catch
%    fprintf(1,'data files not found or broken\n');
%    return
%end
config.dist_border = dist_border;
config.dist_neighbor = dist_neighbor;
[corners,cdata] = buildPoints(config, cdata, sdata);
config = buildMatrix(corners,config,cdata,sdata);
fprintf(1,'initialize compelete.\n');

% Set initial extrinsic parameters
x = [0, 0, 0, mean(cdata(1).T(1,:)),mean(cdata(1).T(2,:)),mean(cdata(1).T(3,:))];

% Global optimization method
% lb = [-pi/3, -pi/3, -pi/2, -tranlation_bound, -tranlation_bound, 0]; % lower bound
% ub = [pi/3, pi/3, pi/2, tranlation_bound, tranlation_bound, mean(LEFT.T(3,:))]; % upper bound

lb = [-pi, -pi, -pi, -tranlation_bound, -tranlation_bound, 0.8*min(cdata(1).T(3,:))]; % lower bound
ub = [pi, pi, pi, tranlation_bound, tranlation_bound, mean(cdata(1).T(3,:))]; % upper bound

sta= tic;
min_x = x;
min_fval = realmax;

% fval_threshold = fval_threshold - (config.num_frame - 3) * 1.2;
round_limit = round_limit - (config.num_frame - 3) * 100;
val_count_limit = 10 - (config.num_frame - 3);
if val_count_limit < 3.5
    val_count_limit = 3.5;
end
if fval_threshold < 3
    fval_threshold = 3;
end

if round_limit < 500
    round_limit = 500;
end

if use_pso
    fprintf(1,'using pso\n');
else
    fprintf(1,'using sa\n');
end

round = 0;
fval = 0;
val_count = 0;
if use_pso
    options = psooptimset('Display','off','Generations',125,'ConstrBoundary','soft')
end
while round < round_limit && min_fval >= fval_threshold && val_count < val_count_limit
    if use_pso
        % [x,fval,exitflag,output] = particleswarm(@(x)cost_function(x,corners,config, LEFT,RIGHT,STEREO),6,lb,ub,optimoptions('particleswarm', 'Display', 'off'));
        [x,fval,exitflag,output] = pso(@(x)costFunction(x,corners,config,cdata,sdata),6,[],[],[],[],lb,ub,[],options);
    else
        [x,fval,exitflag,output] = simulannealbnd(@(x)costFunction(x,corners,config, cdata,sdata),x,lb,ub, saoptimset('Display', 'off'));
    end
    round = round + 1;
    if fval < min_fval
        min_fval = fval;
        min_x = x;
    end
    if fval < realmax
        val_count = val_count + 1;
        fprintf(1,'[R-%d] fval: %.4f\n',round,fval);
        if fast_mode == 1
            break;
        end
    end
end
x = min_x;
fval = min_fval;
cost_time = toc(sta);

if fval > realmax/2
    fprintf(1,'[abort] TLE\n');
    return;
else
    fprintf(1,'[fval] %.4f\n',fval);
    fprintf(1,'[rotation] x:%.1f  y:%.1f  z:%.1f\n',x(1)*180/pi,x(2)*180/pi,x(3)*180/pi);
    fprintf(1,'[translation] %.3f  %.3f  %.3f\n',x(4),x(5),x(6));
end
fprintf(1,'[round] %d\n[time] %.6f\n',round,cost_time);

[Next,NextR] = nextPoints(x, corners, config,cdata,sdata);

plotNext( config, Next,'primary');
writePoints(config, Next, [config.storage_path,'/next/primary.txt'])
if length(cdata) == 2
    plotNext(config, NextR, 'secondary');
    writePoints(config, NextR, [config.storage_path,'/next/secondary.txt'])
end
