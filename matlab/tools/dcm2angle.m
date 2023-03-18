function rvec = dcm2angle(R)
X = 0;
Y = 0;
Z = 0;
Pi = 3.1415926;

if R(3,1) ~= 1 || R(3,1) ~= -1
    Y = -asin(R(3,1));
    % Y2 = Pi - Y1;
    X = atan2(R(3,2) / cos(Y), R(3,3)/cos(Y));
    % X2 = atan2(R(3,2) / cos(Y2), R(3,3)/cos(Y2));
    Z = atan2(R(2,1) / cos(Y), R(1,1)/cos(Y));
    % Z2 = atan2(R(2,1) / cos(Y2), R(1,1)/cos(Y2));
else
    Z = 0;
    if R(3,1) == -1
        Y = Pi/2;
        X = Z + atan2(R(1,2),R(1,3));
    else
        Y = -1 * Pi / 2;
        X = -1 * Z + atan2(-1 * R(1,2),-1*R(1,3));
    end
end
rvec = [X,Y,Z];
end

