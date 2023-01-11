function [X_ret,Y_ret] = projectPoints(objects,cdata)

    f = cdata.f;
    u = cdata.u;
    v = cdata.v;

    S1 = objects(1);
    S2 = objects(2);
    S3 = objects(3);

    switch cdata.k_num
        case 0 % no distortion
            x_ = S1/S3;
            y_ = S2/S3;

            X_ret = f * x_ + u; 
            Y_ret = f * y_ + v;
        case 1 % k1 radial distortion
            k1 = cdata.k1;
            r = (1/S3)*sqrt(S1^2 + S2^2);
            x_ = S1/S3;
            y_ = S2/S3;

            X_ret = (1+k1*r^2)*f * x_ + u; 
            Y_ret = (1+k1*r^2)*f * y_ + v;
        case 2 % k1 k2 radial distortion
            k1 = cdata.k1;
            k2 = cdata.k2;
            r = (1/S3)*sqrt(S1^2 + S2^2);
            x_ = S1/S3;
            y_ = S2/S3;

            X_ret = (1 + k1*r^2 + k2*r^4)*f * x_ + u; 
            Y_ret = (1 + k1*r^2 + k2*r^4)*f * y_ + v;
        case 4 % fisheye
            k1 = cdata.k1;
            k2 = cdata.k2;
            k3 = cdata.k3;
            k4 = cdata.k4;
            r = (1/S3)*sqrt(S1^2 + S2^2);
            theta = atan(r);
            theta_d = theta + k1 * theta^3 + k2 * theta^5 + k3 * theta^7 + k4 * theta^9;
            
            X_ret = u + theta_d * f * S1 / (r*S3);
            Y_ret = v + theta_d * f * S2 / (r*S3);

    end
