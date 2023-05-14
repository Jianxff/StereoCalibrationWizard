function plotUncertaintyMap(Sigma,config,cdata,gap_size,name)

f = cdata.f;
u = cdata.u;
v = cdata.v;

image_height = config.image_height;
image_width = config.image_width;

map = zeros(ceil(image_height/gap_size),ceil(image_width/gap_size));
for i = 1 : gap_size: image_height
    switch config.k_num
        case 0 % No distortion
            for j = 1 : gap_size : image_width
    
                Jt = [(j-u)/f,1,0,0,0,0;
                      (i-v)/f,0,1,0,0,0]; % S13 = (j-u)/f, S23 = (i-v)/f
                uncertain = Jt * Sigma * Jt';
                map(ceil(i/gap_size),ceil(j/gap_size)) = trace(uncertain);
            end
        case 1 % k1
            k1 = cdata.k1;
            for j = 1 : gap_size : image_width
                fun = @(x) (j- u - (1 + k1*x(1)^2+k1*x(2)^2)*f*x(1))^2 + (i - v - (1 + k1*x(1)^2+k1*x(2)^2)*f*x(2))^2;
                x0 = [j-u,i-v]./f;
                x = fminsearch(fun,x0);
                S13 = x(1);
                S23 = x(2);
                
                Jt = [(j-u)/f,1,0,(S13^2+S23^2)*f*S13,0,0,0,0; 
                      (i-v)/f,0,1,(S13^2+S23^2)*f*S23,0,0,0,0];
                uncertain = Jt * Sigma * Jt';
                map(ceil(i/gap_size),ceil(j/gap_size)) = trace(uncertain);
            end
        case 2
            k1 = cdata.k1;
            k2 = cdata.k2;
            for j = 1 : gap_size : image_width
                fun = @(x) (j- u - (1 + k1*(x(1)^2+x(2)^2) + k2*(x(1)^2+x(2)^2)^4)*f*x(1))^2 + (i - v - (1 + k1*(x(1)^2+x(2)^2) + k2*(x(1)^2+x(2)^2)^4)*f*x(2))^2;
                x0 = [j-u,i-v]./f;
                x = fminsearch(fun,x0);
                
                S13 = x(1);
                S23 = x(2);
                
                Jt = [(j-u)/f,1,0,(S13^2+S23^2)*f*S13, (S13^2+S23^2)^2*f*S13,0,0,0,0,0; 
                      (i-v)/f,0,1,(S13^2+S23^2)*f*S23, (S13^2+S23^2)^2*f*S23,0,0,0,0,0];
                uncertain = Jt * Sigma * Jt';
                map(ceil(i/gap_size),ceil(j/gap_size)) = trace(uncertain);
            end    
    end        
end

figure ('name',name); 
map = imresize(map,[image_height,image_width]);
imagesc(map);
c = colorbar;
c.FontSize = 25;
drawnow
axis off
end