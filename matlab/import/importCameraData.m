function cam = importCameraData(config,type)
    addpath(genpath('../tools/'));
    raw = importdata([config.storage_path,'/output/',type,'.txt'],',');
    if strcmp(type,'primary') == 0
        cam.primary = 1;
    else
        cam.primary = 0;
    end
    
    camera_matrix = sscanf(raw{1},'%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c',[1 Inf]);
    camera_matrix = reshape(camera_matrix,[3,3])';
    dist_coeff = sscanf(raw{2},'%*c%f%*c');
    cam.f = camera_matrix(1,1);
    cam.u = camera_matrix(1,3);
    cam.v = camera_matrix(2,3);
    cam.k_num = config.k_num;
    cam.intr_num = config.intr_num;
    cam.k1 = dist_coeff(1); 
    cam.k2 = dist_coeff(2);

    cam.num_frame = config.num_frame;
    for m = 1 : cam.num_frame
        r = sscanf(raw{2 + m},'%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c',[1 Inf]);
        cam.R(:,:,m) = reshape(r,[3,3])';
        cam.T(:,m) = sscanf(raw{2 + cam.num_frame + m},'%*c%f%*c%f%*c%f%*c',[1 Inf])';
        cam.rvec(m,:) = dcm2angle(cam.R(:,:,m));
        cam.tvec(m,:) = cam.T(:,m);
    end
    % cam.params = cameraParameters("IntrinsicMatrix",camera_matrix,"RotationVectors",cam.rvec,"TranslationVectors",cam.tvec);
end
