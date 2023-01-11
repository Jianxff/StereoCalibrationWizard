function [Config,CameraData,StereoData] = importData(xmlpath)

    Config = importConfig(xmlpath);
    cam1 = importCameraData(Config,'primary');
    CameraData = [cam1];
    if Config.dual
        cam2 = importCameraData(Config,'secondary');
        CameraData = [cam1,cam2];
    end
    StereoData = importStereoData(Config);
    
    
function stereo = importStereoData(config)

    raw = importdata([config.storage_path,'/output/stereo.txt'],',');
    r = sscanf(raw{1},'%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c',[1 Inf]);
    stereo.R = reshape(r,[3,3])';

    t = sscanf(raw{2},'%*c%f%*c%f%*c%*c%f%*c',[1 Inf]);
    stereo.T = reshape(t,[1,3])';

    f = sscanf(raw{3},'%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c%f%*c',[1 Inf]);
    stereo.F = reshape(f,[3,3])';