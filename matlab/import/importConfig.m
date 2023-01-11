function config = importConfig(xmlpath)

    xml_info = xmlread(xmlpath);
    config.storage_path = char(xml_info.getElementsByTagName('storage_path').item(0).getTextContent);
    config.storage_path = config.storage_path(2:length(config.storage_path)-1);
    config.board_width = str2double(xml_info.getElementsByTagName('chessboard_width').item(0).getTextContent);
    config.board_height = str2double(xml_info.getElementsByTagName('chessboard_height').item(0).getTextContent);
    config.square_size = str2double(xml_info.getElementsByTagName('chessboard_square_size').item(0).getTextContent);
    
    config.image_width = str2double(xml_info.getElementsByTagName('image_width').item(0).getTextContent);
    config.image_height = str2double(xml_info.getElementsByTagName('image_height').item(0).getTextContent);
    
    config.primary = str2num(xml_info.getElementsByTagName('main_camera_index').item(0).getTextContent);
    config.secondary = str2num(xml_info.getElementsByTagName('second_camera_index').item(0).getTextContent);
    if config.secondary <= -2
        config.dual = false;
    else
        config.dual = true;
    end

    k1_flag = str2double(xml_info.getElementsByTagName('flag_fix_k1').item(0).getTextContent);
    k2_flag = str2double(xml_info.getElementsByTagName('flag_fix_k2').item(0).getTextContent);
    
    fid = fopen([config.storage_path,'/images/count.txt'],'r');
    config.num_frame = fscanf(fid,'%d');
    fclose(fid);

    config.k_num = 2;
    if k2_flag ~= 0
        config.k_num = 1;
    end
    if k1_flag ~= 0
        config.k_num = 0;
    end    
    config.intr_num = config.k_num + 3;

    % 4-corners detection
    Edge = [1, config.board_width,(config.board_height - 1) * config.board_width + 1, config.board_height * config.board_width];
    config.corner = Edge;
    config.corner_neighbor = [Edge(1) + 1, Edge(1) + config.board_width, Edge(1) + config.board_width + 1;
                              Edge(2) - 1, Edge(2) + config.board_width, Edge(2) + config.board_width - 1;
                              Edge(3) + 1, Edge(3) - config.board_width, Edge(3) - config.board_width + 1;
                              Edge(3) - 1, Edge(4) - config.board_width, Edge(4) - config.board_width - 1];

