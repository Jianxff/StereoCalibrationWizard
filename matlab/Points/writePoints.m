function writePoints(config, points, path)

    board_width = config.board_width;
    board_height = config.board_height;

    fid = fopen(path,'w');
    for i = 1 : board_height
        for j = 1 : board_width
            pos = j + (i - 1) * board_width;
            fprintf(fid,'%.2f %.2f\n',points(1,pos),points(2,pos));
        end
    end

    fclose(fid);





