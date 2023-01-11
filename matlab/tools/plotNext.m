function plotNext(config, points, name)

board_width = config.board_width;
board_height = config.board_height;
image_width = config.image_width;
image_height = config.image_height;

figure ('name',name); 
hold on;
for i = 1 : board_height
    for j = 1 : board_width
        pos = j + (i - 1) * board_width;
        plot(points(1,pos), points(2,pos),'ro');
    end
end
plot(points(1,:), points(2,:),'b-');
xlim([0, image_width]);
ylim([0, image_height]);
set(gca,'Ydir','reverse')
drawnow
hold off;
end