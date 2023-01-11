function [Next,NextR] = nextPoints(x, corners,config,cdata,sdata)

      board_width = config.board_width;
      board_height = config.board_height;
      NextR = [];
      Rx = [1 0 0;
            0 cos(x(1)) -sin(x(1));    
            0 sin(x(1)) cos(x(1))];
      Ry = [cos(x(2)) 0 sin(x(2));
            0 1 0;
            -sin(x(2)) 0 cos(x(2))];
      Rz = [cos(x(3)) -sin(x(3)) 0;
            sin(x(3)) cos(x(3)) 0;
            0 0 1];
      R = Rz * Ry * Rx; % Rotation matrix in the next frame
      t = x(4:6)';

      objects = zeros(3);
      P = zeros(2,board_width * board_height);

      if length(cdata) == 2
            objects_R = zeros(3);
            P_R = zeros(2,board_width * board_height);
      end
      

      for i = 1 : board_height
            for j = 1 : board_width
                  pos = j + (i - 1) * board_width;
                  % Calculate 3D points under the camera coordinate with a new pose 
                  objects = R * corners(:,pos) + t;
                  [P(1,pos),P(2,pos)] = projectPoints(objects,cdata(1));

                  if length(cdata) == 2
                        objects_R = sdata.R * objects + sdata.T;
                        [P_R(1,pos),P_R(2,pos)] = projectPoints(objects_R,cdata(2));
                  end
            end
      end
      Next = P;
      if length(cdata) == 2
            NextR = P_R;
      end
