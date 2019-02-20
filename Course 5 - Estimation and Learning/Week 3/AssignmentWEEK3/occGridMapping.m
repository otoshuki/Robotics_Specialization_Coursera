% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function Map = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
Resol = param.resol;
% % the initial map size in pixels
Map = zeros(param.size);
% % the origin of the map in pixels
Origin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
scanN = size(ranges,1);

for j = 1:N % for each time,  
    % Find grids hit by the rays (in the gird map coordinate)
    loc = pose(1:2,j)'; 
    theta = pose(3,j);
    %for each measurement at loc and theta
    j;
    for i = 1:scanN
        dcos = ranges(i,j)*cos(theta+scanAngles(i));
        dsin = -ranges(i,j)*sin(theta+scanAngles(i));
        occ_loc = [dcos,dsin]+loc;

        % Find occupied-measurement cells and free-measurement cells
        grid_loc = ceil(occ_loc*Resol) + Origin'; % Occupied
        grid_pos = ceil(loc*Resol) + Origin';
            % Set maximum sizes for occupied blocks
        if grid_loc(2) > size(Map,1)
            grid_loc(2) = size(Map,1);
        end
        if grid_loc(1) > size(Map,2)
            grid_loc(1) = size(Map,2);
        end
        [freex, freey] = bresenham(grid_pos(1),grid_pos(2),grid_loc(1),grid_loc(2)); %Free
        % Update the log-odds
            % Occupied
        Map(grid_loc(2),grid_loc(1)) = Map(grid_loc(2),grid_loc(1)) + lo_occ;
            % Saturate the log-odd values
        if Map(grid_loc(2),grid_loc(1)) > lo_max
            Map(grid_loc(2),grid_loc(1)) = lo_max;
        end
            % Free
        for k = 1:size(freex,1)-1
            Map(freey(k),freex(k)) = Map(freey(k),freex(k)) - lo_free;
                % Set maximum sizes for free blocks
            if freey(k) > size(Map,1)
                freey(k) = size(Map,1);
            end
            if freex(k) > size(Map,2)
                freex(k) = size(Map,2);
            end
            if Map(freey(k),freex(k)) < lo_min
                Map(freey(k),freex(k)) = lo_min;
            end
        end
        % Visualize the map as needed
        
    end
end

end

