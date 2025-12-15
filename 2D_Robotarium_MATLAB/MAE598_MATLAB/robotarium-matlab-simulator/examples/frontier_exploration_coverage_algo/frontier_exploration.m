%% Frontier Exploration for Robotarium MATLAB Simulator
clear; clc;

%% Parameters
N = 8;
iterations = 2000;
workspace = [-1 1 -0.8 0.8];

grid_res = 50;
occ = zeros(grid_res);   % 0 = unknown, 1 = explored

%% Robotarium Init
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

si_to_uni = create_si_to_uni_dynamics();
si_pos = create_si_position_controller();
barrier = create_si_barrier_certificate_with_boundary();

%% Graphics overlay
fig = r.figure_handle;
figure(fig);
base_axes = gca;

map_axes = axes('Parent', fig, ...
                'Position', get(base_axes,'Position'), ...
                'Color','none',...
                'XLim',workspace(1:2),...
                'YLim',workspace(3:4),...
                'YDir','normal',...
                'XTick',[],'YTick',[]);

hold(map_axes,'on');
uistack(base_axes,'top');

% Occupancy map
map_img = imagesc(map_axes, linspace(workspace(1),workspace(2),grid_res),...
                  linspace(workspace(3),workspace(4),grid_res),...
                  occ);
colormap(map_axes,[0.7 0.7 0.7; 1 1 1]); % gray=unknown, white=explored
set(map_axes,'YDir','normal');

% Frontier graphics
frontier_handle = plot(map_axes, nan,nan,'rs','MarkerSize',6,'MarkerFaceColor','r');

%% MAIN LOOP
for t = 1:iterations

    poses = r.get_poses();
    x = poses(1:2,:);

    %% Update occupancy grid
    for i = 1:N
        gx = round((x(1,i)-workspace(1)) / (workspace(2)-workspace(1)) * (grid_res-1)) + 1;
        gy = round((x(2,i)-workspace(3)) / (workspace(4)-workspace(3)) * (grid_res-1)) + 1;
        
        gx = min(max(gx,1),grid_res);
        gy = min(max(gy,1),grid_res);

        occ(gy, gx) = 1; % mark explored
    end

    set(map_img,'CData',occ);

    %% Compute frontier cells
    frontier = [];
    for i = 2:grid_res-1
        for j = 2:grid_res-1
            if occ(i,j)==1 && any(occ(i-1:i+1,j-1:j+1)==0,'all')
                frontier = [frontier; i j];
            end
        end
    end

    % Convert frontier grid positions to coordinates
    if isempty(frontier)
        frontier_xy = zeros(2,0);
    else
        fx = linspace(workspace(1),workspace(2),grid_res);
        fy = linspace(workspace(3),workspace(4),grid_res);
        frontier_xy = [fx(frontier(:,2)); fy(frontier(:,1))];
    end

    set(frontier_handle,'XData',frontier_xy(1,:), 'YData',frontier_xy(2,:));

    uistack(base_axes,'top');

    %% Compute targets for robots
    targets = zeros(2,N);
    if ~isempty(frontier_xy)
        for i = 1:N
            d = vecnorm(frontier_xy - x(:,i), 2, 1);
            [~,idx] = min(d);
            targets(:,i) = frontier_xy(:,idx);
        end
    else
        targets = zeros(2,N); % no frontier: go home
    end

    %% Control law
    dx = si_pos(x, targets);
    dx = barrier(dx, poses);
    dxu = si_to_uni(dx, poses);

    r.set_velocities(1:N, dxu);
    r.step();
end

r.debug();
