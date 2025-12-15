%% Deadlock-Free Frontier Exploration for Robotarium
clear; clc;

N = 8;
iterations = 2000;
workspace = [-1 1 -0.8 0.8];
grid_res = 50;

occ = zeros(grid_res);

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
si_to_uni = create_si_to_uni_dynamics();
si_pos = create_si_position_controller();
barrier = create_si_barrier_certificate_with_boundary();

%% GRAPHICS OVERLAY
fig = r.figure_handle;
figure(fig);
base_axes = gca;

map_axes = axes('Parent',fig,...
    'Position',get(base_axes,'Position'),...
    'Color','none',...
    'XLim',workspace(1:2),...
    'YLim',workspace(3:4),...
    'YDir','normal',...
    'XTick',[],'YTick',[]);
hold(map_axes,'on');
uistack(base_axes,'top');

map_img = imagesc(map_axes,...
    linspace(workspace(1),workspace(2),grid_res),...
    linspace(workspace(3),workspace(4),grid_res), occ);
colormap(map_axes,[0.7 0.7 0.7; 1 1 1]);

frontier_handle = plot(map_axes,nan,nan,'rs','MarkerFaceColor','r');
target_handle   = plot(map_axes,nan,nan,'bx','MarkerSize',10,'LineWidth',2);

%% MAIN LOOP
for t = 1:iterations

    poses = r.get_poses();
    x = poses(1:2,:);

    %% Mark explored
    for i = 1:N
        gx = round((x(1,i)-workspace(1))/(workspace(2)-workspace(1))*(grid_res-1))+1;
        gy = round((x(2,i)-workspace(3))/(workspace(4)-workspace(3))*(grid_res-1))+1;

        gx = min(max(gx,1),grid_res);
        gy = min(max(gy,1),grid_res);

        occ(gy,gx) = 1;
    end
    set(map_img,'CData',occ);

    %% Identify true frontier pixels
    frontier = [];
    for iy = 2:grid_res-1
        for ix = 2:grid_res-1
            if occ(iy,ix)==1 && any(occ(iy-1:iy+1, ix-1:ix+1)==0,'all')
                frontier = [frontier; iy ix];
            end
        end
    end

    if isempty(frontier)
        dxu = zeros(2,N);
        r.set_velocities(1:N, dxu);
        r.step();
        continue;
    end

    %% Convert to XY coordinates
    fx = linspace(workspace(1),workspace(2),grid_res);
    fy = linspace(workspace(3),workspace(4),grid_res);

    frontier_xy = [fx(frontier(:,2)); fy(frontier(:,1))];
    set(frontier_handle,'XData',frontier_xy(1,:), 'YData',frontier_xy(2,:));

    %% K-means clustering of frontier points
    K = min(N, size(frontier_xy,2));
    [cluster_idx, cluster_centers] = kmeans(frontier_xy', K);

    %% Choose medoids (closest actual frontier point)
    medoids = zeros(2,K);
    for k = 1:K
        pts = frontier_xy(:, cluster_idx==k);
        if isempty(pts)
            medoids(:,k) = cluster_centers(k,:)';
            continue;
        end
        d = vecnorm(pts - cluster_centers(k,:)', 2, 1);
        [~, idx] = min(d);
        medoids(:,k) = pts(:,idx);
    end

    set(target_handle,'XData',medoids(1,:), 'YData',medoids(2,:));

    %% Hungarian assignment (optimal matching)
    cost = zeros(N,K);
    for i = 1:N
        for k = 1:K
            cost(i,k) = norm(x(:,i) - medoids(:,k));
        end
    end

    % For fairness, use matchpairs from MATLAB (Hungarian)
    assignment = matchpairs(cost, 100);

    %% Assign each robot a unique frontier cluster
    targets = zeros(2,N);
    for a = 1:size(assignment,1)
        robot = assignment(a,1);
        clust = assignment(a,2);
        targets(:,robot) = medoids(:,clust);
    end

    %% Add small escape forces to avoid blocking
    escape = zeros(2,N);
    for i = 1:N
        for j = 1:N
            if i==j, continue; end
            d = x(:,i) - x(:,j);
            dist = norm(d);
            if dist < 0.2
                escape(:,i) = escape(:,i) + 0.05 * (d / (dist+1e-6));
            end
        end
    end

    %% Control law
    dx = si_pos(x, targets + escape);
    dx = barrier(dx, poses);
    dxu = si_to_uni(dx, poses);

    r.set_velocities(1:N, dxu);
    r.step();
end

r.debug();
