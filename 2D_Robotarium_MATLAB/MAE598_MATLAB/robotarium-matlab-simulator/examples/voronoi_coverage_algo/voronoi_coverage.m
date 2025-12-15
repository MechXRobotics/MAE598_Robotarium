%% Polished Voronoi / Lloyd Coverage with Colored Cells for Robotarium Simulator
clear; clc;

%% ------------------ PARAMETERS ------------------
N = 8;
iterations = 2000;
workspace = [-1 1 -0.8 0.8];

%% ------------------ CREATE ROBOTARIUM INSTANCE ------------------
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

si_pos_controller = create_si_position_controller();
si_to_uni = create_si_to_uni_dynamics();
barrier_certificate = create_si_barrier_certificate_with_boundary();

%% ------------------ SETUP GRAPHICAL OVERLAY ------------------
fig = r.figure_handle;
figure(fig);

robot_axes = gca;

% Transparent overlay for Voronoi objects
voro_axes = axes('Parent', fig, ...
                 'Position', get(robot_axes, 'Position'), ...
                 'Color', 'none', ...
                 'XLim', workspace(1:2), ...
                 'YLim', workspace(3:4), ...
                 'XTick', [], 'YTick', [], ...
                 'HitTest', 'off');
hold(voro_axes, 'on');

% Keep robot graphics on top
uistack(robot_axes, 'top');

%% Color palette for cells
cmap = hsv(N);         % distinct colors
alpha_val = 0.20;      % cell transparency

%% ------------------ MAIN LOOP ------------------
for t = 1:iterations
    poses = r.get_poses();
    x = poses(1:2, :);

    %% ------------------ Compute Voronoi ------------------
    [V, C] = bounded_voronoi(x', workspace);

    %% ------------------ Compute centroids ------------------
    centroids = zeros(2, N);
    for i = 1:N
        if isempty(C{i})
            centroids(:, i) = x(:, i);
        else
            region = V(C{i}, :);
            centroids(:, i) = polygon_centroid(region)';
        end
    end

    %% ------------------ Graphics Rendering ------------------
    cla(voro_axes);

    % Draw Voronoi Cells - filled with transparency
    for i = 1:N
        if isempty(C{i}), continue; end
        region = V(C{i}, :);

        patch(voro_axes, region(:,1), region(:,2), cmap(i,:), ...
            'FaceAlpha', alpha_val, 'EdgeColor', 'none');
    end

    % Draw Voronoi edges (light)
    [vx, hy] = voronoi(x(1,:), x(2,:));
    for k = 1:length(hy)
        if all(isfinite([vx(:,k); hy(:,k)]))
            plot(voro_axes, vx(:,k), hy(:,k), 'Color', [0.2 0.2 0.8], 'LineWidth', 0.8);
        end
    end

    % Draw centroids
    plot(voro_axes, centroids(1,:), centroids(2,:), 'rx', 'LineWidth', 2, 'MarkerSize', 10);

    % Title
    title(voro_axes, sprintf('Voronoi Coverage — Iteration %d', t), 'FontSize', 14);

    % Keep robot axes on top
    uistack(robot_axes, 'top');

    %% ------------------ Control Law ------------------
    dx = si_pos_controller(x, centroids);

    dx = barrier_certificate(dx, poses); % safety
    dxu = si_to_uni(dx, poses);          % convert to unicycle

    r.set_velocities(1:N, dxu);
    r.step();
end

r.debug();
%% ============================================================
% Helper Functions
%% ============================================================

function [V_new, C_new] = bounded_voronoi(points, box)
    xmin=box(1); xmax=box(2);
    ymin=box(3); ymax=box(4);

    [V,C] = voronoin(points);

    B = [xmin ymin; xmax ymin; xmax ymax; xmin ymax];
    V_new = []; C_new = cell(numel(C),1); offset = 0;

    for i = 1:numel(C)
        region = C{i};
        if any(region == 1), continue; end
        
        poly = V(region, :);
        clipped = polygon_clip(poly, B);
        if isempty(clipped), continue; end

        m = size(clipped,1);
        V_new = [V_new; clipped];
        C_new{i} = offset + (1:m);
        offset = offset + m;
    end
end

function centroid = polygon_centroid(P)
    x=P(:,1); y=P(:,2);
    A = polyarea(x,y);
    if A < 1e-12
        centroid = mean(P,1);
        return;
    end
    cx = sum((x + x([2:end 1])) .* ...
             (x.*y([2:end 1]) - x([2:end 1]).*y)) / (6*A);
    cy = sum((y + y([2:end 1])) .* ...
             (x.*y([2:end 1]) - x([2:end 1]).*y)) / (6*A);
    centroid = [cx cy];
end

function P = polygon_clip(P, B)
    for e = 1:4
        newP = []; p1 = P(end,:);
        for i = 1:size(P,1)
            p2 = P(i,:);
            if inside(p2,B,e)
                if ~inside(p1,B,e)
                    newP = [newP; intersect_line(p1,p2,B,e)];
                end
                newP = [newP; p2];
            elseif inside(p1,B,e)
                newP = [newP; intersect_line(p1,p2,B,e)];
            end
            p1 = p2;
        end
        P = newP;
        if isempty(P), return; end
    end
end

function tf = inside(p,B,e)
    x=p(1); y=p(2);
    switch e
        case 1, tf = x>=B(1,1);
        case 2, tf = x<=B(2,1);
        case 3, tf = y>=B(1,2);
        case 4, tf = y<=B(3,2);
    end
end

function p = intersect_line(p1,p2,B,e)
    x1=p1(1); y1=p1(2);
    x2=p2(1); y2=p2(2);

    switch e
        case 1
            x=B(1,1); y=y1+(x-x1)*(y2-y1)/(x2-x1);
        case 2
            x=B(2,1); y=y1+(x-x1)*(y2-y1)/(x2-x1);
        case 3
            y=B(1,2); x=x1+(y-y1)*(x2-x1)/(y2-y1);
        case 4
            y=B(3,2); x=x1+(y-y1)*(x2-x1)/(y2-y1);
    end
    p=[x y];
end
