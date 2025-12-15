%% Sweep (Boustrophedon) Coverage for Robotarium MATLAB Simulator
clear; clc;

%% Parameters
N = 8;
iterations = 2000;
workspace = [-1 1 -0.8 0.8];

%% Robotarium Initialization
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

si_to_uni = create_si_to_uni_dynamics();
si_pos = create_si_position_controller();
barrier = create_si_barrier_certificate_with_boundary();

%% Sweep lane setup
xmin = workspace(1); xmax = workspace(2);
ymin = workspace(3); ymax = workspace(4);

lane_width = (xmax - xmin) / N;

% Each robot sweeps inside its own lane
lane_centers = xmin + lane_width*(0.5:1:N-0.5);

% Sweep direction per robot
direction = ones(1,N);

%% Overlay Axes
fig = r.figure_handle;
figure(fig);

base_axes = gca;

sweep_axes = axes('Parent', fig, ...
                  'Position', get(base_axes,'Position'), ...
                  'Color','none',...
                  'XLim',workspace(1:2),...
                  'YLim',workspace(3:4),...
                  'XTick',[],'YTick',[],...
                  'HitTest','off');
hold(sweep_axes,'on');
uistack(base_axes,'top');

%% Persistent graphics handles
lane_handles = gobjects(1,N);
target_handles = plot(sweep_axes, nan, nan, 'rx','MarkerSize',10,'LineWidth',1.5);

for i = 1:N
    x_left  = xmin + lane_width*(i-1);
    x_right = xmin + lane_width*i;
    lane_handles(i) = patch(sweep_axes, ...
                            [x_left x_right x_right x_left], ...
                            [ymin ymin ymax ymax], ...
                            [0.8 0.8 1], 'FaceAlpha',0.15, 'EdgeColor','none');
end

%% MAIN LOOP
for t = 1:iterations
    
    poses = r.get_poses();
    x = poses(1:2,:);

    % Compute sweep targets
    targets = zeros(2,N);
    for i = 1:N
        lane_center = lane_centers(i);

        if direction(i) > 0
            targets(:,i) = [lane_center; ymax];
        else
            targets(:,i) = [lane_center; ymin];
        end

        % Switch direction when reaching sweep edge
        if abs(x(2,i) - targets(2,i)) < 0.05
            direction(i) = -direction(i);
        end
    end

    %% Update visualization
    set(target_handles, 'XData', targets(1,:), 'YData', targets(2,:));
    uistack(base_axes,'top');

    %% Control law
    dx = si_pos(x, targets);
    dx = barrier(dx, poses);
    dxu = si_to_uni(dx, poses);

    %% Apply to robots
    r.set_velocities(1:N, dxu);
    r.step();
end

r.debug();
