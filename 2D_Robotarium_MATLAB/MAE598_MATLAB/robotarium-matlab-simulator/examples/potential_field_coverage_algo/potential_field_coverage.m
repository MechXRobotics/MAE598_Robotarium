%% Potential-Field Swarm Coverage for Robotarium MATLAB Simulator
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

%% Graphics Overlay
fig = r.figure_handle;
figure(fig);

base_axes = gca;

pf_axes = axes('Parent',fig,...
               'Position',get(base_axes,'Position'),...
               'Color','none',...
               'XLim',workspace(1:2),...
               'YLim',workspace(3:4),...
               'XTick',[],'YTick',[],...
               'HitTest','off');
hold(pf_axes,'on');
uistack(base_axes,'top');

% Handle for arrows (force vectors)
force_handle = quiver(pf_axes, nan, nan, nan, nan, ...
                      'Color',[0 0.6 0], 'LineWidth',1);

%% MAIN LOOP
for t = 1:iterations
    
    poses = r.get_poses();
    x = poses(1:2,:);

    u = zeros(2,N);

    % Pairwise repulsion + global attraction
    for i = 1:N
        for j = 1:N
            if i == j, continue; end
            d = x(:,i) - x(:,j);
            dist = norm(d);

            if dist < 0.4
                u(:,i) = u(:,i) + 0.15 * (d / (dist+1e-6)); % repulsion
            end
        end
        u(:,i) = u(:,i) - 0.03 * (x(:,i) - [0;0]); % global attraction
    end

    %% Update visualization
    set(force_handle, 'XData', x(1,:), 'YData', x(2,:), ...
                      'UData', u(1,:), 'VData', u(2,:));

    uistack(base_axes,'top');

    %% Control law
    dx = si_pos(x, x + u);
    dx = barrier(dx, poses);
    dxu = si_to_uni(dx, poses);

    r.set_velocities(1:N, dxu);
    r.step();
end

r.debug();
