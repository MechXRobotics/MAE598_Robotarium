%% Improved Potential-Field Swarm Coverage (Deadlock-Free)
clear; clc;

N = 8;
iterations = 2000;
workspace = [-1 1 -0.8 0.8];

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

si_to_uni = create_si_to_uni_dynamics();
si_pos = create_si_position_controller();
barrier = create_si_barrier_certificate_with_boundary();

%% GRAPHICS OVERLAY
fig = r.figure_handle;
figure(fig);
base_axes = gca;

pf_axes = axes('Parent', fig, ...
               'Position', get(base_axes,'Position'), ...
               'Color','none', ...
               'XLim',workspace(1:2), ...
               'YLim',workspace(3:4), ...
               'XTick',[],'YTick',[], ...
               'HitTest','off');
hold(pf_axes,'on');
uistack(base_axes,'top');

force_handle = quiver(pf_axes, nan, nan, nan, nan, ...
                      'Color',[0 0.6 0], 'LineWidth',1);

%% MAIN LOOP
for t = 1:iterations
    
    poses = r.get_poses();
    x = poses(1:2,:);

    u = zeros(2,N);

    %% --- Inter-robot Repulsion ---
    for i = 1:N
        for j = 1:N
            if i == j, continue; end
            d = x(:,i) - x(:,j);
            dist = norm(d);

            if dist < 0.4
                u(:,i) = u(:,i) + 0.25*(d / (dist+1e-6));  % strong repulsion
            end
        end
    end

    %% --- Workspace boundary repulsion ---
    for i = 1:N
        if x(1,i) < workspace(1)+0.2
            u(1,i) = u(1,i) + 0.2;
        elseif x(1,i) > workspace(2)-0.2
            u(1,i) = u(1,i) - 0.2;
        end
        if x(2,i) < workspace(3)+0.2
            u(2,i) = u(2,i) + 0.2;
        elseif x(2,i) > workspace(4)-0.2
            u(2,i) = u(2,i) - 0.2;
        end
    end

    %% --- Circulation field to break symmetry ---
    for i = 1:N
        v = x(:,i);
        u(:,i) = u(:,i) + 0.05 * [ -v(2); v(1) ];  % rotate vector field
    end

    %% --- Noise injection to escape local minima ---
    u = u + 0.01 * randn(2,N);

    %% Update Graphics
    set(force_handle, 'XData', x(1,:), 'YData', x(2,:), ...
                      'UData', u(1,:), 'VData', u(2,:));
    uistack(base_axes,'top');

    %% Control Law
    dx = si_pos(x, x + u);
    dx = barrier(dx, poses);
    dxu = si_to_uni(dx, poses);

    r.set_velocities(1:N, dxu);
    r.step();
end

r.debug();
