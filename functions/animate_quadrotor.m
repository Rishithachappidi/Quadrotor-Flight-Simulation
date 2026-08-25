function animate_quadrotor(log, p, gif_path)
% ANIMATE_QUADROTOR  Render a simple 3D quadrotor frame flying along its
% logged trajectory and save the animation as an animated GIF.
%
%   animate_quadrotor(log, p, gif_path)

    isOctave = logical(exist('OCTAVE_VERSION', 'builtin'));
    if isOctave
        pkg load image;   % needed for rgb2ind() when running under GNU Octave
    end

    % Subsample so the GIF has roughly 100 frames total (fast + small file)
    target_frames = 100;
    step = max(1, round(length(log.t) / target_frames));
    idx  = 1:step:length(log.t);

    fig = figure('Color','w','Position',[100 100 640 520]);
    axis equal; grid on; hold on; box on;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Quadrotor Drone - 3D Flight Simulation');
    view(45,25);

    xlim([min(log.state(1,:))-1, max(log.state(1,:))+1]);
    ylim([min(log.state(2,:))-1, max(log.state(2,:))+1]);
    zlim([0, max(log.state(3,:))+1]);

    plot3(log.ref_pos(1,:), log.ref_pos(2,:), log.ref_pos(3,:), ...
        'r--', 'LineWidth', 1);

    trailLine = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.3);
    armLine1  = plot3(nan, nan, nan, 'k-', 'LineWidth', 2.5);
    armLine2  = plot3(nan, nan, nan, 'k-', 'LineWidth', 2.5);
    rotorPts  = plot3(nan, nan, nan, 'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', [0.1 0.4 0.9], 'MarkerEdgeColor','k');

    l = p.l;
    % Arm endpoints in body frame (X configuration)
    armPtsBody = l * [ 1/sqrt(2), -1/sqrt(2), 0;
                       -1/sqrt(2),  1/sqrt(2), 0]';   % arm 1: FR-RL
    armPtsBody2 = l * [-1/sqrt(2), -1/sqrt(2), 0;
                        1/sqrt(2),  1/sqrt(2), 0]';   % arm 2: FL-RR

    firstFrame = true;
    for k = idx
        pos = log.state(1:3,k);
        phi = log.state(7,k); theta = log.state(8,k); psi = log.state(9,k);

        Rb2i = eul2rotm_zyx(phi, theta, psi);

        arm1 = pos + Rb2i * armPtsBody;
        arm2 = pos + Rb2i * armPtsBody2;

        set(armLine1, 'XData', arm1(1,:), 'YData', arm1(2,:), 'ZData', arm1(3,:));
        set(armLine2, 'XData', arm2(1,:), 'YData', arm2(2,:), 'ZData', arm2(3,:));
        set(rotorPts, 'XData', [arm1(1,:) arm2(1,:)], ...
                      'YData', [arm1(2,:) arm2(2,:)], ...
                      'ZData', [arm1(3,:) arm2(3,:)]);
        set(trailLine, 'XData', log.state(1,1:k), 'YData', log.state(2,1:k), ...
                       'ZData', log.state(3,1:k));

        title(sprintf('Quadrotor Drone - 3D Flight Simulation  (t = %.1f s)', log.t(k)));
        drawnow;

        % Capture frame via print+imread (portable across MATLAB/Octave,
        % works with both qt/opengl and gnuplot graphics toolkits, unlike
        % getframe() which gnuplot does not support).
        tmp_png = [tempname(), '.png'];
        print(fig, tmp_png, '-dpng', '-r55');
        im = imread(tmp_png);
        delete(tmp_png);

        if isOctave
            % Octave's rgb2ind() has no N-color argument, so posterize each
            % channel first to guarantee the image fits under 256 colors.
            im = bitand(im, uint8(192));
            [imind, cm] = rgb2ind(im);
        else
            [imind, cm] = rgb2ind(im, 256);
        end
        if firstFrame
            imwrite(imind, cm, gif_path, 'gif', 'Loopcount', inf, 'DelayTime', 0.03);
            firstFrame = false;
        else
            imwrite(imind, cm, gif_path, 'gif', 'WriteMode', 'append', 'DelayTime', 0.03);
        end
    end
end

function R = eul2rotm_zyx(phi, theta, psi)
% Body-to-inertial rotation matrix (ZYX: yaw-pitch-roll convention)
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R = Rz * Ry * Rx;
end
