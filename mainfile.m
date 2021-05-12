% Slider crank kinematic analysis
%
% Coordinates
% ground
q1 = [0; 0; 0];
% crank
q2 = [0.25 * cosd(20)
    0.25 * sind(20)
    deg2rad(20)];
% link
q3 = [0.5 * cosd(20) + 0.25 * cos(20)
    0.25*sind(20)
    deg2rad(-20)];
% slider
q4 = [0.5 * cosd(20) + 0.5 * cos(20)
    0
    0];

q_0 = [q1; q2; q3; q4]; % initial coordinates

%% We need two constraint types (geometric ones)
% - revolute
% - simple constraints

%% Revolute joints
% 1 connects ground and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [-0.25; 0];

% 2 connects crank and link
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [0.25; 0];
revolute(2).s_j = [-0.25; 0];

% 3 connects link and slider
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [0.25; 0];
revolute(3).s_j = [0; 0];

% % Check revolute joint constraints
% r = revolute(3);
% C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0)

%% Simple constraints

% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

% slider - use simple joints instead of translational
simple(4).i = 4;
simple(4).k = 2;
simple(4).c_k = 0;

simple(5).i = 4;
simple(5).k = 3;
simple(5).c_k = 0;

% % check simple constraints
% for s = simple
%     C_s_i = simple_joint(s.i, s.k, s.c_k, q_0)
% end

%% Add some driving constraints
driving.i = 4;
driving.k = 1;
driving.d_k = @(t) 0.5 * cosd(20) + 0.5 * cos(20) + 0.10*sin(t)
driving.d_k_t = @(t) 0.10*cos(t);
driving.d_k_tt = @(t) -0.10*sin(t);



%% Solve constraint equation using fsolve
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
tic
[T, Q] = position_fsolve(C_fun, 3, q_0, 0.1);
toc
%% Some verification plots
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal

%% Jacobian of our constraints
Cq = constraint_dq(revolute, simple, driving, 0, q_0)

%% Solve constraint equation using NR
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
tic
[T, Q] = position_NR(C_fun, Cq_fun, 3, q_0, 0.1);
toc
%% Some verification plots
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal

%% Verify Ct
Ct = constraint_dt(revolute, simple, driving, 0, q_0)

%% Solve constraint equation using NR for position and velocity
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
[T, Q, QP] = pos_vel_NR(C_fun, Cq_fun, Ct_fun, 5, q_0, 0.1);

%% Some verification plots
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title 'Position'



%% Some verification plots - velocity
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal
title 'Velocity'

%% Accelecaration slider

tsim=linspace (0,5,51); % same amount of time steps as Q, QP

gx=(Q(:,4).*QP(:,6).^2)-(Q(:,7).*QP(:,9).^2); % acceleration in x-direction


figure;
plot (tsim, -0.10*sin(tsim),'LineWidth', 2)
title 'Driven acceleration'

figure;
plot (tsim, gx, 'g','LineWidth', 2)
title 'Revolute joint acceleration in x-direction'




