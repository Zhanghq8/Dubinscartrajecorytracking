%% Main function of the closed-loop trajectory tracking controller for the augmented system of dubinscar
clc;
clear all;
close all;

% Initial and final state of pos, vel, theta, time
x0 = 10;
xf = 0;
y0 = 20;
yf = 0;
v0 = 10;
vf = 0;
theta0 = 0.5;
thetaf = 0;
t0 = 0;
tf = 10;
dt = 0.1;

X0 = [x0, y0, theta0, v0];
Xf = [xf, yf, thetaf, vf];

% PD controller gain
kp1 = 10;
kp2 = 10;
kd1 = 10;
kd2 = 10;

Ttemp = t0:dt:tf;

% Trajectory generation
[X_d, trajparam] = Generatetraj(X0, Xf, t0, tf, dt);

conparam = {kp1, kp2, kd1, kd2};

%the initial state, final state
%x0 = [9, 15, 0.5, 8];
x0 = [11.0,21.0,0.5,10]';

options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) odeaugdubinscarztracking(t,x, trajparam, conparam),[t0 tf],x0, options);

%X: x, y, dx, dy
%X_d: x, vx, ax, y, vy, ay, theta, v
figure('Name','perfomance of the system');

subplot(2, 3, 1);
plot(T, X(:,1),'r-');
hold on
plot(Ttemp, X_d{1}, 'b--');
legend('actual','desired', 'Location', 'SouthOutside');
title('x tracking.');
grid on;
%x Trajectory

subplot(2, 3, 2);
plot(T, X(:,2),'r-');
hold on
plot(Ttemp, X_d{4}, 'b--');
legend('actual','desired', 'Location', 'SouthOutside');
title('y tracking.');
grid on;
%y Trajectory

subplot(2, 3, 4);
plot(T, X(:,4).*cos(X(:,3)),'r-');
hold on
plot(Ttemp, X_d{2}, 'b--');
legend('actual','desired', 'Location', 'SouthOutside');
title('vx tracking.');
grid on;
%x Velocity

subplot(2, 3, 5);
plot(T, X(:,4).*sin(X(:,3)),'r-');
hold on
plot(Ttemp, X_d{5}, 'b--');
legend('actual','desired', 'Location', 'SouthOutside');
title('vy tracking.');
grid on;
%y Velocity

subplot(2, 3, 3);
plot(T, X(:,3),'r-');
hold on
plot(Ttemp, X_d{7}, 'b--');
legend('actual','desired', 'Location', 'SouthOutside');
title('theta tracking.');
grid on;
%theta Trajectory

subplot(2, 3, 6);
plot(T, X(:,4), 'r-');
hold on
plot(Ttemp, X_d{8}, 'b--');
legend('actual','desired', 'Location', 'SouthOutside');
title('v tracking.');
grid on;
%v Velocity









