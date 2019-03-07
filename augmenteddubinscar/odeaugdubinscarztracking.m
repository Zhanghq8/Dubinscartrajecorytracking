%% Ode function of the closed-loop trajectory tracking controller for the augmented system of dubinscar
function [ dx ] = odeaugdubinscarztracking( t, x, trajparam, conparam )
        %trajectory parameters(coefficients);
        a = trajparam{1};
        b = trajparam{2};
        
        %kp, kd (controller parameters);        
        kp1 = conparam{1};
        kp2 = conparam{2};
        kd1 = conparam{3};
        kd2 = conparam{4};
        kp = [kp1 0;0 kp2];
        kd = [kd1 0;0 kd2];

        % note state vector x is in the form of x, y, dx, dy
        vec_t = [1; t; t^2; t^3]; % cubic polynomials

        % desired trajectory
        x_d= [a'*vec_t; b'*vec_t];
        
        % compute the velocity and acceleration in x.
        a_vel = [a(2), 2*a(3), 3*a(4), 0];
        a_acc = [2*a(3), 6*a(4),0,0 ];
        b_vel = [b(2), 2*b(3), 3*b(4), 0];
        b_acc = [2*b(3), 6*b(4),0,0 ];
        
        % compute the desired trajectory for x and y
        dx_d =[a_vel*vec_t; b_vel* vec_t];
        ddx_d =[a_acc*vec_t; b_acc* vec_t];

        % the actual trajectory
        x_a= x(1:2,1);
        theta = x(3);
        v = x(4);

        dx_a = [v*cos(theta); v*sin(theta)];



        %position error
        e=[x(1)-x_d(1);x(2)-x_d(2)];
        
        %velocity error
        de=[dx_a(1)-dx_d(1);dx_a(2)-dx_d(2)]; 
        
        %New Controller
        u = - kp * e - kd * de + ddx_d;

        %linear acceleration and angular accleration
        acc = u(1,:)*cos(theta) + u(2,:)*sin(theta);
        w = (u(2,:)*cos(theta) - u(1,:)*sin(theta))/v;
        
        ddx_a = [w; acc];

        % use the computed torque and state space model to compute
        % the increment in state vector.
        dx=zeros(4,1);
        dx(1)=dx_a(1);
        dx(2)=dx_a(2);
        dx(3)= ddx_a(1);
        dx(4)= ddx_a(2);

end