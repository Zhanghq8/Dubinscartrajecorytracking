%% Function to generate the desired trajectory 
function [traj, coeparam] = Generatetraj(x0, xf, t0, tf, dt)

    traj = cell(8, 1);
    coeparam = cell(2, 1);
        
    %polynomial trajectory coefficients
    syms t
    coe = sym('coe',[8,1]);
    a = coe(1:4);
    b = coe(5:8);

    %pos and vol coe
    p_t = [1; t; t^2; t^3];
    v_t = [0; 1; 2*t; 3*t^2];
    a_t = [0; 0; 2; 6*t];

    %x, y 
    x = a'*p_t;
    vx = a'*v_t;
    y = b'*p_t;
    vy = b'*v_t;

    equ1 = subs(x, t, t0) - x0(1);
    equ2 = subs(x, t, tf) - xf(1);
    equ3 = subs(y, t, t0) - x0(2);
    equ4 = subs(y, t, tf) - xf(2);
    equ5 = subs(vx, t, t0)*cos(x0(3)) + subs(vy, t, t0)*sin(x0(3)) - x0(4);
    equ6 = subs(vx, t, tf)*cos(xf(3)) + subs(vy, t, tf)*sin(xf(3)) - xf(4);
    equ7 = subs(vx, t, t0)*sin(x0(3)) - subs(vy, t, t0)*cos(x0(3));
    equ8 = subs(vx, t, tf)*sin(xf(3)) - subs(vy, t, tf)*cos(xf(3));
    
    %solve
    sol = solve([equ1==0, equ2==0, equ3==0, equ4==0, equ5==0, equ6==0, equ7==0, equ8==0], coe);
    coe = vpa([sol.coe1; sol.coe2; sol.coe3; sol.coe4; sol.coe5; sol.coe6; sol.coe7; sol.coe8]);

    %x coefficients
    coeparam{1} = coe(1:4);
    %y coefficients
    coeparam{2} = coe(5:8);

    for T = t0:dt:tf

        %x trajectory
        traj{1} = [traj{1}, subs(coeparam{1}'*p_t, t, T)];
        traj{2} = [traj{2}, subs(coeparam{1}'*v_t, t, T)];
        traj{3} = [traj{3}, subs(coeparam{1}'*a_t, t, T)];

        %y trajectory
        traj{4} = [traj{4}, subs(coeparam{2}'*p_t, t, T)];
        traj{5} = [traj{5}, subs(coeparam{2}'*v_t, t, T)];
        traj{6} = [traj{6}, subs(coeparam{2}'*a_t, t, T)];

        %theta trajectory
        traj{7} = [traj{7}, atan2(subs(coeparam{2}'*v_t, t, T), subs(coeparam{1}'*v_t, t, T))];
        %v trajectory
        traj{8} = [traj{8}, sqrt(subs(coeparam{1}'*v_t, t, T)^2 + subs(coeparam{2}'*v_t, t, T)^2)];

    end
end


