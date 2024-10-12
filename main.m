% housekeeping
clear all;
clc;

%% Powered descent module paramters
m_dry = 280.9 + 506;             	                                        % drymass, kg
m_fuel = 907.4 - 280.9;              	                                    % fuel mass, kgm_t = m_f + m_d;			            
Ft = 1100;        	  	                                                    % maximum thrust per engine, N
Isp = 321;                                                                  % specific impulse, s
r_0 = [-250000;0;24000];                                                    % initial position starting from the main braking stage, m
v_0 = [1454;0;0];                                                           % initial position starting from the main braking stage, m
v_f = [55;0;-30];                                                           % initial velocity starting from the main braking stage, m/s
r_f = [-2100;0;3000];                                                       % initial velocity starting from the main braking stage, m/s

% code to find optimal time
[t_opt, rocket] = pre_descent_stage(Ft, m_dry, m_fuel, Isp, r_0, v_0, r_f, v_f);

% Solve the problem using the optimal time of flight
sol_pds = solve_pdg_fft_descent(t_opt, rocket);
disp('fuel used: ')
disp(sol_pds.m_used)

deltat=15;

r_f(3)=r_f(3)+v_f(3)*deltat-0.5*1.3*deltat^2;
r_f(1)=r_f(1)+v_f(1)*deltat;
v_f(3)=v_f(3)-1.3*deltat;
%% Lander module parameters
r_f(1) = r_f(1) + 100
m_dry = 429.5;             	                                                % drymass, kg
m_fuel = 506 - m_dry;              	                                        % fuel mass, kgm_t = m_f + m_d;			            
Ft = 525.9;        	  	                                                    % maximum thrust per engine, N
Isp = 223;                                                                  % specific impulse, s
r_0 = r_f                                                                   % initial position starting from the terminal descent stage, m
v_0 = v_f                                                                   % initial velocity starting from the terminal descent stage, m/s  

[t_opt, rocket] = pre_lander(Ft, m_dry, m_fuel, Isp, r_0, v_0);

% Solve the problem using the optimal time of flight
sol_lander = solve_pdg_fft_lander(t_opt, rocket);
disp('fuel used: ')
disp(sol_lander.m_used)

r_land = sol_lander.r;
v_land = sol_lander.v;
u_land = sol_lander.u;
z_land = sol_lander.z;

save('r_l', "r_land")
save('v_l', 'v_land')
save('u_l', 'u_land')
save('z_l', 'z_land')
%% Plotting of trajectory

plot_trajectory_combined(sol_lander, sol_pds)
pds_trajec_plot(sol_pds)
landing_trajec_plot(sol_lander)

%% Plotting of forces
% plot_forces_pds(sol_pds)
plot_forces_lander(sol_lander)