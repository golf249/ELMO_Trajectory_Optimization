function [t_opt,rocket] = pre_descent_stage(Ft, m_dry, m_fuel, Isp, r_0, v_0, r_f, v_f)

% constants and vehicle parameters
e_x = [1,0,0];                                                              % x-direction unit vector
e_y = [0,1,0];                                                              % y-direction unit vector
e_z = [0,0,1];                                                              % z-direction unit vector (altitude +ve normal away from surface)
g0 = 9.807;					                                                % Earth gravity, m/s^2
rocket.g_europa = -1.2894 * e_z;		                                    % Europa's gravity, m/s^2
rocket.gamma_p = 90;				                                        % pointing angle 
cant_angle = 0 * pi/180;                                                    % The cant angle off the thrust line
n_en = 5;                                                                   % The number of engine				                                                            % specific impulse, s
rocket.m_d = m_dry;             	                                        % drymass, kg
m_f = m_fuel;              	                                                % fuel mass, kg
rocket.m_t = m_f + rocket.m_d;			            
rocket.rho2 = n_en * Ft * 1 * cos(cant_angle);              	            % thrust (upper bound), Newtons
rocket.rho1 = n_en * Ft * 0.81 * cos(cant_angle);       	                    % lowest throttleability (lower bound), Newtons

% initial and final conditions -- dynamics  [x y z]
rocket.r_0 = r_0;	                                                        % position vector, m
rocket.v_0 = v_0;		                                                    % velocity vector, m/s
rocket.r_N = r_f;				                                            % terminal position, m (zero for soft landing problem)
rocket.v_N = v_f;				                                            % terminal velocity, m (zero for soft landing problem)

rocket.dt = 5;            		                                            % period of calculation
rocket.a = 1/(Isp*g0*cos(cant_angle));				                        % alpha used for fuel mass consumption

% e_x = [1,0,0];                                                              % x-direction unit vector
% e_y = [0,1,0];                                                              % y-direction unit vector
% e_z = [0,0,1];                                                              % z-direction unit vector (altitude +ve normal away from surface)
% g0 = 9.807;					                                                % Earth gravity, m/s^2
% rocket.g_europa = -3.71  * e_z;		                                    % Europa's gravity, m/s^2
% rocket.gamma_p = 45;				                                        % pointing angle 
% cant_angle = 0 * pi/180;                                                    % The cant angle off the thrust line
% n_en = 1;                                                                   % The number of engine				                                                            % specific impulse, s
% rocket.m_d = m_dry;             	                                        % drymass, kg
% m_f = m_fuel;              	                                                % fuel mass, kg
% rocket.m_t = m_f + rocket.m_d;			            
% rocket.rho2 = n_en * Ft * 0.8 * cos(cant_angle);              	            % thrust (upper bound), Newtons
% rocket.rho1 = n_en * Ft * 0.2 * cos(cant_angle);       	                    % lowest throttleability (lower bound), Newtons
% 
% % initial and final conditions -- dynamics  [x y z]
% rocket.r_0 = r_0;	                                                        % position vector, m
% rocket.v_0 = v_0;		                                                    % velocity vector, m/s
% rocket.r_N = r_f;				                                            % terminal position, m (zero for soft landing problem)
% rocket.v_N = v_f;				                                            % terminal velocity, m (zero for soft landing problem)
% 
% rocket.dt = 1;            		                                            % period of calculation
% rocket.a = 1/(Isp*g0*cos(cant_angle));				                        % alpha used for fuel mass consumption

% Define tolerance
tol = 1e0;

% Compute tf_min and tf_max
tf_min = rocket.m_d * norm(rocket.v_0) / rocket.rho2;
tf_max = m_f / (rocket.a * rocket.rho1);

% Define the function to be minimized
f = @(t_f, rocket) solve_pdg_fft_descent(t_f, rocket).cost;

% Perform the golden section search
t_opt = golden(f, tf_min, tf_max, tol, rocket);

end