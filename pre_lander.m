function [t_opt,rocket] = pre_lander(Ft, m_dry, m_fuel, Isp, r_0, v_0)

% constants and vehicle parameters
e_x = [1,0,0];                                                              % x-direction unit vector
e_y = [0,1,0];                                                              % y-direction unit vector
e_z = [0,0,1];                                                              % z-direction unit vector (altitude +ve normal away from surface)
g0 = 9.807;					                                                % Earth gravity, m/s^2
rocket.g_europa = -1.315 * e_z;		                                        % Europa's gravity, m/s^2
rocket.gamma_p = 60;				                                        % pointing angle 
cant_angle = 15 * pi/180;                                                   % The cant angle off the thrust line
n_en = 8;                                                                   % The number of engine
rocket.m_d = m_dry;             	                                        % drymass, kg
m_f = m_fuel;              	                                                % fuel mass, kg
rocket.m_t = m_f + rocket.m_d;			                                    % total mass, kg
rocket.rho2 = n_en * Ft * 1 * cos(cant_angle);              	            % thrust (upper bound), Newtons
rocket.rho1 = n_en * Ft * 0.34 * cos(cant_angle);       	                % lowest throttleability (lower bound), Newtons

% initial and final conditions -- dynamics  [x y z]
rocket.r_0 = r_0;	                                                        % position vector, m
rocket.v_0 = v_0;		                                                    % velocity vector, m/s
rocket.r_N = [0; 0; 0];				                                        % terminal position, m (zero for soft landing problem)
rocket.v_N = [0; 0; 0];				                                        % terminal velocity, m (zero for soft landing problem)

% glide slop constraint ( creating all four facets )
gs = 86 * pi/180;					    	                                % glides slope angle constraint

% normal vectors for the four facets constraint
n_1 = [cos(gs); 0; -sin(gs)];
n_2 = [0; cos(gs); -sin(gs)];
n_3 = [-cos(gs); 0; -sin(gs)];
n_4 = [0; -cos(gs); -sin(gs)];

rocket.H_gs = [n_1';n_2';n_3';n_4'];
rocket.h_gs = zeros(4, 1);

% other timing and constraints
rocket.dt = 1;            		                                            % period of calculation
rocket.a = 1/(Isp*g0*cos(cant_angle));				                        % alpha used for fuel mass consumption

% Define tolerance
tol = 1e0;

% Compute tf_min and tf_max
tf_min = rocket.m_d * norm(rocket.v_0) / rocket.rho2;
tf_max = m_f / (rocket.a * rocket.rho1);

% Define the function to be minimized
f = @(t_f, rocket) solve_pdg_fft_lander(t_f, rocket).cost;

% Perform the golden section search
t_opt = golden(f, tf_min, tf_max, tol, rocket);

end