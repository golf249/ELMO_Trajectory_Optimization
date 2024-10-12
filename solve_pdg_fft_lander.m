function sol = solve_pdg_fft_lander(t_f, rocket)
    % solve_pdg_fft Solves the optimal control problem for the rocket landing.
    %
    % Inputs:
    %   rocket - Struct containing rocket parameters and initial conditions.
    %   t_f - Final time for the trajectory.
    %
    % Outputs:
    %   sol - Struct containing the optimized variables and cost.
    
    % Extract parameters from the rocket struct
    r_0 = rocket.r_0;               % Initial position
    v_0 = rocket.v_0;               % Initial velocity
    r_N = rocket.r_N;               % Final position
    v_N = rocket.v_N;               % Final velocity
    m_t = rocket.m_t;               % Initial mass (wet mass)
    m_d = rocket.m_d;              	% Dry mass
    g_europa = rocket.g_europa;     % Gravity on Europa
    a = rocket.a;                   % Specific impulse parameter
    rho1 = rocket.rho1;             % Minimum thrust
    rho2 = rocket.rho2;             % Maximum thrust
    gamma_p = rocket.gamma_p;       % Maximum thrust vector angle in degrees
    H_gs = rocket.H_gs;             % Glide slope constraint matrix
    h_gs = rocket.h_gs;             % Glide slope constraint bound
    
    dt = rocket.dt;            		% period of calculation
    N = 1 + floor(t_f/dt);			% calculation steps

    % Define CVX optimization problem
    cvx_begin quiet
        cvx_solver sedumi
        cvx_precision best
        variables u(3,N) z(1,N) s(1,N) r(3,N) v(3,N)
        % u : (Thrust / mass) or acceleration
        % z : log(m_total)
        % s : Slack variable / mass
        % r : Position vector
        % v : Velocity vector
        minimize(sum(z) * dt) % Objective function
        subject to
            % Initial conditions
            r(:,1) == r_0;
            v(:,1) == v_0;
            z(1) == log(m_t);

            % Terminal conditions
            r(:,N) == r_N;
            v(:,N) == v_N;

            % Constraints
            r(3,:) >= 0;
            z(1,:) >= log(m_d);
            u(3,N) == s(1, N) .* cos(deg2rad(0));
            %u(3,1) == s(1, 1) .* cos(deg2rad(15));

            % Dynamics constraints
            for k = 1:N-1
                r(:,k+1) == r(:,k) + ((dt/2)*(v(:,k) + v(:,k+1))) + (((dt^2)/12)*(u(:,k+1) - u(:,k)));
                v(:,k+1) == v(:,k) + ((dt/2)*(u(:,k) + u(:,k+1))) + (g_europa'*dt);
                z(1,k+1) == z(1,k) - (((a*dt)/2)*(s(1,k) + s(1,k+1)));
            end

            % Additional constraints
            for k = 1:N
                H_gs * r(:,k) <= h_gs; % glideslope constraint
                u(3,k) >= s(1, k) .* cos(deg2rad(gamma_p)); % pointing constraint
                norm(u(:,k)) <= s(1,k);

                z_0 = m_t - (a * rho2 * dt * (k - 1));
                z1 = log(m_t - (a * rho1 * dt * (k - 1)));
                z0 = log(z_0);
                z0 <= z(1,k) <= z1;

                m_1 = rho1 / z_0;
                m_2 = rho2 / z_0;   
                m_1 * (1 - (z(1,k) - z0) + ((z(1,k) - z0)^2) / 2) <= s(1,k) <= m_2 * (1 - (z(1,k) - z0));
            end
    cvx_end
    
    m_used = m_t;
	m = exp(z);
	if strcmp(cvx_status, 'Solved')
	    m_used = m(1) - m(N);
        fprintf('%s \n', cvx_status);
    else
        m_used = m(1) - m(N);
	    fprintf('Error! %s \n', cvx_status);
	end
    
    % Output the solutions
    sol.u = u;
    sol.z = z;
    sol.s = s;
    sol.r = r;
    sol.v = v;
    sol.cost = -z(N);
    sol.N = N;
    sol.m_used = m_used;
    sol.t_f = t_f;
end
