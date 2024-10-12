function [x_sol, f_sol] = golden(f, a, b, tol, rocket)
    % Golden search for minimizing a unimodal function f(x) on the
    % interval [a, b] to within a prescribed tolerance in x.
    
    phi = (1 + sqrt(5)) / 2;
    n = ceil(log((b - a) / tol) / log(phi) + 1);
    rho = phi - 1;
    d = rho * b + (1 - rho) * a;
    yd = f(d,rocket);
    
    for i = 1:n-1
        c = rho * a + (1 - rho) * b;
        yc = f(c,rocket);
        if yc < yd
            b = d;
            d = c;
            yd = yc;
        else
            a = b;
            b = c;
        end
        bracket = sort([a, b, c, d]);
        fprintf('Golden bracket: [%.3f, %.3f, %.3f, %.3f]\n', bracket);
    end
    
    x_sol = (a + b) / 2;
    f_sol = f(x_sol,rocket);
end