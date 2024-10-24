f = @(x) [0.5*P.rho*P.Va0^2*P.S_wing*(P.C_L_0 + P.C_L_alpha*x(1) + P.C_L_delta_e*x(2)) - P.mass*P.gravity*cos(x(3));
          0.5*P.rho*P.Va0^2*P.S_wing*(P.C_D_0 + P.C_D_alpha*x(1) + P.C_D_delta_e*x(2)) + P.mass*P.gravity*sin(x(3));
          0.5*P.rho*P.Va0^2*P.S_wing*P.c*(P.C_m_0 + P.C_m_alpha*x(1) + P.C_m_delta_e*x(2))];
x0 = [3 3 1];
[a, fval] = fsolve(f,x0)
