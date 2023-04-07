function test_dd = racing_ode(x, u, k_slip)
% importing the physical parameters
global L H m_t m_b m_w r W k g dt
% defining the states and inputs
x_pos = x(1);
y_pos = x(2);
theta = x(3);
omega_R = x(4);
omega_L = x(5);
x_dot = x(6);
y_dot = x(7);


% torque applied to the wheels
T_L = u(1);
T_R = u(2);

% randomized coefficient of friction scale (0 - 1) 
% where K_f = k on dry track
k_f = k*k_slip;

% moment of inertia of wheels
Iyy_b = m_w*(r^2)/2;
% moment of inertia of the body
Izz_b = (1/12)*m_t*(H^2 + W^2);

% ode:
F_tot = (T_L/(r)) + (T_R/(r)) - k_f*m_t*g;
x_dd = (F_tot/m_t)*cos(theta);
y_dd = (F_tot/m_t)*sin(theta);
theta_dd = ((L/2)/Izz_b)*((T_R/(r)) - (T_L/(r)));

test_dd = [x_dd; y_dd; theta_dd];
end