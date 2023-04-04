function test_dd = racing_ode_new(x, u, k_slip)
% importing the physical parameters
global L H m_t m_b m_w r W k g dt
% defining the states and inputs
x_pos = x(1);
y_pos = x(2);
theta = x(3);

% torque applied to the wheels
T_L = u(1);
T_R = u(2);

% randomized coefficient of friction scale (0 - 1) 
% where K_f = k on dry track
k_f = k*k_slip;

% moment of inertia
Iyy_b = m_w*(r^2)/2;

% angular acceleration ----------------------------------------------------
alpha_L = T_L/Iyy_b; alpha_R = T_R/Iyy_b;
omega_L = alpha_L*dt; omega_R = alpha_R*dt;

% Total force
F = (T_L/r) + (T_R/r) - k_f*m_t*g;

% linear acceleration -----------------------------------------------------
acc = F/m_t;

% thetadot ----------------------------------------------------------------
theta_dot = r*(omega_R - omega_L)/L;

% absolute speed ----------------------------------------------------------
V_r = omega_R*r; V_l = omega_L*r;
speed = (V_r + V_l)/2;

test_dd = [speed; acc; theta_dot; alpha_L; alpha_R];
end