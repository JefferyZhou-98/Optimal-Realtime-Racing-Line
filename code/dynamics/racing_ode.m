function x_dd = racing_ode(x, u, k_slip)
% importing the physical parameters
global L H m_t m_b m_w r W k g dt
% defining the states and inputs
x_pos = x(1);
y_pos = x(2);
theta = x(3);
phi_l = x(4);
phi_r = x(5);
% torque applied to the wheels
T_L = u(1);
T_R = u(2);

% randomized coefficient of friction scale (0 - 1) 
% where K_f = k on dry track
k_f = k*k_slip;

% state vectors
q = [x_pos; y_pos; theta; phi_l; phi_r];
% distance between inertial frame and body frame 
d = sqrt(x_pos^2 + y_pos^2);

% inertia tensor of the cart (rectangle) only Izz 
I_body = (1/12)*m_b*(W^2 + L^2);

% inertia tensor of the wheels in body frame
Ixx_b = m_w*(r^2)/4; Iyy_b = m_w*(r^2)/2; Izz_b = m_w*(r^2)/4;

% state vector derivative
omega_L = (T_L/Iyy_b)*dt;
omega_R = (T_R/Iyy_b)*dt;
q_dot = (r/2)*[(omega_L - omega_R)*cos(theta);
               (omega_L - omega_R)*sin(theta);
               -(omega_L + omega_R)/W];
x_d = q_dot(1); y_d = q_dot(2); theta_d = q_dot(3); 

% total inertia
I_T = I_body + m_b*d^2 + 2*m_w*W^2 + 2*Izz_b;

% Lagrangian form: M(q) * q_dd + B(q, q_d) = F
M_q = [m_t, 0, -m_b*d*sin(theta), 0, 0;
       0, m_t, m_b*d*cos(theta), 0, 0;
       -m_b*d*sin(theta), m_b*d*cos(theta), I_T, 0, 0;
       0, 0, 0, Iyy_b, 0;
       0, 0, 0, 0, Iyy_b];

B_q = -m_b*(theta_d^2)*[cos(theta); sin(theta); 0; 0; 0];

F = [0; 0; 0; (T_R/r) - k_f*m_t*g/2; (T_L/r) - k_f*m_t*g/2];

x_dd = M_q\(F - B_q);
end