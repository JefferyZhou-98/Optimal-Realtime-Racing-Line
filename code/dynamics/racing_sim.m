clear all;clc
% setting the parameters --------------------------------------------------
global L H m_t m_b m_w r W k g dt
% W: width of cart (m)
% length of cart (m)
% height of cart (m)
% m_t: total mass (kg)
% m_b: mass of robot body (kg)
% m_w: mass of wheels (kg)
% r: radius of wheels (kg)
% W: width of cart (m)
% k: friction coefficient (dry condition) 
% g: gravitational constant (kg/m2)

W = 2;
L = 5.5;
H = 1;
m_t = 1000; 
m_b = 600;
m_w = 200;
r = 0.3175;
k = 1.8;
g = 9.81; 

% initial conditions ------------------------------------------------------
% states
x_0 = 1000; 
y_0 = 0;
theta_0 = pi; 
phi_r_0 = 0;
phi_l_0 = 0;

vx_0 = 0; Vx_k = vx_0;
vy_0 = 0; Vy_k = vy_0;
theta_d_0 = 0; theta_d_k = theta_d_0;
% inputs
T_L_0 = 1000; % Nm
T_R_0 = 2000; % Nm
% slipping condition
k_slip = 1;


% storing values
x_track = ones(1,10); x_track(1) = x_0;
y_track = ones(1,10); y_track(1) = y_0;
theta_track = ones(1,10); theta_track(1) = theta_0;

t = linspace(0, 5, 10);
dt = t(2) - t(1);
for i = 1:length(t) - 1
    x_init = [x_track(i); y_track(i); theta_track(i); phi_l_0; phi_r_0];
    u_init = [T_L_0; T_R_0];
    s_dd = racing_ode(x_init, u_init, k_slip);
    % grabbing the second order terms
    x_dd = s_dd(1); y_dd = s_dd(2); theta_dd = s_dd(3); omega_l_d = s_dd(4); omega_r_d = s_dd(5);
    % propagation
    x_track(i+1) = x_track(i) + Vx_k*dt + 0.5*x_dd*(dt^2);
    y_track(i+1) = y_track(i) + Vy_k*dt + 0.5*y_dd*(dt^2);
    theta_track(i+1) = theta_track(i) + theta_d_k*dt + 0.5*theta_dd*(dt^2);
    % update
    Vx_k = Vx_k + x_dd*dt;
    Vy_k = Vy_k + y_dd*dt;
    theta_d_k = theta_d_k + theta_dd*dt;

    % storing
end

% import track
figure(1)
[x, y] = track(1,1);
plot(x,y, linewidth=2)
hold on
plot(x_track, y_track, LineStyle=":", LineWidth=2.3)
axis equal
axis padded


    
