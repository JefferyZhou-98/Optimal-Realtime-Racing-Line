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
m_t = 500; 
m_b = 400;
m_w = 50;
r = 0.3175;
k = 1.8;
g = 9.81; 

% initial conditions ------------------------------------------------------
% states
x_0 = 0; 
y_0 = 0;
theta_0 = pi; theta_k = theta_0;
phi_r_0 = 0;
phi_l_0 = 0;

vx_0 = 0; Vx_k = vx_0;
vy_0 = 0; Vy_k = vy_0;
theta_d_0 = 0; theta_d_k = theta_d_0;
% inputs
T_L_0 = 2000; % Nm
T_R_0 = 2030; % Nm
% slipping condition
k_slip = 1;

% dynamic simulation ------------------------------------------------------
% storing values
x_save = ones(1,10); x_save(1) = x_0;
y_save = ones(1,10); y_save(1) = y_0;
theta_save = ones(1,10); theta_save(1) = theta_0;

t = linspace(0, 5, 10);
dt = t(2) - t(1);
for i = 1:length(t) - 1
    x_init = [x_save(i); y_save(i); theta_save(i)];
    u_init = [T_L_0; T_R_0];
    s_dd = racing_ode_new(x_init, u_init, k_slip);
    % grabbing the second order terms
    speed_k = s_dd(1); acc = s_dd(2); theta_dot_k = s_dd(3); alpha_L = s_dd(4); alpha_R = s_dd(5);
    % propagation
    x_save(i+1) = x_save(i) + (speed_k*dt + 0.5*acc*(dt^2))*cos(theta_k);
    y_save(i+1) = y_save(i) + (speed_k*dt + 0.5*acc*(dt^2))*sin(theta_k);
    % update    
    theta_k = theta_k + theta_dot_k*dt;

    % storing
end

% waypoint assignment -----------------------------------------------------
x_waypoints = ones(1, 20);
y_waypoints = ones(1, 20);
% importing track
[x_track, y_track] = track();
figure(1)
plot(x_track,y_track, linewidth=4, Color='k')
axis equal
axis padded
hold on
for i = 1:length(x_track) - 20
    if rem(i, 20) == 0 || i == 1
        x_waypoints(i) = x_track(i);
        y_waypoints(i) = y_track(i);
        [x_poly, y_poly, p] = track_sensor(i, i);
        y_plot = p(1)*x_poly.^2 + p(2)*x_poly + p(3);
        scatter(x_waypoints(i), y_waypoints(i), 'filled', LineWidth=4, MarkerFaceColor="#D95319");
        hold on
        plot(x_poly, y_plot, LineStyle=":", LineWidth=2.5);
        drawnow
    end
end

