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
m_t = 400; 
m_b = 360;
m_w = 30;
r = 0.35;
k = 0.7;
g = 9.81; 

% initial conditions ------------------------------------------------------
% states
x_0 = 60; 
y_0 = -20;
theta_0 = pi; theta_k = theta_0;

vx_0 = 0; Vx_k = vx_0;
vy_0 = 0; Vy_k = vy_0;
theta_d_0 = 0; theta_d_k = theta_d_0;
omegaR_0 = 0; omegaR_k = omegaR_0;
omegaL_0 = 0; omegaL_k = omegaL_0;
% inputs
T_L_0 = 800; % Nm
T_R_0 = 400; % Nm
% slipping condition
k_slip = 1;

% dynamic simulation ------------------------------------------------------
% storing values
x_save = ones(1,10); x_save(1) = x_0;
y_save = ones(1,10); y_save(1) = y_0;
theta_save = ones(1,10); theta_save(1) = theta_0;
omegaL_save = ones(1,10); omegaL_save(1) = omegaL_0;
omegaR_save = ones(1,10); omegaR_save(1) = omegaR_0;

% t = linspace(0, 5, 10);
% dt = t(2) - t(1);
% for i = 1:length(t) - 1
%     x_init = [x_save(i); y_save(i); theta_save(i); omegaR_save(i); omegaL_save(i)];
%     u_init = [T_L_0; T_R_0];
%     s_dd = racing_ode_new(x_init, u_init, k_slip);
%     % grabbing the second order terms
%     speed_k = s_dd(1); acc = s_dd(2); theta_dot_k = s_dd(3); alpha_L = s_dd(4); alpha_R = s_dd(5);
%     % propagation
%     x_save(i+1) = x_save(i) + (speed_k*dt + 0.5*acc*(dt^2))*cos(theta_k);
%     y_save(i+1) = y_save(i) + (speed_k*dt + 0.5*acc*(dt^2))*sin(theta_k);
%     theta_save(i+1) = theta_save(i) + theta_dot_k*dt;
%     omegaL_save(i+1) = omegaL_save(i) + alpha_L*dt;
%     omegaR_save(i+1) = omegaR_save(i) + alpha_R*dt;
%     % update
%     theta_k = theta_k + theta_dot_k*dt;
%     speed_k = speed_k + acc*dt;
% end

% waypoint assignment -----------------------------------------------------
x_waypoints = ones(1, 40);
y_waypoints = ones(1, 40);
% importing track
beta = pi/3;
[x_track, y_track] = race_track(beta);
figure(1)
% plot(x_track,y_track, linewidth=4, Color='k')
axis equal
axis padded
hold on
for i = 1:length(x_track) - 10
    if rem(i, 10) == 0 || i == 1
        x_waypoints(i) = x_track(i);
        y_waypoints(i) = y_track(i);
        [x_poly, y_poly, p] = track_sensor(i, i, beta);
%         y_plot = p(1)*x_poly.^2 + p(2)*x_poly + p(3);
        y_plot = polyval(p, x_poly);
        scatter(x_waypoints(i), y_waypoints(i), 'filled', LineWidth=40, MarkerFaceColor="#D95319");
        hold on
        plot(x_poly, y_plot, LineStyle=":", LineWidth=3.5);
        drawnow
        legend("Race Track","Waypoints","Polyfit Trajectory", fontsize=15)
    end
end

% % plotting
% [x_track, y_track] = race_track();
% figure(1)
% plot(x_track,y_track, linewidth=4, Color='k')
% axis equal
% axis padded
% hold on
% plot(x_save, y_save, LineStyle="--", LineWidth=1.8)