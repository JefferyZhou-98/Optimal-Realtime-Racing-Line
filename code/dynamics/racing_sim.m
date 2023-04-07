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

W = 0.7;
L = 1.5;
H = 0.5;
m_t = 50; 
m_b = 40;
m_w = 5;
r = 0.25;
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
T_L_0 = 80; % Nm
T_R_0 = 80; % Nm
% slipping condition
k_slip = 1;

% dynamic simulation ------------------------------------------------------
% storing values
x_save = ones(1,10); x_save(1) = x_0;
y_save = ones(1,10); y_save(1) = y_0;
theta_save = ones(1,10); theta_save(1) = theta_0;
omegaL_save = ones(1,10); omegaL_save(1) = omegaL_0;
omegaR_save = ones(1,10); omegaR_save(1) = omegaR_0;
vx_save = ones(1,10); vx_save(1) = vx_0;
vy_save = ones(1,10); vy_save(1) = vy_0;

t = linspace(0, 5, 10);
dt = t(2) - t(1);
for i = 1:length(t) - 1
    x_init = [x_save(i); y_save(i); theta_save(i); 
              omegaR_save(i); omegaL_save(i); vx_save(i); vy_save(i)];
    u_init = [T_L_0; T_R_0];
    s_dd = racing_ode(x_init, u_init, k_slip);
    % grabbing the second order terms
    x_dd = s_dd(1); y_dd = s_dd(2); theta_dd = s_dd(3);
    % propagation
    x_save(i+1) = x_save(i) + Vx_k*dt + 0.5*x_dd*(dt^2);
    y_save(i+1) = y_save(i) + Vy_k*dt + 0.5*y_dd*(dt^2);
    theta_save(i+1) = theta_save(i) + theta_d_k*dt + 0.5*theta_dd*(dt^2);

    % update
    Vx_k = Vx_k + x_dd*dt;
    Vy_k = Vy_k + y_dd*dt;
    theta_d_k = theta_d_k + theta_dd*dt;
end

% waypoint assignment -----------------------------------------------------
% x_waypoints = ones(1, 40);
% y_waypoints = ones(1, 40);
% % importing track
% [x_track, y_track] = racetrack_generation_2();
% figure(1)
% plot(x_track,y_track, linewidth=4, Color='k')
% axis equal
% axis padded
% hold on
% for i = 1:length(x_track) - 10
%     if rem(i, 10) == 0 || i == 1
%         x_waypoints(i) = x_track(i);
%         y_waypoints(i) = y_track(i);
%         [x_poly, y_poly, p] = track_sensor_new(i, i);
%         y_plot = p(1)*x_poly.^2 + p(2)*x_poly + p(3);
%         scatter(x_waypoints(i), y_waypoints(i), 'filled', LineWidth=40, MarkerFaceColor="#D95319");
%         hold on
%         plot(x_poly, y_plot, LineStyle=":", LineWidth=3.5);
%         drawnow
%         legend("Race Track","Waypoints","Polyfit Trajectory", fontsize=15)
%     end
% end

% plotting
[x_track, y_track] = racetrack_generation_2();
figure(1)
plot(x_track,y_track, linewidth=4, Color='k')
axis equal
axis padded
hold on
plot(x_save, y_save, LineStyle="--", LineWidth=1.8)