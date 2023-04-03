function [x, y] = track(a, b)
% a: current x position of the car
% b: current y positino of the car
% section 1
x1 = linspace(2000,0, 100);
y1 = zeros(1, 100);

% section 2
x2 = ones(1, 100); y2 = ones(1, 100);
r1 = 400;
theta_2 = linspace(0, 180, 100);
for i = 1:length(theta_2)
    if theta_2(i) >= 0 && theta_2(i) <= 90
        x2(i) = x1(end) - r1*sind(theta_2(i));
        y2(i) = r1 - r1*cosd(theta_2(i));
    else
        x2(i) = -sind(theta_2(i))*r1;
        y2(i) = r1 - r1*cosd(theta_2(i));
    end
end

% section 3
x3 = linspace(x2(end), 2000, 100);
y3 = y2(end)*ones(1, 100);

% section 4
x4 = ones(1, 100); y4 = ones(1, 100);
r2 = 400;
theta_4 = linspace(0, 180, 100);
for i = 1:length(theta_4)
    if theta_4(i) >= 0 && theta_4(i) <= 90
        x4(i) = x3(end) + r2*sind(theta_4(i));
        y4(i) = r2 - r2*cosd(theta_4(i));
    else
        x4(i) = x3(end) + sind(theta_4(i))*r1;
        y4(i) = r1 - r1*cosd(theta_4(i));
    end
end

% storing all values of x and y
x_tot = ones(1, 400);
y_tot = ones(1, 400);
x_tot(1:100) = x1; x_tot(101:200) = x2; x_tot(201:300) = x3; x_tot(301:400) = x4;
y_tot(1:100) = y1; y_tot(101:200) = y2; y_tot(201:300) = y3; y_tot(301:400) = y4;

% output
% x = x_tot(a:a+10); 
% y = y_tot(b:b+10);
x = x_tot;
y = y_tot;
end