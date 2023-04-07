x1 = flip(linspace(0,20,27));
y1 = -(20^2 - (x1-20).^2).^0.5;

x2 = (0:20);
y2 = (20^2 - (x2-20).^2).^0.5;

x3 = (20:50);
y3 = zeros(1,31) + 20;

x4 = (50:60);
y4 = -(10^2 - (x4-50).^2).^0.5 + 30;

x5 = (60:70);
y5 = (10^2 - (x5-70).^2).^0.5 + 30;

x6 = (70:110);
y6 = zeros(1,length(x6)) + 40;

x7 = (110:130);
y7 = (20^2 - (x7-110).^2).^0.5 + 20;

x8 = zeros(1,81) + 130;
y8 = flip(-60:20);

x9 = flip(110:130);
y9 = -(20^2 - (x9-110).^2).^0.5 - 60; 

x10 = flip(80:110);
y10 = zeros(1,31) - 80;

x11 = flip(70:80);
y11 = -(10^2 - (x11-80).^2).^0.5 - 70;

x12 = zeros(1,41) + 70;
y12 = (-70:-30);

x13 = flip(60:70);
y13 = (10^2 - (x13-60).^2).^0.5 - 30;

x14 = flip(20:60);
y14 = zeros(1,41) - 20;

x = [x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14];
y = [y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11,y12,y13,y14];

coords = [x;y]';

plot(x,y,'LineWidth',2)

hold on
saved_trajectory = zeros(20,3);
coords_temp = zeros(400,2);
count=1;
for i=1:20:400
    if i > 380
        x_temp = x(i:length(x));
        y_temp = y(i:length(y));        
    else
        x_temp = x(i:i+20);
        y_temp = y(i:i+20);
    end
    coordinates = [x_temp;y_temp]';
    trajectory_temp = trajectory_generation(coordinates);

    %y_plot = polyval(x_temp,trajectory_temp);
    y_plot = trajectory_temp(1)*x_temp.^2 + trajectory_temp(2)*x_temp + trajectory_temp(3);
    plot(x_temp,y_temp,'LineWidth',3)
    saved_trajectory(count,1) = trajectory_temp(1);
    saved_trajectory(count,2) = trajectory_temp(2);
    saved_trajectory(count,3) = trajectory_temp(3);
    
    hold on
    pause(0.05)
    count = count+1;
end

upper_fric = 0.6;
lower_fric = 0.1;

friction_outside = (upper_fric - lower_fric).*rand(20,1) + lower_fric;
friction_inside = (upper_fric - lower_fric).*rand(20,1) + lower_fric;

    
function trajectory = trajectory_generation(coordinates)

    trajectory = polyfit(coordinates(:,1),coordinates(:,2),2);

end
    
    