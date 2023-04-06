x1 = flip(0:20);

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



function trajectory = trajectory_generation(coordinates)

    trajectory = polyfit(coordinates(:,1),coordinates(:,2),3);

end
    
    