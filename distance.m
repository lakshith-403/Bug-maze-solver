% fit polynomial to distance vs reading curve of proximity sensor

y = [6, 5, 4, 3, 2, 1, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.01];
x = [67, 96, 127,166, 238, 460, 475, 546,574,667,849,984,1083,1352,1600];

p = polyfit(x,y,5);

x1 = linspace(0,2000);
y1 = polyval(p,x1);
figure
plot(x,y,'o')
hold on
plot(x1,y1)
hold off

disp(p)