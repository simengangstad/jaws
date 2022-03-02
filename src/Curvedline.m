%for square 100x100
scale = 1;
frequency = 1;
wpt.pos.x = scale*[0 75 75 25 25 75 75 25 25 75 75 0];
wpt.pos.y = scale*[0 0 20 20 40 40 60 60 80 80 100 100];
wpt.time = frequency*[0 10 32 42 67 77 100 110 135 145 173 193];
t = 0:1:max(wpt.time);

x_p = pchip(wpt.time, wpt.pos.x,t); %Straight lines from point to point
y_p= pchip(wpt.time,wpt.pos.y,t); 
x_s = spline(wpt.time, wpt.pos.x, t); %The curved line between points
y_s = spline(wpt.time, wpt.pos.y,t);

%Position of the vessel
vesselPosx = 34.4;
vesselPosy = 25;


dist = zeros([1 length(x_s)]);
for i= 1:length(x_s)
    dist(i) = sqrt((x_s(i)-vesselPosx)^2+(y_s(i)-vesselPosy)^2);
end

[~,idx] = min(dist); %returns the index of the minimum distance
disp(idx)
minPointx = x_s(idx);
minPointy= y_s(idx);
subplot(311), plot(wpt.time, wpt.pos.x, 'o', t, [x_p; x_s]), title("x-position");
subplot(312), plot(wpt.time, wpt.pos.y, 'o', t, [y_p; y_s]),title("y-position");
subplot(313), plot(wpt.pos.x, wpt.pos.y,'o',minPointx,minPointy,'o',vesselPosx,vesselPosy,'o',x_p, y_p,x_s,y_s), title("xy-position");

