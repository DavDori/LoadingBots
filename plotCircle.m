function plotCircle(x,y,r)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
    ang=0:pi/10:2*pi; 
    xp = r * cos(ang);
    yp = r * sin(ang);
    plot(x + xp, y + yp, 'Color', 'Yellow', 'LineWidth', 1);
end

