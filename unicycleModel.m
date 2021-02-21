function [stateDot] = unicycleModel(cmd,x) 
    %{
    unicycle movement model:
    x(t+∆t) = x(t) + cos(θ(t))v∆t
    y(t+∆t) = y(t) + sin(θ(t))v∆t
    θ(t+∆t) = θ(t) + ω∆t
    %}
    xDot = cos(x(3)) * cmd(1);
    yDot = sin(x(3)) * cmd(1);
    thetaDot = cmd(2);
    stateDot = [xDot; yDot; thetaDot];
end

