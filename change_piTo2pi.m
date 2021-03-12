function angle = change_piTo2pi(phi)
% given an angle between -pi and pi, returns an angle between 0 and 2pi
    if(phi < 0)
        angle = 2*pi + phi;
    else
        angle = phi;
    end  
end