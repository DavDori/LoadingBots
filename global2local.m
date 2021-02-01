function [x_local] = global2local(x_reference, x_global)
%GLOBAL2LOCAL change from a global reference frame to a local frame
    x_local = x_global - x_reference;
end

