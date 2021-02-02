function [x_global] = local2global(x_reference, x_local)
%GLOBAL2LOCAL change from a global reference frame to a local frame
    x_global = x_local + x_reference;
end