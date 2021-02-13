function [index] = findStructFromName(struct_ary, name)
%FINDSTRUCTFROMNAME form an array of structures search the first element 
%with the specified name
    index = NaN; % element not found flag
    for i=1:length(struct_ary)
        if(strcmp(struct_ary(i).name, name) == true)
            index = i;
            break;  
        end
    end
end

