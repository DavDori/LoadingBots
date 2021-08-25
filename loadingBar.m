function msg = loadingBar(current, final, n_bars, symbol)
    % represent a loading bar for the simulation
    perc = fix(current / final * 100);
    current_bar = floor(perc * n_bars / 100);
    
    msg = '|';
    filler = '';
    for i = 1:current_bar
        msg = strcat(msg, symbol);
    end
    for i = 1:(n_bars-current_bar)
        filler = strcat(filler, '-');
    end
    
    msg = strcat(msg, filler, '| ', string(perc), '%');
end

