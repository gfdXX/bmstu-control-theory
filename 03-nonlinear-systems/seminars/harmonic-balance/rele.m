function y = rele(c, b, d, x)
    persistent state; % сохранение состояния для гистерезиса
    if isempty(state)
        state = 0;
    end

    if x >= state + b
        y = c;
        state = y;
    elseif x <= state - b
        y = -c;
        state = y;
    else
        y = state;
    end

    if x > state - d && x < state + d
        y = (c/(2*d)) * x;
    end
end
