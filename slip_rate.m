function gamma = slip_rate(args)

    v = args(1);
    w = args(2);
    R = args(3);

    if (w*R) > v
        if w == 0
            gamma = 0.01;
        else
            gamma = (w*R - v)/(w*R); % Acceleration
        end
    else
        if v == 0
            gamma = -0.01;
        else 
            gamma = (w*R - v)/(v); % Breaking
        end
    end

end
