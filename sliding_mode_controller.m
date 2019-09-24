function T_out = sliding_mode_controller(args)
%SMC Sliding mode controller for slip rate of tires
%   Implements as sliding mode controller...
%   Inputs args = [sr, w, mu, T_dec]
    
    sr = args(1);
    w = args(2);
    T_dec = args(3);
    
    sr_set = -0.1;
    epsilon = 1e-2;
    
    sd = (sr - sr_set);
    
    if sd > 0
        T_out = T_dec;
    else
        T_out = 0;
    end
end

