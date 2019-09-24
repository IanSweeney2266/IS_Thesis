function mu = magic_formula(gamma)

% Surface defines the type of road surface.
% Values for B,C,D,E from Pacejka tyre model constants (Short et al., 2004) 
%   1 - Dry
%   2 - Wet
%   3 - Snow
%   4 - Icy
%   5 - Custom (Thomas Stevens)
%   6 - Custom (Ian Sweeney)

surface = 2;

B = [10, 12, 5, 4, 10, 10];         % stiffness factor
C = [1.9, 2.3, 2, 2, 0.09, 0.15];   % shape factor
D = [1, 0.82, .3, .1, 1, 1];        % peak value
E = [0.97, 1, 1, 1, 0.65, 0.65];    % curvature factor

Bi = B(surface);
Ci = C(surface);
Di = D(surface);
Ei = E(surface);


mu = Di*sin(Ci*atan(Bi*gamma - Ei*(Bi*gamma - atan(Bi*gamma))));
% mu = Di*sin(Ci*atan(Bi*(1-Ei)*gamma + Ei*atan(Bi*gamma)));
% mu = Fx_tire / Fz_tire