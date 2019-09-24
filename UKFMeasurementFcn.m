function y = UKFMeasurementFcn(x)
% Unscented Kalman Filter State Transition Function
% x = [v, wf, wr, if, ir]
% dt = 0.01s;

%% Define some constants

g = 9.81;       %[m/s] Gravitational Constant

M = 3.832;      %[kg] Total Mass
W = M*g;        %[N] Total Weight
R = 0.055;      %[m] Tire Radius (ProgreSSIV)
H = 0.075;      %[m] CG Height

l_front = 0.1968;   %[m] Distance from front axle to CG
l_rear = 0.2042;    %[m] Distance from rear axle to CG
l = l_front + l_rear;

J = 0.001;        %[kg-m^2] From wheel acc test 
b_wheel = 5e-4;     %[Nm-s] From wheel acc test

Rm = 0.608;     %[Ohm] Motor resistance
Lm = 0.463e-3;  %[H] Motor inductance
Km = 36.9e-3;   %[Nm/A] Motor torque constant

%% Update the state vector
% x = [v, wf, wr, if, ir]
dt = 0.01;

mu_f = magic_formula(slip_rate([x(1), x(2), R]));
mu_r = magic_formula(slip_rate([x(1), x(3), R]));

N_f = W/4;
N_r = W/4;

y = (2/M)*(N_f*mu_f + N_r + mu_r);


end

