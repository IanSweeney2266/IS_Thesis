% Parameters.m
% Defines the physical parameters of the car and of the system model for
% use in Simulink.

%% Test and plot the magic tyre formula
gamma = -1:0.001:1;
mu = zeros(1, length(gamma));
for i = 1:length(gamma)
    mu(i) = magic_formula(gamma(i));
end

%figure(1);
%plot(gamma, mu);

%% Dynamics
% gamma_f = (V - w_f*R) / V;
% w_f_dot = (T_app + R*F_x_f - w_f*C_f) / J_tire;
clear;

%% Vehicle Constants

g = 9.81;       %[m/s] Gravitational Constant

M = 3.832;      %[kg] Total Mass
W = M*g;        %[N] Total Weight
R = 0.055;      %[m] Tire Radius (ProgreSSIV)
H = 0.075;      %[m] CG Height

%J = 0.5*0.25*R^2;   %[kg-m^2] Estimated from theory
%M_eq = (1/2)*M*R^2 + J; %[kg-m^2] equivalent inertia for the tire+car (unused)

l_front = 0.1968;   %[m] Distance from front axle to CG
l_rear = 0.2042;    %[m] Distance from rear axle to CG
l = l_front + l_rear;

%J = 0.00015;        %[kg-m^2] From wheel acc test 
J = 0.001;        %[kg-m^2] From wheel acc test 

b_wheel = 5e-4;     %[Nm-s] From wheel acc test
%b_drag = 1;


%% Motor Parameters
Rm = 0.608;     %[Ohm] Motor resistance
Lm = 0.463e-3;  %[H] Motor inductance
Km = 36.9e-3;   %[Nm/A] Motor torque constant
V_max = 24;     %[V] Max voltage


%% Initial Conditions
V_0 = 1; %[m/s]
slip = 0.00;
w_0 = (1-slip)*V_0/R;

%% Inputs
w_max = 6110/60*2*pi; %[rad/s] No load speed
T_max = 1460/1000;  %[Nm] Stall torque
T_acc = 128/1000;   %[Nm] Acceleration Torque
T_dec = T_max;      %[Nm] Deceleration Torque
t_dec = 5;          %[s] Deceleration start
t_sim = 8;          %[s] Simulation Time

v1 = 2.5;           %[m/s] velocity setpoint 1


%% Wheel SS

% x = [wf, if, wr, ir]
WheelSS_A = [-b_wheel/J, Km/J, 0, 0;
             -Km/J, -Rm/Lm, 0, 0;
             0, 0, -b_wheel/J, Km/J;
             0, 0, -Km/J, -Rm/Lm];
   
% u = [Ff, Fr, Vsf, Vsr]
WheelSS_B = [-R/J, 0, 0, 0;
             0, 0, 1/Lm, 0;
             0, -R/J, 0, 0;
             0, 0, 0, 1/Lm];
         
WheelSS_C = [1, 0, 0, 0;
             0, 0, 1, 0];
         
WheelSS_D = [0, 0, 0, 0;
             0, 0, 0, 0];
         
WheelSS_ICs = [w_0, 0, w_0, 0];      


%% Run simulation

sim('Model_v13.slx');
 


