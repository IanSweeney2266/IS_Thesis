% SMC_one_wheel.m
% A sliding mode controller for a single wheeled vehicle.


%% Vehicle Constants
clear;

g = 9.81;       %[m/s] Gravitational Constant

M = 3.832/4;    %[kg] Total Mass
W = M*g;        %[N] Total Weight
R = 0.055;      %[m] Tire Radius
H = 0.075;      %[m] CG Height

l_front = 0.1968;   %[m] Distance from front axle to CG
l_rear = 0.2042;    %[m] Distance from rear axle to CG
l = l_front + l_rear;

J = 0.00015;    %[kg-m^2] From wheel acc test
b = 0;       %[Nm-s] From wheel acc test


%% Motor Parameters
Rm = 0.608;     %[Ohm] Motor resistance
Lm = 0.463e-3;  %[H] Motor inductance
Km = 36.9e-3;   %[Nm/A] Motor torque constant 

w_max = 6110/60*2*pi;   %[rad/s] No load speed
T_max = 1460/1000;      %[Nm] Stall torque


%% Initial Conditions
v_0 = 5; %[m/s]
slip = 0.00;
w_0 = (1-slip)*v_0/R;


%% Running the model

dt = 0.0001;
t = 0:dt:3;
w = zeros(length(t), 1);
v = zeros(length(t), 1);
sr = zeros(length(t), 1);
sr_dot = zeros(length(t), 1);

w(1) = w_0;
v(1) = v_0;
sr(1) = slip;

N = W;
sr_set = -0.1;
epsilon = 1e-3;

% Main simulation loop
for i = 1:length(t)-1
    mu = magic_formula(sr(i), 2);
    J_eq = 1/((1/J) + (sr(i)+1)/(M*R^2));
    
    e = sr(i) - sr_set;
    
    if w(i) > 0
        if e < -1*epsilon
            T_in = 1*e;
        elseif e > epsilon
            T_in = -2*e;
        else
            T_in = b*w(i) + mu*N*R*J/J_eq;
        end
    else
        T_in = 0;
    end
    
    w_dot = (-b/J)*w(i) - mu*N*R/J + (1/J)*T_in;
    sr_dot(i+1) = ((sr(i)+1)/w(i))*((-b/J)*w(i) - mu*N*R/J_eq + (1/J)*T_in);
    v_dot = (1/M)*mu*N;
    
    w(i+1) = w(i) + w_dot*dt;
    %sr(i+1) = sr(i) + sr_dot(i+1)*dt;
    v(i+1) = v(i) + v_dot*dt;
    sr(i+1) = w(i+1)/v(i+1) - 1;
    
end 

plot(t, w);
%plot(sr, sr_dot)
