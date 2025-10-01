%% PID Controller Design for DC Motor Speed Control
clc; clear; close all;

% === Step 1: Motor Parameters ===
J = 0.01;   % Inertia
B = 0.1;    % Friction
K = 0.01;   % Motor constant
R = 1;      % Resistance
L = 0.5;    % Inductance

% Transfer Function: W(s)/V(s)
num = [K];
den = [J*L, (J*R + L*B), (B*R + K^2)];
motor_tf = tf(num, den);

disp('DC Motor Transfer Function:');
motor_tf

% === Step 2: Open-Loop Response ===
figure;
step(motor_tf, 5);
title('Open Loop Step Response of DC Motor');
grid on;
info_open = stepinfo(motor_tf)

% === Step 3: PID Controller (Manual Values) ===
Kp = 100; Ki = 200; Kd = 10;
C = pid(Kp, Ki, Kd);

sys_cl = feedback(C*motor_tf, 1);

figure;
step(sys_cl, 5);
title('Closed Loop Step Response with Manual PID');
grid on;
info_manual = stepinfo(sys_cl)

% === Step 4: PID Controller (Auto Tuning) ===
[C_auto, info_auto] = pidtune(motor_tf, 'PID');
sys_tuned = feedback(C_auto*motor_tf, 1);

figure;
step(sys_tuned, 5);
title('Closed Loop Step Response with Tuned PID');
grid on;
info_tuned = stepinfo(sys_tuned)

% === Step 5: Compare Responses ===
figure;
step(motor_tf, sys_cl, sys_tuned, 5);
legend('Open Loop', 'Manual PID', 'Tuned PID');
title('Comparison of Motor Speed Control');
grid on;
