clear;
clc;


%% Serial port utility
ports = serialportlist("available");
disp(ports);


%% Parameters
N = 6;
ts_us = 10000;
dt = ts_us / 1e6;
port = ports(1);
baudrate = 115200;
T = 0:dt:1;
pwms = [255 * sin(2*pi.*T./10); 255 * cos(2*pi.*T./10); 2 * T; 255 * sin(2*pi.*T./10); 255 * cos(2*pi.*T./10); 2 * T];

num = zeros(1, length(T));
status = zeros(1, length(T));
switches = zeros(N, length(T));
delta = zeros(N, length(T));
encoders = zeros(N, length(T));


%% Initialization
clear robot
robot = Robot(N, ts_us, port, baudrate);


%% Test 1
time = tic();
for k = 1:length(T)
    delta = toc(time) * 1e6;
    time = tic();
    % res = robot.ctrl_pwm(pwms(:, k));
    res = robot.ctrl_idle();
    switches(1:N, k) = robot.getEndstops();
    encoders(1:N, k) = robot.getEncoders();
    fprintf("cycle:    %d\n", k);
    fprintf("delta:    %0.1f us\n", delta);
    fprintf("res:      %s\n", string(res));
    fprintf("switches: %d %d %d %d %d %d\n", switches(1,k), switches(2,k), switches(3,k), switches(4,k), switches(5,k), switches(6,k));
    fprintf("encoders: %d %d %d %d %d %d\n", encoders(1,k), encoders(2,k), encoders(3,k), encoders(4,k), encoders(5,k), encoders(6,k));
    fprintf("\n");
end

robot.ctrl_idle();



%%
tiledlayout(3,1);

ax1 = nexttile;
plot(T', pwms');
grid on;
title("PWMs");
legend("pwm " + string(1:N));

ax2 = nexttile;
plot(T', switches');
grid on;
title("Switches");
legend("switch " + string(1:N));

ax3 = nexttile;
plot(T', encoders');
grid on;
title("Encoders");
legend("encoders " + string(1:N));

linkaxes([ax1, ax2, ax3], 'x');

