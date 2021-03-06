clear all
close all
clc
format short
%Parse data file and define constants:
data = load('results.txt');
rollE = data(:,1);
pitchE = data(:,2);
yawE = data(:,3);
roll = data(:,4);
pitch = data(:,5);
yaw = data(:,6);
gyroR = data(:,7);
gyroP = data(:,8);
gyroY = data(:,9);
elapsedTime = 3.5;
[M,N] = size(data);
Ts = elapsedTime/M;
t = 0:Ts:Ts*(M-1);

gyroRoll(1,:) = roll(1);
gyroPitch(1,:) = pitch(1);
gyroYaw(1,:) = yaw(1);
%Calculate angles from gyroscope measurements:
for k = 2:M
    gyroRoll(k,:) = gyroRoll(k-1,:) + Ts*gyroR(k);
    gyroPitch(k,:) = gyroPitch(k-1,:) + Ts*gyroP(k);
    gyroYaw(k,:) = gyroYaw(k-1,:) + Ts*gyroY(k);
end

%Plot results:
subplot(3,1,1);
title('Kalman Filter IMU');
plot(t, rollE, 'color', [0 0 1]);
hold on
plot(t, roll, 'color', [1 0 0]);
legend('Kalman Filter Estimation', 'Measurement (indirect)', 'location', 'northwest',...
        'orientation', 'horizontal');
xlabel('Time [s]');
ylabel('Roll [degrees]');
subplot(3,1,2);
plot(t, pitchE, 'color', [0 0 1]);
hold on
plot(t, pitch, 'color', [1 0 0]);
xlabel('Time [s]');
ylabel('Pitch [degrees]');
subplot(3,1,3);
plot(t, yawE, 'color', [0 0 1]);
hold on
plot(t, yaw, 'color', [1 0 0]);
xlabel('Time [s]');
ylabel('Yaw [degrees]');