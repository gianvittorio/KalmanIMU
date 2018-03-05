clear all
close all
clc
format short
data = load('results.txt');
rollE = data(:,1);
pitchE = data(:,2);
yawE = data(:,3);
roll = data(:,4);
pitch = data(:,5);
yaw = data(:,6);

plot(rollE, 'color', [0 0 1]);
hold on
plot(roll, 'color', [1 0 0]);

figure(2);

plot(pitchE, 'color', [0 0 1]);
hold on
plot(pitch, 'color', [1 0 0]);

figure(3);
plot(yawE, 'color', [0 0 1]);
hold on
plot(yaw, 'color', [1 0 0]);