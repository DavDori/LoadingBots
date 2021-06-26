clear all
close all
clc

%% SET UP
% map
map = png2BOMap('map_test_1.png',22);

Ts = 1e-2;

%% TESTING SECTION
% comment if not needed
fprintf('TESTING SECTION\n');
% for simulation need a high resolution

Testing_unit = tester(map, Ts, 1e-2);
% Testing_unit.dodgeMovingObstacle(true);
% test = Testing_unit.priorityPD(true);
Testing_unit.runAll(true);