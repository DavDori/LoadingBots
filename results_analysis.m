clear all
close all
clc
general_path = 'C:\Users\Davide\Desktop\Projects\LoadingBots\LoadingBots\results\multiple_simulations';
speed_path_slow = '\speed0.2';
speed_path_norm = '\speed0.4';
speed_path_fast = '\speed0.6';
obs_medium_path = '\obs0.2m';
obs_big_path = '\obs0.3m';

range_agents = 5:10;
index_range = 1:length(range_agents);
c = ['#0072BD';'#D95319';'#EDB120';'#7E2F8E';'#77AC30';'#4DBEEE';'#A2142F'];
c2 = [[0, 0.4470, 0.7410];[0.8500, 0.3250, 0.0980];[0.9290, 0.6940, 0.1250];...	
      [0.4940, 0.1840, 0.5560];[0.4660, 0.6740, 0.1880];[0.3010, 0.7450, 0.9330];...
      [0.6350, 0.0780, 0.1840]];
	

%% LOAD DATA
% reads results from slow moving object

for i = index_range
    R_slow(i).val = readmatrix(strcat(general_path, speed_path_slow,...
        '\', string(range_agents(i)), '_agents\results.xlsx'));
end

% reads results from medium speed moving object
for i = index_range
    R_norm(i).val = readmatrix(strcat(general_path, speed_path_norm,...
        '\agents',string(range_agents(i)),'.xlsx'));
end

% reads results from high speed moving object
for i = index_range
    R_fast(i).val = readmatrix(strcat(general_path, speed_path_norm,...
        '\agents',string(range_agents(i)),'.xlsx'));
end

% reads results from medium size moving object
for i = index_range
    R_medium(i).val = readmatrix(strcat(general_path, obs_medium_path,...
        '\agents', string(range_agents(i)), 'obs0.2.xlsx'));
end

for i = index_range
    R_big(i).val = readmatrix(strcat(general_path, obs_big_path,...
        '\agents', string(range_agents(i)), 'obs0.3.xlsx'));
end

%% POLAR REPRESENTATION

% works for every results since have the same range of angles
edges = [R_big(1).val(1,1:20)] - (R_big(1).val(1,2) - R_big(1).val(1,1))/2;
n_sims = length(R_big(i).val(1,:));

% SLOW MOVING OBSTACLE
figure()
sgtitle('Slow and small obstacle')
for i = index_range
    subplot(2,3,i)
    A = R_slow(i).val(1,:) .* R_slow(i).val(2,:);
    % discard 0s
    A = A(A ~= 0);
    polarhistogram(A, 'BinEdges', edges, 'FaceColor', c(i,:), 'EdgeColor', c(i,:))
    title(strcat(string(range_agents(i)), ' agents')) 
    set(gcf,'color','w');
end

% MEDIUM SPEED MOVING OBSTACLE
figure()
sgtitle('Medium speed and small obstacle')
for i = index_range
    subplot(2,3,i)
    A = R_norm(i).val(1,:) .* R_norm(i).val(2,:);
    % discard 0s
    A = A(A ~= 0);
    polarhistogram(A, 'BinEdges', edges, 'FaceColor', c(i,:), 'EdgeColor', c(i,:))
    title(strcat(string(range_agents(i)), ' agents')) 
    set(gcf,'color','w');
end

% FAST MOVING OBSTACLE
figure()
sgtitle('High speed and small obstacle')
for i = index_range
    subplot(2,3,i)
    A = R_fast(i).val(1,:) .* R_fast(i).val(2,:);
    % discard 0s
    A = A(A ~= 0);
    polarhistogram(A, 'BinEdges', edges, 'FaceColor', c(i,:), 'EdgeColor', c(i,:))
    title(strcat(string(range_agents(i)), ' agents')) 
    set(gcf,'color','w');
end


% MEDIUM SIZE OBSTACLE
figure()
sgtitle('Slow and medium size obstacle')
for i = index_range
    subplot(2,3,i)
    A = R_medium(i).val(1,:) .* R_medium(i).val(2,:);
    % discard 0s
    A = A(A ~= 0);
    polarhistogram(A, 'BinEdges', edges, 'FaceColor', c(i,:), 'EdgeColor', c(i,:))
    title(strcat(string(range_agents(i)), ' agents')) 
    set(gcf,'color','w');
end

% BIG OBSTACLE
figure()
sgtitle('Slow and big obstacle')
for i = index_range
    subplot(2,3,i)
    
    A = R_big(i).val(1,:) .* R_big(i).val(2,:);
    % discard 0s
    A = A(A ~= 0);
    polarhistogram(A, 'BinEdges', edges, 'FaceColor', c(i,:), 'EdgeColor', c(i,:))
    title(strcat(string(range_agents(i)), ' agents')) 
    set(gcf,'color','w');
end

%% TOTAL EFFECTIVENESS

R_tot = zeros(length(index_range), 5 * n_sims); 
R_tot_angles = zeros(length(index_range), 5 * n_sims); 
R_size = zeros(length(index_range), 1);
R_speed = zeros(length(index_range), 1);
for i = index_range
    angles_x1 = R_big(i).val(1,:);
    angles = [angles_x1, angles_x1, angles_x1, angles_x1, angles_x1];
    R_tot(i,:) = [R_big(i).val(2,:), R_fast(i).val(2,:), R_norm(i).val(2,:),...
                  R_medium(i).val(2,:), R_slow(i).val(2,:)];
    R_size(i) = sum([R_big(i).val(2,:), R_medium(i).val(2,:), R_slow(i).val(2,:)]);
    R_speed(i) = sum([R_fast(i).val(2,:), R_norm(i).val(2,:), R_slow(i).val(2,:)]);
    R_tot_angles(i,:) = R_tot(i,:) .* angles;
end

figure()
for i = index_range
    subplot(2,3,i)
    
    A = R_tot_angles(i,:);
    % discard 0s
    A = A(A ~= 0);
    polarhistogram(A, 'BinEdges', edges, 'FaceColor', c(i,:), 'EdgeColor', c(i,:))
    title(strcat(string(range_agents(i)), ' agents')) 
    set(gcf,'color','w');
end

disp('Total number of successes per n of agent:')
for i = index_range
    disp(strcat(string(range_agents(i)), ' agents: ', string(sum(R_tot(i,:)))))
end
disp('Total number of successes per n of agent, obs size:')
for i = index_range
    disp(strcat(string(range_agents(i)), ' agents: ', string(R_size(i)),...
        ' % ', string(100 * R_size(i) / (3 * 40))))
end
disp('Total number of successes per n of agent, obs speed:')
for i = index_range
    disp(strcat(string(range_agents(i)), ' agents: ', string(R_speed(i)),...
        ' % ', string(100 * R_speed(i) / (3 * 40))))
end 
%% BARS
% for obstacle dimension
Z = [];
for i = index_range
    Z = [Z ; sum(R_slow(i).val(2,:)), sum(R_medium(i).val(2,:)), sum(R_big(i).val(2,:))];
end

figure

set(gcf,'color','w');
bar(Z, 'stacked')
title('Obstacle of different sizes')
ylabel('Successful simulations')
yticks('auto')
xlabel('Number of active agents')
xticks([1,2,3,4,5,6]);
xticklabels({5,6,7,8,9,10})
legend('0.1m', '0.2m', '0.3m')

% obstacle speed
Z = [];
for i = index_range
    Z = [Z ; sum(R_slow(i).val(2,:)), sum(R_norm(i).val(2,:)), sum(R_fast(i).val(2,:))];
end

figure

set(gcf,'color','w');
bar(Z, 'stacked')
title('Obstacle with different speed')
ylabel('Successful simulations')
yticks('auto')
xlabel('Number of active agents')
xticks([1,2,3,4,5,6]);
xticklabels({5,6,7,8,9,10})
legend('0.2m/s', '0.4m/s', '0.6m/s')


%% 

% SLOW MOVING OBSTACLE
figure()
set(gcf,'color','w');
A = zeros(6, 1);
for i = index_range
    A(i) = sum(R_slow(i).val(2,:));
end
b = bar(A);
b.FaceColor = 'flat';
for i = index_range
    b.CData(i,:) = c2(i,:);
end
title('Slow and small obstacle')
xlabel('Number of active agents')
ylabel('Successful simulations')
xticks([1,2,3,4,5,6]);
xticklabels({5,6,7,8,9,10})


% MEDIUM SPEED MOVING OBSTACLE
figure()
set(gcf,'color','w');
A = zeros(6, 1);
for i = index_range
    A(i) = sum(R_norm(i).val(2,:));
end
b = bar(A);
b.FaceColor = 'flat';
for i = index_range
    b.CData(i,:) = c2(i,:);
end
title('Medium speed and small obstacle')
xlabel('Number of active agents')
ylabel('Successful simulations')
xticks([1,2,3,4,5,6]);
xticklabels({5,6,7,8,9,10})

% FAST MOVING OBSTACLE
figure()
set(gcf,'color','w');
A = zeros(6, 1);
for i = index_range
    A(i) = sum(R_fast(i).val(2,:));
end
b = bar(A);
b.FaceColor = 'flat';
for i = index_range
    b.CData(i,:) = c2(i,:);
end
title('Fast and small obstacle')
xlabel('Number of active agents')
ylabel('Successful simulations')
xticks([1,2,3,4,5,6]);
xticklabels({5,6,7,8,9,10})


% MEDIUM SIZE OBSTACLE
figure()
set(gcf,'color','w');
A = zeros(6, 1);
for i = index_range
    A(i) = sum(R_medium(i).val(2,:));
end
b = bar(A);
b.FaceColor = 'flat';
for i = index_range
    b.CData(i,:) = c2(i,:);
end
title('Slow speed and medium size obstacle')
xlabel('Number of active agents')
ylabel('Successful simulations')
xticks([1,2,3,4,5,6]);
xticklabels({5,6,7,8,9,10})

% BIG OBSTACLE
figure()
set(gcf,'color','w');
A = zeros(6, 1);
for i = index_range
    A(i) = sum(R_big(i).val(2,:));
end
b = bar(A);
b.FaceColor = 'flat';
for i = index_range
    b.CData(i,:) = c2(i,:);
end

title('Slow speed and big obstacle')
xlabel('Number of active agents')
ylabel('Successful simulations')
xticks([1,2,3,4,5,6]);
xticklabels({5,6,7,8,9,10})
