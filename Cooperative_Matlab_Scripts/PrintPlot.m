function [ ] = PrintPlot( plt )


% some predefined plots
% you can add your own
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
saveas(f, ['Results/' '1.jpg']);

% 
% 
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(3);
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');
saveas(f, ['Results/' '2.jpg']);
% 
% f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
% %figure(4);
% hplot = plot(plt.t, plt.a(1:7,:));
% set(hplot, 'LineWidth', 2);
% legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');
% saveas(f, ['Results/' '3.jpg']);
% 
% 
% 
% f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
% %figure(5);
% hplot = plot(plt.t, plt.a(8:9,:));
% set(hplot, 'LineWidth', 2);
% legend('Amu', 'Aha');
% saveas(f, ['Results/' '4.jpg']);
% 
% %% Altitude
% f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
% %figure(6)
% hplot = plot(plt.t, plt.altitude,'b');
% set(hplot, 'LineWidth', 2);
% hold on; 
% hplot1 = plot([plt.time1, plt.time1], [0, 35], 'g'); 
% legend('altitude', 'EndingPhase1');
% saveas(f, ['Results/' '5.jpg']);

%% vehicle positions
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(2);
subplot(3,3,1);
ylim([5, 11])
hplot = plot(plt.t, plt.p(1,:),'b');
set(hplot, 'LineWidth', 2);
legend('x');


subplot(3,3,2);
ylim([36, 39]);
hplot = plot(plt.t, plt.p(2,:),'m');
set(hplot, 'LineWidth', 2);
legend('y');

subplot(3,3,3);
ylim([-40,-36])
hplot = plot(plt.t, plt.p(3,:),'c');
set(hplot, 'LineWidth', 2);
legend('z');



subplot(3,3,4);
ylim([0, 0.4]);
hplot = plot(plt.t, plt.p(4,:),'k');
set(hplot, 'LineWidth', 2);
legend('roll');


subplot(3,3,5);
ylim([-0.06,-0.05]);
hplot = plot(plt.t, plt.p(5,:),'r');
set(hplot, 'LineWidth', 2);
legend('pitch');

subplot(3,3,6);
ylim([-0.05, 0.6]);
hplot = plot(plt.t, plt.p(6,:),'g');
set(hplot, 'LineWidth', 2);
legend('yaw');
saveas(f, ['Results/' '6.jpg']);

%% xi - alignment z_vehicle/rock
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(7)
hplot = plot(plt.t, plt.xi, 'r');
set(hplot, 'LineWidth', 2);
legend('xi');
saveas(f, ['Results/' '7.jpg']);

%% tool position 
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
subplot(1,3,1);
hplot = plot(plt.t, plt.toolPos(1,:), 'r');
set(hplot, 'LineWidth', 2);
hold on
hplot2 = plot([0,65], [plt.goalTool(1), plt.goalTool(1)], 'g');
legend('x tool position', 'x goal position','Location', 'southeast') 

subplot(1,3,2);
hplot = plot(plt.t, plt.toolPos(2,:), 'r');
set(hplot, 'LineWidth', 2);
hold on
hplot2 = plot([0,65], [plt.goalTool(2), plt.goalTool(2)], 'g');
legend('y tool position', 'y goal position','Location', 'southeast')

subplot(1,3,3);
hplot = plot(plt.t, plt.toolPos(3,:), 'r');
set(hplot, 'LineWidth', 2);
hold on
hplot2 = plot([0,65], [plt.goalTool(3), plt.goalTool(3)], 'g');
legend('y tool position', 'y goal position', 'Location', 'southeast')
saveas(f, ['Results/' 'tool_pos.jpg']); 

%% joint tool configurations 
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
hold on;
for j = 1:7
    subplot(3,3,j)
    set(gca,'ColorOrderIndex',3)
    hplot = plot(plt.t, plt.q(j,:));
    ylim([-3, 3])
    set(hplot, 'LineWidth', 2);
    %hplot = yline(plt.goal(3,end), 'LineWidth', 3, 'LineStyle','--');
    xlabel('time [s]')
    ylabel(strcat('q_',num2str(j), ' [rad]'))
    hplot = yline(plt.jlmin(j),'r');
    hplot = yline(plt.jlmax(j),'r');
end

saveas(f, ['Results/' 'tool_configuration.jpg']); 

f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
hplot = plot(plt.t, plt.error(1,:));
set(hplot, 'LineWidth', 2);
legend('error');
saveas(f, ['Results/' 'error_tool_position.jpg']);


end 