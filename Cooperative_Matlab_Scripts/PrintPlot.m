function [ ] = PrintPlot( plt )

plt.time2 = plt.time1 + plt.time2; 
plt.time3 = plt.time2 + plt.time3; 

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

f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(4);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');
saveas(f, ['Results/' '3.jpg']);



f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(5);
hplot = plot(plt.t, plt.a(8:9,:));
set(hplot, 'LineWidth', 2);
legend('Amu', 'Aha');
saveas(f, ['Results/' '4.jpg']);

%% Altitude
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(6)
hplot = plot(plt.t, plt.altitude,'b');
set(hplot, 'LineWidth', 2);
hold on; 
hplot1 = plot([plt.time1, plt.time1], [0, 35], 'g'); 
hold on; 
hplot2 = plot([plt.time2, plt.time2], [0, 35], 'y');
hold on; 
hplot3 = plot([plt.time3, plt.time3], [0, 35], 'r');
legend('altitude', 'EndingPhase1', 'EndingPhase2', 'EndingPhase3');
saveas(f, ['Results/' '5.jpg']);

%% vehicle positions
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(2);
subplot(3,3,1);
ylim([8, 11])
hplot = plot(plt.t, plt.p(1,:),'b');
set(hplot, 'LineWidth', 2);
hold on; 
hplot1 = plot([plt.time1, plt.time1], [8, 11], 'g'); 
hold on; 
hplot2 = plot([plt.time2, plt.time2], [8, 11], 'y');
hold on; 
hplot3 = plot([plt.time3, plt.time3], [8, 11], 'r');
legend('x', 'EndingPhase1', 'EndingPhase2', 'EndingPhase3');


subplot(3,3,2);
ylim([36, 39]);
hplot = plot(plt.t, plt.p(2,:),'m');
set(hplot, 'LineWidth', 2);
hold on; 
hplot1 = plot([plt.time1, plt.time1], [36, 39], 'g'); 
hold on; 
hplot2 = plot([plt.time2, plt.time2], [36, 39], 'y');
hold on; 
hplot3 = plot([plt.time3, plt.time3], [36, 39], 'r');
legend('y', 'EndingPhase1', 'EndingPhase2', 'EndingPhase3');

subplot(3,3,3);
ylim([-39,-35.5])
hplot = plot(plt.t, plt.p(3,:),'c');
set(hplot, 'LineWidth', 2);
hold on; 
hplot1 = plot([plt.time1, plt.time1], [-35.5, -39], 'g'); 
hold on; 
hplot2 = plot([plt.time2, plt.time2], [-35.5, -39], 'y');
hold on; 
hplot3 = plot([plt.time3, plt.time3], [-35.5, -39], 'r');
legend('z', 'EndingPhase1', 'EndingPhase2', 'EndingPhase3');



subplot(3,3,4);
ylim([0, 0.04]);
hplot = plot(plt.t, plt.p(4,:),'k');
set(hplot, 'LineWidth', 2);
hold on; 
hplot1 = plot([plt.time1, plt.time1], [0, 0.04], 'g'); 
hold on;  
hplot2 = plot([plt.time2, plt.time2], [0, 0.04], 'y');
hold on; 
hplot3 = plot([plt.time3, plt.time3], [0, 0.04], 'r');
legend('roll', 'EndingPhase1', 'EndingPhase2', 'EndingPhase3');


subplot(3,3,5);
ylim([-0.06,-0.035]);
hplot = plot(plt.t, plt.p(5,:),'r');
set(hplot, 'LineWidth', 2);
hold on; 
hplot1 = plot([plt.time1, plt.time1], [-0.035, -0.06], 'g'); 
hold on; 
hplot2 = plot([plt.time2, plt.time2], [-0.035, -0.06], 'y');
hold on; 
hplot3 = plot([plt.time3, plt.time3], [-0.035, -0.06], 'r');
legend('pitch', 'EndingPhase1', 'EndingPhase2', 'EndingPhase3');

subplot(3,3,6);
ylim([-0.05, 0.6]);
hplot = plot(plt.t, plt.p(6,:),'g');
set(hplot, 'LineWidth', 2);
hold on; 
hplot1 = plot([plt.time1, plt.time1], [-0.2, 0.6], 'g'); 
hold on; 
hplot2 = plot([plt.time2, plt.time2], [-0.2, 0.6], 'y');
hold on; 
hplot3 = plot([plt.time3, plt.time3], [-0.2, 0.6], 'r');
legend('yaw', 'EndingPhase1', 'EndingPhase2', 'EndingPhase3');
saveas(f, ['Results/' '6.jpg']);

%% xi 
f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(7)
hplot = plot(plt.t, plt.xi, 'r');
set(hplot, 'LineWidth', 2);
hold on; 
hplot1 = plot([plt.time1, plt.time1], [0, 1], 'g'); 
hold on; 
hplot2 = plot([plt.time2, plt.time2], [0, 1], 'y');
hold on; 
hplot3 = plot([plt.time3, plt.time3], [0, 1], 'r');
legend('xi', 'EndingPhase1', 'EndingPhase2', 'EndingPhase3');
saveas(f, ['Results/' '7.jpg']);



end 