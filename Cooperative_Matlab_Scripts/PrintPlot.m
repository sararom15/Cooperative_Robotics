function [ ] = PrintPlot( plt )



% some predefined plots
% you can add your own
% f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
% %figure(1);
% subplot(2,1,1);
% hplot = plot(plt.t, plt.q);
% set(hplot, 'LineWidth', 1);
% legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
% subplot(2,1,2);
% hplot = plot(plt.t, plt.q_dot);
% set(hplot, 'LineWidth', 1);
% legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
% saveas(f, ['Results/' '1.jpg']);
% 
% 
% 
% f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
% %figure(3);
% subplot(3,1,1);
% hplot = plot(plt.t, plt.p);
% set(hplot, 'LineWidth', 1);
% legend('x','y','z','roll','pitch','yaw');
% subplot(2,1,2);
% hplot = plot(plt.t, plt.p_dot);
% set(hplot, 'LineWidth', 1);
% legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');
% saveas(f, ['Results/' '2.jpg']);
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

f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(6)
hplot = plot(plt.t, plt.altitude);
set(hplot, 'LineWidth', 2);
legend('altitude');
saveas(f, ['Results/' '5.jpg']);

f = figure('units','normalized','outerposition',[0 0 1 1], 'visible', 'off');
%figure(2);
subplot(3,3,1);
hplot = plot(plt.t, plt.p(1,:),'g');
set(hplot, 'LineWidth', 2);
legend('x');
subplot(3,3,2);
hplot = plot(plt.t, plt.p(2,:),'b');
set(hplot, 'LineWidth', 2);
legend('y');
subplot(3,3,3);
hplot = plot(plt.t, plt.p(3,:),'y');
set(hplot, 'LineWidth', 2);
legend('z');
subplot(3,3,4);
hplot = plot(plt.t, plt.p(4,:),'r');
set(hplot, 'LineWidth', 2);
legend('roll');
subplot(3,3,5);
hplot = plot(plt.t, plt.p(5,:),'b');
set(hplot, 'LineWidth', 2);
legend('pitch');
subplot(3,3,6);
hplot = plot(plt.t, plt.p(6,:));
set(hplot, 'LineWidth', 2);
legend('yaw');
saveas(f, ['Results/' '6.jpg']);

end 