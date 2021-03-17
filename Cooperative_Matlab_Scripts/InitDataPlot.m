function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);

    plt.p = zeros(6, maxloops);

    plt.p_dot = zeros(6, maxloops);
    
    plt.altitude = zeros(1, maxloops);
    plt.xi = zeros(1, maxloops); 
    
    plt.time1 = 0; 
    plt.time2 = 0; 
    plt.time3 = 0; 
    
    plt.toolPos = zeros(3, maxloops); 
    
    plt.goalTool = zeros(3,maxloops); 
    
    plt.Ajl=zeros();
    
    plt.xdot_jl = zeros(7, maxloops);
    plt.xdot_mu = zeros(1, maxloops);
    plt.xdot_t = zeros(6, maxloops);

    plt.a = zeros(11, maxloops);

end

