close all, clc
a1=0.1125; b1=0.1125; r1=0.0254;
a2=-0.1125; b2=0.1125; r2=0.0254;
a3=-0.1125; b3=-0.1125; r3=0.0254;
a4=0.1125; b4=-0.1125; r4=0.0254;
d1=0; d2=0; d3=0; d4=0;
load('dataset_combined.mat')
%load('params_con.mat')
%load('params_ga.mat')
%load('params_ps.mat')
%%
initial_params=[a1,a2,a3,a4,b1,b2,b3,b4,r1,r2,r3,r4]; 
% CHOOSE DIFFERENT PARAMS,e.g. params_con (after optimization)
[J_ga,ex,ey,epsi]=plot_data(dataset_combined,initial_params)
av_ex=zeros(1,5); % calculating average max error
av_ey=av_ex;
av_th=av_ex;
%%
function [J,exx,eyy,eppsi]=plot_data(dataset_combined, params)
    point=0;
    trajectory_repetition_total_num=size(dataset_combined,2);
    jp = 30;
    J=0;
    trajectory_rep = 0;
    aa1=params(1);
    aa2=params(2);
    aa3=params(3);
    aa4=params(4);
    bb1=params(5);
    bb2=params(6);
    bb3=params(7);
    bb4=params(8);
    rr1=params(9);
    rr2=params(10);
    rr3=params(11);
    rr4=params(12);
    exx=0;eyy=0;eppsi=0;
    %for every rep
    for rep = 1:1:trajectory_repetition_total_num
        % vr=1, dt=2, x=3, y=4, psi=5, g1=6, g2=7, g3=8, g4=9,
        % w1=10, w2=11, w3=12, w4=13
        time_abs=dataset_combined{rep}(:,1);
        dt=dataset_combined{rep}(:,2);
            
        x_abs=dataset_combined{rep}(:,3);
        y_abs=dataset_combined{rep}(:,4);
        psi_abs=dataset_combined{rep}(:,5);
            
        g1=dataset_combined{rep}(:,6);
        g2=dataset_combined{rep}(:,7);
        g3=dataset_combined{rep}(:,8);
        g4=dataset_combined{rep}(:,9);
            
        w1=dataset_combined{rep}(:,10);
        w2=dataset_combined{rep}(:,11);
        w3=dataset_combined{rep}(:,12);
        w4=dataset_combined{rep}(:,13);

        x_odom=zeros(size(time_abs));
        y_odom=zeros(size(time_abs));
        psi_odom=zeros(size(time_abs));
        psi_odom(1)=psi_abs(1);
        
        % can come in useful
        %if rep == 10 
            %y_abs=y_abs-y_abs(1);
        %end
            
        for t=1:(length(time_abs)-1)
            v1=rr1*w1(t);
            v2=rr2*w2(t);
            v3=rr3*w3(t);
            v4=rr4*w4(t);
            c1=cos(g1(t)+psi_odom(t))/4;
            c2=cos(g2(t)+psi_odom(t))/4;
            c3=cos(g3(t)+psi_odom(t))/4;
            c4=cos(g4(t)+psi_odom(t))/4;
            s1=sin(g1(t)+psi_odom(t))/4;
            s2=sin(g2(t)+psi_odom(t))/4;
            s3=sin(g3(t)+psi_odom(t))/4;
            s4=sin(g4(t)+psi_odom(t))/4;
            Vwh=[v1;v2;v3;v4];

            WJ1=(-bb1*cos(g1(t))+aa1*sin(g1(t)))/(4*(aa1^2+bb1^2));
            WJ2=(-bb2*cos(g2(t))+aa2*sin(g2(t)))/(4*(aa2^2+bb2^2));
            WJ3=(-bb3*cos(g3(t))+aa3*sin(g3(t)))/(4*(aa3^2+bb3^2));
            WJ4=(-bb4*cos(g4(t))+aa4*sin(g4(t)))/(4*(aa4^2+bb4^2));
            Jacob=[c1, c2, c3, c4;s1, s2, s3, s4; WJ1, WJ2, WJ3, WJ4];
            Vodo=Jacob*Vwh;


            x_odom(t+1)=x_odom(t)+dt(t+1)*Vodo(1);
            y_odom(t+1)=y_odom(t)+dt(t+1)*Vodo(2);
            psi_odom(t+1)=psi_odom(t)+dt(t+1)*Vodo(3);

            exx=exx+abs(x_odom(t+1)-x_abs(t+1));
            eyy=eyy+abs(y_odom(t+1)-y_abs(t+1));
            eppsi=eppsi+abs(psi_odom(t+1)-psi_abs(t+1));
            J=J+(x_odom(t+1)-x_abs(t+1))^2+(y_odom(t+1)-y_abs(t+1))^2+(psi_odom(t+1)-psi_abs(t+1))^2;    
        end
       
    if mod(rep,5)==1
       point = point+1;
       figure(2*point-1)
       fig = gcf; fig.Color=[1 1 1 0]
       ax=gca;ax.FontSize = 9;
       figure(2*point)
       fig = gcf; fig.Color=[1 1 1 0]
    end
    lw=1.5;  
    figure(2*point-1)
    % y(x)
    subplot(2,2,1)
    plot(x_abs(jp:end), y_abs(jp:end),'linewidth', lw), grid on, hold on 
    axis equal    
    xlabel("X position [m]"),ylabel("Y position [m]")
    title('Y - X'),legend('1','2','3','4','5')
    ax=gca; ax.YAxis.Exponent = 0; ax.XAxis.Exponent = 0; ax.FontSize = 16;
    % x(t)
    subplot(2,2,2)
    plot(time_abs, x_abs, 'linewidth', lw), hold on 
    ylabel("X position [m]"), grid on, xlabel("Time [s]"),title('X position')
    xlim([0,time_abs(end)])
    ax=gca; ax.YAxis.Exponent = 0; ax.XAxis.Exponent = 0; ax.FontSize = 16;
    %y(t)
    subplot(2,2,3)
    plot(time_abs, y_abs, 'linewidth', lw)
    ylabel("Y position [m]"), grid on, xlabel("Time [s]")
    xlim([0,time_abs(end)]),title('Y position'), hold on 
    ax=gca; ax.YAxis.Exponent = 0; ax.XAxis.Exponent = 0; ax.FontSize = 16;
    % theta(t)
    subplot(2,2,4)
    plot(time_abs, psi_abs, 'linewidth', lw), grid on, hold on
    ylabel("Orientation \theta [rad]"), xlabel("Time [s]")
    title('Orientation \theta'), xlim([0,time_abs(end)])
    ax=gca; ax.YAxis.Exponent = 0; ax.XAxis.Exponent = 0; ax.FontSize = 16;
    %saveas(gcf,figTitle+".png")
    %% all in one error
    figure(2*point)
    % errors
    ex=abs(x_odom-x_abs); ey=abs(y_odom-y_abs); epsi=abs(psi_odom-psi_abs);
    
    disp("Trajectory"),disp(point)
    trajectory_rep = trajectory_rep+1;
    % average max errors
    av_ex(trajectory_rep)=max(ex); 
    av_ey(trajectory_rep)=max(ey);
    av_eth(trajectory_rep)=max(epsi);
    if mod(rep,5)==0
        fprintf("%f, %f, %f, %f, %f, %f", max(av_ex),mean(av_ex) , max(av_ey),mean(av_ey), max(av_eth),mean(av_eth) )
        trajectory_rep = 0;
    end
    
    % y - x abs
    subplot(221) 
    plot(x_abs,y_abs,'linewidth',lw),grid on
    ylabel('Y position [m]'),xlabel('X position [m]')
    title('Y - X'), hold on, axis equal
    ax=gca; ax.YAxis.Exponent = 0; ax.XAxis.Exponent = 0; ax.FontSize = 16;

    subplot(222)
    % x abs errors
    plot(time_abs,ex,'linewidth',lw)
    title('Absolute Error - x'),ylabel('|e_x| [m]'),xlabel('Time [s]')  
    xlim([0,time_abs(end)]), grid on , hold on
    ax=gca; ax.YAxis.Exponent = 0; ax.XAxis.Exponent = 0; ax.FontSize = 16;

    subplot(223)
    % y abs errors
    plot(time_abs,ey,'linewidth',lw)
    title('Absolute Error - y'),ylabel('|e_y| [m]'),xlabel('Time [s]')
    xlim([0,time_abs(end)]), grid on, hold on
    ax=gca; ax.YAxis.Exponent = 0; ax.XAxis.Exponent = 0; ax.FontSize = 16;

    subplot(224)
    % theta abs errors
    plot(time_abs,epsi,'linewidth',lw)
    title('Absolute Error - \theta')
    ylabel('|e_\theta| [rad]'),legend('1','2','3','4','5')
    xlabel('Time [s]'), xlim([0,time_abs(end)]), grid on, hold on
    ax=gca; ax.YAxis.Exponent = 0; ax.XAxis.Exponent = 0; ax.FontSize = 16;   
    end
end
