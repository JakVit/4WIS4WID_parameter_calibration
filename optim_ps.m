% robot parameters
a1=0.1125; b1=0.1125; r1=0.0254;
a2=-0.1125; b2=0.1125; r2=0.0254;
a3=-0.1125; b3=-0.1125; r3=0.0254;
a4=0.1125; b4=-0.1125; r4=0.0254;
load('dataset_combined.mat')
%% cost function
J = @(params) Cost_fun(dataset_combined,params);
initial_params=[a1,a2,a3,a4,b1,b2,b3,b4,r1,r2,r3,r4];
lb=0.95; ub=1.05;
LB=initial_params.*[0.95 1.05 1.05 0.95 0.95 0.95 1.05 1.05 0.95 0.95 0.95 0.95];
UB=initial_params.*[1.05 0.95 0.95 1.05 1.05 1.05 0.95 0.95 1.05 1.05 1.05 1.05];
%% particle swarm optimization
tic
nvars=12;
options_ps = optimoptions('particleswarm', 'Display', 'iter');
[params_ps, fval_ps] = particleswarm(J, nvars, LB, UB, options_ps)
t_ps=toc;
save('params_ps.mat','params_ps','t_ps')

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%function [J,Kalman_podaci]=Cost_fun(podaci, params)

function J=Cost_fun(DS, params)
    repetition_total_num=size(DS,2);
    J=0;
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
    % trajectory repetition
    % remove par if you don't want parallel pool optimization
    parfor rep = 1:repetition_total_num
        % time=1, dt=2, x=3, y=4, psi=5, g1=6, g2=7, g3=8, g4=9,
        % w1=10, w2=11, w3=12, w4=13
        time_abs=DS{rep}(:,1);
        dt=DS{rep}(:,2);
            
        x_abs=DS{rep}(:,3);
        y_abs=DS{rep}(:,4);
        psi_abs=DS{rep}(:,5);
            
        g1=DS{rep}(:,6);
        g2=DS{rep}(:,7);
        g3=DS{rep}(:,8);
        g4=DS{rep}(:,9);
            
        w1=DS{rep}(:,10);
        w2=DS{rep}(:,11);
        w3=DS{rep}(:,12);
        w4=DS{rep}(:,13);

        x_odom=zeros(size(time_abs));
        y_odom=zeros(size(time_abs));
        psi_odom=zeros(size(time_abs));
        psi_odom(1)=psi_abs(1);
            
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
            %% integration - backward Euler
            x_odom(t+1)=x_odom(t)+dt(t+1)*Vodo(1);
            y_odom(t+1)=y_odom(t)+dt(t+1)*Vodo(2);
            psi_odom(t+1)=psi_odom(t)+dt(t+1)*Vodo(3);        
            J=J+(x_odom(t+1)-x_abs(t+1))^2+(y_odom(t+1)-y_abs(t+1))^2+(psi_odom(t+1)-psi_abs(t+1))^2;

        end
    end
end
