load('dataset_combined.mat')
repetition_total_num=size(dataset_combined,2);
T=1e-3;
a1=0.1125; b1=0.1125; r1=0.0254;
a2=-0.1125; b2=0.1125; r2=0.0254;
a3=-0.1125; b3=-0.1125; r3=0.0254;
a4=0.1125; b4=-0.1125; r4=0.0254;
lb=0.95; ub=1.05;
amax=ub*a1;
amin=lb*a1;
bmin=lb*b1;
bmax=ub*b1;
rmax=ub*r1;
rmin=lb*r1;
%%
all_in_one_dataset=zeros(1,14);

for a=1:repetition_total_num
        % vr=1, dt=2, x=3, y=4, psi=5, g1=6, g2=7, g3=8, g4=9,
        % w1=10, w2=11, w3=12, w4=13
        total_time=dataset_combined{a}(10:end,1);
        dt=dataset_combined{a}(10:end,2);
            
        x_abs=dataset_combined{a}(10:end,3);
        y_abs=dataset_combined{a}(10:end,4);
        psi_abs=dataset_combined{a}(10:end,5);
            
        g1=dataset_combined{a}(10:end,6);
        g2=dataset_combined{a}(10:end,7);
        g3=dataset_combined{a}(10:end,8);
        g4=dataset_combined{a}(10:end,9);
            
        w1=dataset_combined{a}(10:end,10);
        w2=dataset_combined{a}(10:end,11);
        w3=dataset_combined{a}(10:end,12);
        w4=dataset_combined{a}(10:end,13);
       
        repetition=a*ones(size(w4));
        total_time=total_time+all_in_one_dataset(end,1);

        all_in_one_dataset=[all_in_one_dataset; [total_time,dt,x_abs,y_abs,psi_abs,g1,g2,g3,g4,w1,w2,w3,w4,repetition]];

end

%% structure format data for Simulink model 
clear dt x_abs y_abs psi_abs g1 g2 g3 g4 w1 w2 w3 w4 repetition
total_time=all_in_one_dataset(:,1);

dt.time=total_time;
dt.signals.values=all_in_one_dataset(:,2);

x_abs.time=total_time;
x_abs.signals.values=all_in_one_dataset(:,3);

y_abs.time=total_time;
y_abs.signals.values=all_in_one_dataset(:,4);

psi_abs.time=total_time;
psi_abs.signals.values=all_in_one_dataset(:,5);

g1.time=total_time;
g1.signals.values=all_in_one_dataset(:,6);

g2.time=total_time;
g2.signals.values=all_in_one_dataset(:,7);

g3.time=total_time;
g3.signals.values=all_in_one_dataset(:,8);

g4.time=total_time;
g4.signals.values=all_in_one_dataset(:,9);

w1.time=total_time;
w1.signals.values=all_in_one_dataset(:,10);

w2.time=total_time;
w2.signals.values=all_in_one_dataset(:,11);

w3.time=total_time;
w3.signals.values=all_in_one_dataset(:,12);

w4.time=total_time;
w4.signals.values=all_in_one_dataset(:,13);

repetition.time=total_time;
repetition.signals.values=all_in_one_dataset(:,14);

% change SDO_OutputTimes in Simulink options if needed
