% robot params
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
%% GA algorithm with fixed evolution generations (itermax)
tic;
N=12; B=12; iterMax=600;
population=initial_params.*(lb+(ub-lb)*rand(N,B));
[params_ga, fval_ga] = geneticAlg(dataset_combined,population, LB, UB, iterMax)
t_ga=toc;
save('params_ga.mat','params_ga','t_ga')
%%
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
        % vr=1, dt=2, x=3, y=4, psi=5, g1=6, g2=7, g3=8, g4=9,
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
%%
function [opti_params,Jbest]=geneticAlg(dataSet,population,LB,UB,iterMax)
    [N,B]=size(population);
    Jabs=zeros(N,1);
    Jabs_pot=zeros(N,1);
    pc=0.75; pm=0.06;
    parfor params_entity=1:N
        params=population(params_entity,:);
        Jabs(params_entity) = Cost_fun(dataSet,params);
    end
    Jrel=Jabs/sum(Jabs);
    Jrel=1-Jrel;
    Jrel=Jrel/sum(Jrel);
    
    for iter=1:iterMax
        potential_parents=Roulet(population,Jrel);
        new_gen = Crossover1(potential_parents,pc); %% can choose different Crossover functions
        new_gen = Mutation(new_gen, pm, UB, LB);
        
        parfor params_entity=1:N
            params=new_gen(params_entity,:);
            Jabs_pot(params_entity) = Cost_fun(dataSet,params);
        end
        
        [best_old,index_old]=min(Jabs);
        [best_new, index_new]=min(Jabs_pot);
        rr=randi(N);
        if best_old<Jabs_pot(rr)
            new_gen(rr,:)=population(index_old,:);
        end
        disp(iter)
        if best_old>best_new
            disp(best_new)
        else
            disp(best_old)
        end
        population=new_gen;  
        parfor params_entity=1:N
            params=population(params_entity,:);
            Jabs(params_entity) = Cost_fun(dataSet,params);
        end
        Jrel=Jabs/sum(Jabs);
        Jrel=1-Jrel;
        Jrel=Jrel/sum(Jrel);
    end  
    [Jbest,ind]=min(Jabs);
    opti_params=population(ind,:)    
end 

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function potential_parents = Roulet(population,Jrel)
    [N,B] = size(population);
    potential_parents=zeros(N,B);
    roulet_ar=rand(N,1);
    for i = 1:N
        cumulative_sum=0;
        for j = 1:N
            cumulative_sum=cumulative_sum+Jrel(j);
            if cumulative_sum>=roulet_ar(i)
                potential_parents(i,:)=population(j,:);
                break
            end
        end
    end
end
%% basic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function new_gen = Crossover(potential_parents,pc)
    [N,B] = size(potential_parents);
    couple_num=round(N/2);
    rc=rand(1,couple_num);
    new_gen=zeros(N,B);
    for par =1:couple_num
        if rc(1,par)<pc
            a=randi(B-1,1);
            new_gen((2*par-1),1:a)=potential_parents((2*par-1),1:a); %1,3,5,...
            new_gen((2*par-1),a+1:end)=potential_parents((2*par),a+1:end);

            new_gen((2*par),1:a)=potential_parents((2*par),1:a); %2,4,6,...
            new_gen((2*par),a+1:end)=potential_parents((2*par-1),a+1:end);
        else 
            new_gen((2*par-1),:)=potential_parents((2*par-1),:);
            new_gen((2*par),:)=potential_parents((2*par),:);
        end
    end
end
%% adding weight factors + basic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%33
function new_gen = Crossover1(potential_parents,pc)
    [N,B] = size(potential_parents);
    couple_num=round(N/2);
    rc=rand(1,couple_num);
    new_gen=zeros(N,B);
    for par =1:couple_num
        if rc(1,par)<pc
            c=rand(1);
            d=1-c;
            a=randi(B-1,1);
            suma1=(potential_parents(2*par-1,:)*c+potential_parents(2*par,:)*d);%.*(0.95+0.1*rand(1));
            suma2=(potential_parents(2*par-1,:)*d+potential_parents(2*par,:)*c);%.*(0.95+0.1*rand(1));
            
            new_gen((2*par-1),1:a)=suma1(1:a); %1,3,5,...
            new_gen((2*par-1),a+1:end)=suma2(a+1:end);

            new_gen((2*par),1:a)=suma2(1:a); %2,4,6,...
            new_gen((2*par),a+1:end)=suma1(a+1:end);
            
        else 
            new_gen((2*par-1),:)=potential_parents((2*par-1),:);
            new_gen((2*par),:)=potential_parents((2*par),:);
        end
    end
end
%% adding weight factors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%33
function new_gen = Crossover2(potential_parents,pc)
    [N,B] = size(potential_parents);
    couple_num=round(N/2);
    rc=rand(1,couple_num);
    new_gen=zeros(N,B);
    for par =1:couple_num
        if rc(1,par)<pc
            c=rand(1);
            d=1-c;
            new_gen((2*par-1),:)=(potential_parents(2*par-1,:)*c+potential_parents(2*par,:)*d);%.*(0.95+0.1*rand(1));
            new_gen((2*par),:)=(potential_parents(2*par-1,:)*d+potential_parents(2*par,:)*c);%.*(0.95+0.1*rand(1));
        else 
            new_gen((2*par-1),:)=potential_parents((2*par-1),:);
            new_gen((2*par),:)=potential_parents((2*par),:);
        end
    end
end
%% each param taken from random parent
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%33
function new_gen = Crossover3(potential_parents,pc)
    [N,B] = size(potential_parents);
    couple_num=round(N/2);
    rc=rand(1,couple_num);
    new_gen=zeros(N,B);
    for par =1:couple_num
        if rc(1,par)<pc
            for n=1:B
                c=randi(2)-1;
                d=1;
                if c==1
                    d=0;
                end
                new_gen((2*par-1),n)=potential_parents(2*par-c,n);
                new_gen((2*par),n)=potential_parents(2*par-d,n);
                
            end
        else 
            new_gen((2*par-1),:)=potential_parents((2*par-1),:);
            new_gen((2*par),:)=potential_parents((2*par),:);
        end
    end
end
%% each param taken from random parent + adding weight factors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%33
function new_gen = Crossover4(potential_parents,pc)
    [N,B] = size(potential_parents);
    couple_num=round(N/2);
    rc=rand(1,couple_num);
    suma=zeros(2,B);
    new_gen=zeros(N,B);
    for par =1:couple_num
        if rc(1,par)<pc
            e=rand(1);
            f=1-e;
            suma(1,:)=(potential_parents(2*par-1,:)*e+potential_parents(2*par,:)*f);%.*(0.95+0.1*rand(1));
            suma(2,:)=(potential_parents(2*par-1,:)*f+potential_parents(2*par,:)*e);%.*(0.95+0.1*rand(1));
            
            for n=1:B
                c=randi(2);
                d=1;
                if c==1
                    d=2;
                end
                new_gen((2*par-1),n)=suma(c,n);
                new_gen((2*par),n)=suma(d,n);
                
            end
        else 
            new_gen((2*par-1),:)=potential_parents((2*par-1),:);
            new_gen((2*par),:)=potential_parents((2*par),:);
        end
    end
end
%% Blend (BLX-?) 
function new_gen = Crossover5(potential_parents,pc)
    [N,B] = size(potential_parents);
    couple_num=round(N/2);
    rc=rand(1,couple_num);
    suma=zeros(2,B);
    new_gen=zeros(N,B);
    eta_c=5;
    for par =1:couple_num
        if rc(1,par)<pc
            
            u = rand();
            if u <= 0.5
                beta = (2*u)^(1/(eta_c+1));
            else
                beta = (1/(2*(1-u)))^(1/(eta_c+1));
            end
      
            new_gen(2*par-1,:)=0.5 * ((1+beta)*potential_parents(2*par-1,:) + (1-beta)*potential_parents(2*par,:));
            new_gen(2*par,:)=0.5 * ((1-beta)*potential_parents(2*par-1,:) + (1+beta)*potential_parents(2*par,:));
                
            
        else 
            new_gen((2*par-1),:)=potential_parents((2*par-1),:);
            new_gen((2*par),:)=potential_parents((2*par),:);
        end
    end
end
%% Blend (BLX-?) + weights
function new_gen = Crossover6(potential_parents,pc)
    [N,B] = size(potential_parents);
    couple_num=round(N/2);
    rc=rand(1,couple_num);
    suma=zeros(2,B);
    new_gen=zeros(N,B);
    eta_c=5;
    for par =1:couple_num
        if rc(1,par)<pc
            
            c=rand(1);
            d=1-c;
            a=randi(B-1,1);
            suma1=(potential_parents(2*par-1,:)*c+potential_parents(2*par,:)*d);%.*(0.97+0.06*rand(1,B));
            suma2=(potential_parents(2*par-1,:)*d+potential_parents(2*par,:)*c);%.*(0.97+0.06*rand(1,B));
            
            u = rand();
            if u <= 0.5
                beta = (2*u)^(1/(eta_c+1));
            else
                beta = (1/(2*(1-u)))^(1/(eta_c+1));
            end
      
            new_gen(2*par-1,:)=0.5 * ((1+beta)*suma1 + (1-beta)*suma2);
            new_gen(2*par,:)=0.5 * ((1-beta)*suma1 + (1+beta)*suma2);
                
            
        else 
            new_gen((2*par-1),:)=potential_parents((2*par-1),:);
            new_gen((2*par),:)=potential_parents((2*par),:);
        end
    end
end
%%
function new_gen = Mutation(population, pm, UB, LB)
    [N,B] = size(population);
    rm=rand(N,B);
    mutation_factors = 0.95 + 0.1* rand(N, B);
    new_gen = population.*(rm > pm) + population.*(rm < pm).*mutation_factors;
    % clipping method or bounding method:
    new_gen = min(max(new_gen, LB), UB);
end

