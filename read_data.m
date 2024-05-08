clc, clear all, close all

%%
trajectory_num=6;
repetition_num=5;
repetition_total_num=trajectory_num*repetition_num;

dataset_combined = cell(1, repetition_total_num);

for rep = 1:1:repetition_total_num
    % a) read motor_signals
    pointer_w='odom_'+string(rep)+'.csv'
    dataArray = readtable(pointer_w, 'Delimiter', ';');
    row_num=size(dataArray,1);
    pos_str=dataArray.x_position;
    vel_str=dataArray.x_velocity;
    time_odom_str=dataArray.time;    

    pattern = '[-+]?\d+(\.\d+)?';
    pos=zeros(row_num,8);
    vel=zeros(row_num,8);
    time_odom=zeros(row_num,1);
    
    for i=1:row_num
        matches_pos = regexp(pos_str{i}, pattern, 'match');
        pos(i,:) = str2double(matches_pos);
        matches_vel = regexp(vel_str{i}, pattern, 'match');
        vel(i,:) = str2double(matches_vel);
        dtObj=datetime(time_odom_str{i}, 'InputFormat', 'yyyy/MM/dd/HH:mm:ss.SSSSSS');
        time_odom(i,1) = posixtime(dtObj); 
    end
    dt=diff(time_odom);
    % g = wheel_position, w = wheel_omega    
    g1=pos(:,5);
    w1=vel(:,6);
    g2=pos(:,1);
    w2=vel(:,2);
    g3=pos(:,3);
    w3=vel(:,4);
    g4=pos(:,7);
    w4=vel(:,8);
    
    % b) real positions - OptiTrack
    pointer_abs="abs_"+string(rep)+".csv"
    dataArray2= readtable(pointer_abs, 'Delimiter', ';');
    x_abs=dataArray2.x_pose_position_x;
    y_abs=dataArray2.x_pose_position_y;

    ang_q_x=dataArray2.x_pose_orientation_x;
    ang_q_y=dataArray2.x_pose_orientation_y;
    ang_q_z=dataArray2.x_pose_orientation_z;
    ang_q_w=dataArray2.x_pose_orientation_w;
    [phi,theta, psi_abs]=quat_to_eul(ang_q_x,ang_q_y,ang_q_z,ang_q_w);

    time_abs_str=dataArray2.time;
    row_num_abs=size(dataArray2,1)
    time_abs=zeros(row_num_abs,1);   
    for i=1:row_num_abs
        dtObj=datetime(time_abs_str{i}, 'InputFormat', 'yyyy/MM/dd/HH:mm:ss.SSSSSS');
        time_abs(i,1) = posixtime(dtObj); 
    end
    % some data can come at the same time
    for i = 1:length(time_odom)-1
        if time_odom(i)>=time_odom(i+1)
            time_odom(i+1)=time_odom(i+1)+1e-6;
        end
    end
    for i = 1:length(time_abs)-1
        if time_abs(i)>=time_abs(i+1)
            time_abs(i+1)=time_abs(i+1)+1e-6;
        end
    end
    % sync motor data to OptiTrak time
    g1 = interp1(time_odom, g1, time_abs, 'linear', 'extrap');
    g2 = interp1(time_odom, g2, time_abs, 'linear', 'extrap');
    g3 = interp1(time_odom, g3, time_abs, 'linear', 'extrap');
    g4 = interp1(time_odom, g4, time_abs, 'linear', 'extrap');
    w1 = interp1(time_odom, w1, time_abs, 'linear', 'extrap');
    w2 = interp1(time_odom, w2, time_abs, 'linear', 'extrap');
    w3 = interp1(time_odom, w3, time_abs, 'linear', 'extrap');
    w4 = interp1(time_odom, w4, time_abs, 'linear', 'extrap');  
    jp = 30;
    time_abs = time_abs - time_abs(jp);
    for t=2:length(time_abs)
        % keep angle in positive range if angle increases 
        if(psi_abs(t)-psi_abs(t-1))>pi
            psi_abs(t:end)=psi_abs(t:end)-2*pi;
        end
        % keep angle in negative range if angle decreases  
        if((psi_abs(t)-psi_abs(t-1))<-pi)
            psi_abs(t:end)=psi_abs(t:end)+2*pi;
        end
    end

    dt=diff(time_abs);
    dataset_trajectory=[time_abs(jp:end),[0;dt(jp:end)],x_abs(jp:end),y_abs(jp:end), psi_abs(jp:end)', g1(jp:end), g2(jp:end), g3(jp:end),g4(jp:end),w1(jp:end),w2(jp:end),w3(jp:end),w4(jp:end)];
    % vr=1, dt=2, x=3, y=4, psi=5, g1=6, g2=7, g3=8, g4=9,
    % w1=10, w2=11, w3=12, w4=13
    dataset_combined{rep}=dataset_trajectory;
end
% save data
save('dataset_combined.mat', 'dataset_combined')

%%
function [phi, theta, psi] = quat_to_eul(ax,ay,az,aw)
    phi=zeros(1,length(ax));
    theta=phi(1,:);
    psi=phi(1,:);
    for k = 1:length(ax)
        x=ax(k);
        y=ay(k);
        z=az(k);
        w=aw(k);
        phi(k) = atan2(2 * (w * x + y * z), 1 - 2 * (x^2 + y^2));
        theta(k) = asin(2 * (w * y - z * x));
        psi(k) = atan2(2 * (w * z + x * y), 1 - 2 * (y^2 + z^2));
    end
end