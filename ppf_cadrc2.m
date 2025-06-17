% PSO-based Online Adaptive ADRC in MATLAB
%加入事件触发

% 清空环境
clc;
clear;
close all;

% 参数设置
e_hold = 0.5; % 误差阈值，触发PSO重新优化

%粒子群初始化
n_para = 6; %粒子群维度（参数数量）%%%%%%%%%%%%%%%%%%%%%%%%%
n_pso = 100; % 粒子数量
n_iter = 20; % 迭代次数
w_max = 0.8; w_min = 0.2;
c1 = 2; c2 = 2; 
gbest_p = zeros(1,n_para);

% 模拟控制系统
sim_t = 20; % 仿真时间
dt = 0.01; % 时间步长
time = dt:dt:sim_t;
n_steps = length(time);

% 初始化控制系统的状态
ctrl_e = zeros(1, n_steps);
ctrl_para_his = zeros(n_steps, n_para); % 记录参数变化
pso_trigger = zeros(1,n_steps);
trigtime = 0;

%初始化被控对象
%输出维度:[x,vx, y, vy, z, vz,phi, vphi, theta, vtheta, psi, vpsi, aphi, atheta, apsi, U1, U2, U3, U4]%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_output = 23;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a_output = zeros(n_output, n_steps); % 实际输出
% a_output(3,1) = 5; 
a_output_his = a_output(:,1);%初始值
a_output_tp_his = a_output(:,1);%事件触发记录值
et_time = zeros(18);%[x,y,z,phi,theta,psi,vx,vy,vz,vphi,vtheta,vpsi,U1,U2,U3,U4]

% 初始化控制器状态
%[u; h; al1; al2; be1; be2]六个一组+[dphi; dtheta]%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n_ctrl = 38; %控制器参数维度 
ctrller = zeros(n_ctrl,n_steps);
ctrller_his = zeros(n_ctrl,1);

% 期望输出
x0=0;y0=0;z0=3;psi0=0;
for t = 1:n_steps
    % d_output(1:4,t) = [x0;y0;z0;psi0];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % if t <= 300
    %     x0 = x0 + 0.05;
    %     y0 = y0 + 0.05;
    %     z0 = z0 + 0.05;
    % end
    % psi0 = 0;

    d_output(1,t) = 0.5*time(t)*sin(0.8*time(t));%x
    d_output(2,t) = 0.5*time(t)*cos(0.8*time(t));%y
    z0 = z0 + 0.0005;
    d_output(3,t) = z0;%z
    d_output(4,t) = 0;%psi
end

%可变载荷[mm; llx; lly; llz]
load = zeros(4,n_steps);
for t = 1:n_steps
    % if t < 500
        load(1,t) = 1.5;%mm
        load(2,t) = 0.1*cos(1*time(t))+0.2*sin(2*time(t));%llx
        load(3,t) = 0.2*cos(2*time(t))+0.1*sin(1*time(t));%lly
        load(4,t) = 0.3;%llz
    % elseif t >= 500 && t < 1500
    %     load(1,t) = load(1,t-1) - 0.001;%mm
    %     load(2,t) = 0.1*cos(1*time(t))+0.1*sin(2*time(t));%llx
    %     load(3,t) = 0.1*cos(2*time(t))+0.1*sin(1*time(t));%lly
    %     load(4,t) = load(4,t-1) - 0.0002;%llz
    % else
    %     load(1,t) = 1;%mm
    %     load(2,t) = 0.1*cos(1*time(t))+0.1*sin(2*time(t));%llx
    %     load(3,t) = 0.1*cos(2*time(t))+0.1*sin(1*time(t));%lly
    %     load(4,t) = 0.1;%llz
    % end
end

%扰动
dis(1,:) = 0*randn(1, n_steps);
dis(2,:) = 0.5*randn(1, n_steps);
dis(3,:) = 0*randn(1, n_steps);
dis(4,:) = 0.5*randn(1, n_steps);

% figure;
% plot(time, dis, 'LineWidth', 1.5);

%预设性能
px0 = 3.5; pxinfty = 0.3;mux = 1; unlamx = 0.5; ovlamx = 0.5;
py0 = 3.5; pyinfty = 0.3;muy = 1; unlamy = 0.5; ovlamy = 0.5;
pz0 = 3.5; pzinfty = 0.3;muz = 1; unlamz = 0.5; ovlamz = 1;
pphi0 = 3; pphiinfty = 0.1;muphi = 0.5; unlamphi = 0.9; ovlamphi = 0.9;
ptheta0 = 3; pthetainfty = 0.1;mutheta = 0.5; unlamtheta = 0.9; ovlamtheta = 0.9;
ppsi0 = 3; ppsiinfty = 0.1;mupsi = 0.5; unlampsi = 0.9; ovlampsi = 0.9;

px = (px0-pxinfty)*exp(-mux.*time)+pxinfty;    pxdot = -mux*(px0-pxinfty)*exp(-mux.*time);
py = (py0-pyinfty)*exp(-muy.*time)+pyinfty;    pydot = -muy*(py0-pyinfty)*exp(-muy.*time);
pz = (pz0-pzinfty)*exp(-muz.*time)+pzinfty;    pzdot = -muz*(pz0-pzinfty)*exp(-muz.*time);
pphi = (pphi0-pphiinfty)*exp(-muphi.*time)+pphiinfty;    pphidot = -muphi*(pphi0-pphiinfty)*exp(-muphi.*time);
ptheta = (ptheta0-pthetainfty)*exp(-mutheta.*time)+pthetainfty;    pthetadot = -mutheta*(ptheta0-pthetainfty)*exp(-mutheta.*time);
ppsi = (ppsi0-ppsiinfty)*exp(-mupsi.*time)+ppsiinfty;    ppsidot = -mupsi*(ppsi0-ppsiinfty)*exp(-mupsi.*time);

p = [px;py;pz;pphi;ptheta;ppsi;pxdot;pydot;pzdot;pphidot;pthetadot;ppsidot];
lam = [unlamx,ovlamx,unlamy,ovlamy,unlamz,ovlamz,unlamphi,ovlamphi,unlamtheta,ovlamtheta,unlampsi,ovlampsi];

%故障
a = ones(4,n_steps); %[a1,a2,a3,a4,b1,b2,b3,b4]
% a(1,:) = 0.5;
for t = 1:n_steps
    % if a(1,t-1) > 0.5
    %     a(1,t) = a(1,t-1)-0.001;
    % else
    %     a(1,t) = 0.5;
    % end
    if t > 1000
        a(1,t) = 0.5;
    end
end
b = zeros(4,n_steps);
% for t = 1:n_steps
%     if t>500
%         b(1,t) = 10;
%         b(2,t) = 10;
%     end
% end

ab = [a;b];

% figure;
% plot3(d_output(1,:), d_output(2,:),d_output(3,:),'r', 'LineWidth', 1.5);

% 主循环
for step = 1:n_steps

    % 如果误差超过阈值，触发PSO重新优化
    if step == 1 || (ctrl_e(step-1) > e_hold && step ~= 1000  && step-trigtime>200)
        pso_trigger(step) = 1;
        trigtime = step;

        v_pso = 20*rand(n_pso,n_para)-10; %随机粒子初始速度
        x_pso = 20*rand(n_pso,n_para); %随机粒子初始位置
        x_pso(1,:) = gbest_p;
        pbest_p = x_pso;
        pbest_v = inf(n_pso, 1);
        gbest_p = zeros(1,n_para);
        gbest_v = inf;

        for dd = 1:n_iter
            w = w_max - (w_max - w_min) * (dd / n_iter); % 动态调整惯性权重

            %粒子群参数择优
            for i = 1:n_pso
                %粒子群控制器初始状态赋值
                pctrller(:,1) = ctrller_his;
                pso_ctrl_para = x_pso(i,:);
                %粒子群被控对象初始状态赋值
                v_output = a_output_his;
                v_output_his = a_output_his;
                v_outout_tp = a_output_tp_his;

                % fgs = 3;%pso仿真时间为剩余时间的几分之1
                % maxptime = floor((n_steps-step)/fgs);
                maxptime = 400;

                for ptime = 1:maxptime
                    %事件触发更新
                    [v_outout_tp, ~] = event_trigger(v_output(:,ptime), v_outout_tp, pctrller(:,ptime),d_output(:,step+ptime-1));
                    %记算控制器输出（u+参数）%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    pctrller(:,ptime+1) = adrc_controller(pso_ctrl_para, d_output(:,step+ptime-1), v_outout_tp, dt, pctrller(:,ptime),p(:,step+ptime-1), lam);
                    %记算虚拟被控对象输出%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    v_output(:,ptime+1) = uav(pctrller([1,7,13,19,25,31],ptime+1),v_output_his,dt,load(:,ptime), dis(:,step+ptime-1), ab(:,step+ptime-1));
                    v_output_his = v_output(:,ptime+1);
                end
                % 计算当前参数组控制误差%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                Dd_output = [d_output(1:3,step:step+maxptime-1); pctrller(37:38,1:maxptime); d_output(4,step:step+maxptime-1)];
                error = fit_fun(v_output([1,3,5,7,9,11],:), Dd_output,time(step:step+maxptime-1));
                pso_e(i) = error;
                %更新粒子状态
                v_pso(i, :) = w * v_pso(i, :) + ...
                              c1 * rand * (pbest_p(i, :) - x_pso(i, :)) + ...
                              c2 * rand * (gbest_p - x_pso(i, :));
                x_pso(i, :) = abs(x_pso(i, :) + v_pso(i, :));
                %得到个体最优位置
                if pso_e(i) < pbest_v(i)
                    pbest_v(i) = pso_e(i);
                    pbest_p(i,:) = pso_ctrl_para;
                end

            end
            %得到全局最优位置
            [gbest_v,g] = min(pbest_v);
            gbest_p = pbest_p(g,:);
            best(dd) = gbest_v;
            % if gbest_v <= 10
            %     break
            % end
        end

        ctrller_params = gbest_p;

    end
    %进行控制
    %事件触发更新
    [a_output_tp(:,step), et_time(:,step)] = event_trigger(a_output_his, a_output_tp_his, ctrller_his,d_output(:,step));
    %控制器
    ctrller(:,step) = adrc_controller(ctrller_params, d_output(:,step), a_output_tp(:,step), dt, ctrller_his,p(:,step), lam);
    %被控对象%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    a_output(:,step) = uav(ctrller([1,7,13,19,25,31],step),a_output_his,dt,load(:,step), dis(:,step), ab(:,step));
    % 记算实时误差%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ctrl_e(step) = sqrt((d_output(1,step) - a_output(1,step))^2 ...
                      + (d_output(2,step) - a_output(3,step))^2 ...
                      + (d_output(3,step) - a_output(5,step))^2);...
               % + 1 * abs(ctrller(37,step) - a_output(7,step))...
               % + 1 * abs(ctrller(38,step) - a_output(9,step))...
               % + 1 * abs(d_output(4,step) - a_output(11,step));

    ctrl_para_his(step, :) = ctrller_params;
    a_output_his = a_output(:,step);
    ctrller_his = ctrller(:,step);
    a_output_tp_his = a_output_tp(:,step);

end

% 绘制结果
figure;
subplot(3, 1, 1);
plot(time, d_output(1,:), 'b', time, a_output(1,:), 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('x-Output');
legend('Desired Output', 'Actual x Output');
subplot(3, 1, 2);
plot(time, d_output(2,:), 'b', time, a_output(3,:), 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('y-Output');
legend('Desired Output', 'Actual y Output');
subplot(3, 1, 3);
plot(time, d_output(3,:), 'b', time, a_output(5,:), 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('z-Output');
legend('Desired Output', 'Actual z Output');
title('ADRC Controller Performance');
% plot(time, ctrl_e(1:1000), 'g', 'LineWidth', 1.5);
% xlabel('Time');
% ylabel('Control Error');
% title('Control Error Over Time');

figure('Name','3D_Position')
plot3(d_output(1,:),d_output(2,:),d_output(3,:),'r','LineWidth',1.5);
hold on;
plot3(a_output(1,:),a_output(3,:),a_output(5,:),'b','LineWidth',1.5);
xlim([-12 12]);ylim([-12 12]);%zlim([-0.2 4.5]);
xlabel('x/m');ylabel('y/m');zlabel('z/m');
legend('Target','Actual','FontSize', 10);
grid on;

figure;
subplot(3, 1, 1);
plot(time, ctrller(37,:), 'b', time, a_output(7,:), 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('phi-Output');
% legend('Desired Output', 'Actual phi Output');
subplot(3, 1, 2);
plot(time, ctrller(38,:), 'b', time, a_output(9,:), 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('theta-Output');
% legend('Desired Output', 'Actual theta Output');
subplot(3, 1, 3);
plot(time, d_output(4,:), 'b', time, a_output(11,:), 'r', 'LineWidth', 1.5);
xlabel('Time');
ylabel('psi-Output');
% legend('Desired Output', 'Actual psi Output');
title('ADRC Controller Performance');

figure('Name','PSO_Trigger');
plot(time,pso_trigger,'b','LineWidth',2)
xlabel('time(s)');ylabel('pso_trigger');

figure;
subplot(2, 2, 1);
plot(time, a_output(16,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
ylabel('U1');
legend( 'U1 Output');
subplot(2, 2, 2);
plot(time, a_output(17,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
ylabel('U2');
legend('U2 Output');
subplot(2, 2, 3);
plot(time, a_output(18,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
ylabel('U3');
legend('U3 Output');
subplot(2, 2, 4);
plot(time, a_output(19,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
ylabel('U4');
legend('U4 Output');
title('ADRC Controller Performance');

figure('Name','Position_Event_Trigger');
subplot(3, 1, 1);
plot(time, et_time(1,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
% ylabel(' ');
%legend('Desired Output', 'X_Event_Trigger');
subplot(3, 1, 2);
plot(time, et_time(2,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
% ylabel(' ');
%legend('Desired Output', 'Y_Event_Trigger');
subplot(3, 1, 3);
plot(time, et_time(3,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
% ylabel(' ');
%legend('Desired Output', 'Z_Event_Trigger');
title('Position_Event_Trigger');

figure('Name','error');
plot(time,ctrl_e,'b','LineWidth',2)
xlabel('time(s)');ylabel('pso_trigger');

figure('Name','111');
subplot(3, 1, 1);
plot(time, ctrller(1,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
ylabel('x-Output');
% legend('Desired Output', 'Actual x Output');
subplot(3, 1, 2);
plot(time, ctrller(7,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
ylabel('y-Output');
% legend('Desired Output', 'Actual y Output');
subplot(3, 1, 3);
plot(time, ctrller(13,:), 'b', 'LineWidth', 1.5);
xlabel('Time');
ylabel('z-Output');
% legend('Desired Output', 'Actual z Output');
% title('ADRC Controller Performance');

% figure;
% subplot(2, 2, 1);
% plot(time, a_output(1,:), 'b', 'LineWidth', 1.5);
% xlabel('Time');
% % ylabel('x-Output');
% % legend('Desired Output', 'Actual x Output');
% subplot(2, 2, 2);
% plot(time, a_output(2,:), 'b', 'LineWidth', 1.5);
% xlabel('Time');
% % ylabel('x-Output');
% % legend('Desired Output', 'Actual x Output');
% subplot(2, 2, 3);
% plot(time, a_output(3,:), 'b', 'LineWidth', 1.5);
% xlabel('Time');
% % ylabel('x-Output');
% % legend('Desired Output', 'Actual x Output');
% subplot(2, 2, 4);
% plot(time, a_output(4,:), 'b', 'LineWidth', 1.5);
% xlabel('Time');
% % ylabel('x-Output');
% % legend('Desired Output', 'Actual x Output');

% figure('Name','PSO_Triggered bound');
% plot(time,0.5-(0.3)*exp(-5*ctrl_e),'b','LineWidth',2)
% xlabel('time(s)');ylabel('pso_trigger');

% 
% figure('Name','PSO_Trigger');
% plot(1:n_iter,best,'b','LineWidth',2)
% xlabel('time(s)');ylabel('pso_trigger');
% 
% a=time(1:maxptime);
% figure;
% plot(time(1:maxptime), v_output(1,:), 'r', 'LineWidth', 1.5);






