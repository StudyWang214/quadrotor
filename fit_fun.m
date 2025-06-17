% 目标函数（控制误差的绝对值）
function error = fit_fun(virtual, desird, time)
    t = length(time);
    % 计算控制误差
    error = 0; exyz = 0;
    for k = 1:t
        ex = desird(1,k)-virtual(1,k);
        ey = desird(2,k)-virtual(2,k);
        ez = desird(3,k)-virtual(3,k);
        % ephi = abs(desird(4,k)-virtual(4,k));
        % etheta = abs(desird(5,k)-virtual(5,k));
        % epsi = abs(desird(6,k)-virtual(6,k));
        exyz = exyz + time(k) * sqrt(sqrt(ex^2+ey^2+ez^2));
        % exyz = exyz + sqrt(sqrt(ex^2+ey^2+ez^2));
    end
    stdphi = std(virtual(4,k));
    stdtheta = std(virtual(5,:));
    stdpsi = std(virtual(6,:));
    error = 1*exyz +500*(stdphi+stdtheta+stdpsi);

    % figure;
    % plot(time,virtual,time,desird);
end


% function error = fit_fun(virtual, desird, time,dt)
%     t = dt:dt:time;
%     % 计算控制误差
%     error = 0;
%     for k = 1:time
%         e = abs(desird(k)-virtual(k));
%         error = error + t(k) * e ;
%     end
% end