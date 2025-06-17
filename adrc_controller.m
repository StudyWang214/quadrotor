% ADRC控制器模型y
function output = adrc_controller(params, desired_output, actual_output, dt, ctrller,p,lam)
    % 参数解析
    wox1 = params(1); wox2 = params(2); wcx = params(3); 
    woy1 = params(1); woy2 = params(2); wcy = params(3); 
    woz1 = params(1); woz2 = params(2); wcz = params(3); 
    % woy1 = params(4); woy2 = params(5); wcy = params(6); 
    % woz1 = params(7); woz2 = params(8); wcz = params(9); 
    wophi1 = params(4); wophi2 = params(5); wcphi = params(6); 
    wotheta1 = params(4); wotheta2 = params(5); wctheta = params(6); 
    wopsi1 = params(4); wopsi2 = params(5); wcpsi = params(6); 
    %  wophi1 = params(10); wophi2 = params(11); wcphi = params(12); 
    % wotheta1 = params(10); wotheta2 = params(11); wctheta = params(12); 
    % wopsi1 = params(10); wopsi2 = params(11); wcpsi = params(12); 
    % wophi1 = params(10); wophi2 = params(11); wcphi = params(12); 
    % wotheta1 = params(13); wotheta2 = params(14); wctheta = params(15); 
    % wopsi1 = params(16); wopsi2 = params(17); wcpsi = params(18); 
    kc = 1; ka = 1; 

    
    uux = ctrller(1); hx = ctrller(2); alx1 = ctrller(3); alx2 = ctrller(4); bex1 = ctrller(5); bex2 = ctrller(6);
    uuy = ctrller(7); hy = ctrller(8); aly1 = ctrller(9); aly2 = ctrller(10); bey1 = ctrller(11); bey2 = ctrller(12);
    uuz = ctrller(13); hz = ctrller(14); alz1 = ctrller(15); alz2 = ctrller(16); bez1 = ctrller(17); bez2 = ctrller(18);
    uuphi = ctrller(19); hphi = ctrller(20); alphi1 = ctrller(21); alphi2 = ctrller(22); bephi1 = ctrller(23); bephi2 = ctrller(24);
    uutheta = ctrller(25); htheta = ctrller(26); altheta1 = ctrller(27); altheta2 = ctrller(28); betheta1 = ctrller(29); betheta2 = ctrller(30);
    uupsi = ctrller(31); hpsi = ctrller(32); alpsi1 = ctrller(33); alpsi2 = ctrller(34); bepsi1 = ctrller(35); bepsi2 = ctrller(36);
    % uuphi = ctrller(1); hphi = ctrller(2); alphi1 = ctrller(3); alphi2 = ctrller(4); bephi1 = ctrller(5); bephi2 = ctrller(6);
    % uutheta = ctrller(7); htheta = ctrller(8); altheta1 = ctrller(9); altheta2 = ctrller(10); betheta1 = ctrller(11); betheta2 = ctrller(12);
    % uupsi = ctrller(13); hpsi = ctrller(14); alpsi1 = ctrller(15); alpsi2 = ctrller(16); bepsi1 = ctrller(17); bepsi2 = ctrller(18);

    phi = actual_output(7); theta = actual_output(9); psi = actual_output(11);

    px = p(1);py = p(2);pz = p(3);pphi = p(4);ptheta = p(5);ppsi = p(6);
    pxdot = p(7);pydot = p(8);pzdot = p(9);pphidot = p(10);pthetadot = p(11);ppsidot = p(12);

    unlamx = lam(1);ovlamx = lam(2);unlamy = lam(3);ovlamy = lam(4);unlamz = lam(5);ovlamz = lam(6);
    unlamphi = lam(7);ovlamphi = lam(8);unlamtheta = lam(9);ovlamtheta = lam(10);unlampsi = lam(11);ovlampsi = lam(12);

    
    % 观测器更新
    %X
    ex1 = actual_output(1) - alx1;
    alx1 = alx1 + dt * (alx2 + wox1^2 * ex1 + hx);
    alx2 = alx2 + dt * (2*wox1 * ex1); 
    hx = wcx^2 * (desired_output(1) - alx1) - alx2;
    % hx = k1x * (desired_output(1) - alx1) - alx2;
    ex2 = actual_output(2) - bex1;
    bex1 = bex1 + dt * (bex2 + wox2^2 * ex2 + uux);
    bex2 = bex2 + dt * (2*wox2 * ex2);
    % uux = 2*wcx * (hx-bex1)-bex2;
    % uux = k2x * (hx-bex1)-bex2;
    xx = (actual_output(1) - desired_output(1))/px;
    rx = (ovlamx+unlamx)/(2*px*(xx+unlamx)*(ovlamx-xx));
    epx = 1/2 * log((xx+unlamx)/(ovlamx-xx));
    uux = 2*wcx * (hx-bex1)-bex2 - 1/rx * kc * epx + xx*pxdot;
    % uux = 2*wcx * (hx-bex1)-bex2 + xx*pxdot;
    % % uux = k2x * (hx-bex1)-bex2 + xx*pxdot;

    %Y
    ey1 = actual_output(3) - aly1;
    aly1 = aly1 + dt * (aly2 + woy1^2 * ey1 + hy);
    aly2 = aly2 + dt * (2*woy1 * ey1);
    hy = wcy^2 * (desired_output(2) - aly1) - aly2;
    % hy = k1y * (desired_output(2) - aly1) - aly2;
    ey2 = actual_output(4) - bey1;
    bey1 = bey1 + dt * (bey2 + woy2^2 * ey2 + uuy);
    bey2 = bey2 + dt * (2*woy2 * ey2);
    % uuy = k2y * (hy-bey1)-bey2;
    % uuy = 2*wcy * (hy-bey1)-bey2;
    yy = (actual_output(3) - desired_output(2))/py;
    ry = (ovlamy+unlamy)/(2*py*(yy+unlamy)*(ovlamy-yy));
    epy = 1/2 * log((yy+unlamy)/(ovlamy-yy));
    uuy = 2*wcy * (hy-bey1)-bey2 - 1/ry * kc * epy + yy*pydot;
    % uuy = 2*wcy * (hy-bey1)-bey2 + yy*pydot;
    % uuy = k2y * (hy-bey1)-bey2 + yy*pydot;

    %Z
    ez1 = actual_output(5) - alz1;
    alz1 = alz1 + dt * (alz2 + woz1^2 * ez1 + hz);
    alz2 = alz2 + dt * (2*woz1 * ez1);
    hz = wcz^2 * (desired_output(3) - alz1) - alz2;
    % hz = k1z * (desired_output(3) - alz1) - alz2;
    ez2 = actual_output(6) - bez1;
    bez1 = bez1 + dt * (bez2 + woz2^2 * ez2 + uuz);
    bez2 = bez2 + dt * (2*woz2 * ez2);
    % uuz = 2*wcz * (hz-bez1)-bez2;
    % uuz = k2z * (hz-bez1)-bez2;
    zz = (actual_output(5) - desired_output(3))/pz;
    rz = (ovlamz+unlamz)/(2*pz*(zz+unlamz)*(ovlamz-zz));
    epz = 1/2 * log((zz+unlamz)/(ovlamz-zz));
    uuz = 2*wcz * (hz-bez1)-bez2 - 1/rz * kc * epz + zz*pzdot;
    % uuz = 2*wcz * (hz-bez1)-bez2 + zz*pzdot;
    % uuz = k2z * (hz-bez1)-bez2 + zz*pzdot;

    %Phi
    % d_phi = 0.1;
    d_phi = atan(cos(theta)*((uux*sin(psi)-uuy*cos(psi))/uuz));
    if isnan(d_phi)
        d_phi = 0;
    end

    ephi1 = actual_output(7) - alphi1;
    alphi1 = alphi1 + dt * (alphi2 + wophi1^2 * ephi1 + hphi);
    alphi2 = alphi2 + dt * (2*wophi1 * ephi1);
    hphi = wcphi^2 * (d_phi - alphi1) - alphi2;
    % hphi = k1phi * (d_phi - alphi1) - alphi2;
    ephi2 = actual_output(8) - bephi1;
    bephi1 = bephi1 + dt * (bephi2 + wophi2^2 * ephi2 + uuphi);
    bephi2 = bephi2 + dt * (2*wophi2 * ephi2);
    % uuphi = 2*wcphi * (hphi-bephi1)-bephi2;
    % uuphi = k2phi * (hphi-bephi1)-bephi2;
    phiphi = (actual_output(7) - d_phi)/pphi;
    rphi = (ovlamphi+unlamphi)/(2*pphi*(phiphi+unlamphi)*(ovlamphi-phiphi));
    epphi = 1/2 * log((phiphi+unlamphi)/(ovlamphi-phiphi));
    uuphi = 2*wcphi * (hphi-bephi1)-bephi2 - 1/rphi * ka * epphi + phiphi*pphidot;
    % uuphi = 2*wcphi * (hphi-bephi1)-bephi2 + phiphi*pphidot;
    % uuphi = k2phi * (hphi-bephi1)-bephi2 + phiphi*pphidot;

     %Theta
    % d_theta = 0.1;
    d_theta = atan((uux*cos(psi)+uuy*sin(psi))/uuz);
    if isnan(d_theta)
        d_theta = 0;
    end

    etheta1 = actual_output(9) - altheta1;
    altheta1 = altheta1 + dt * (altheta2 + wotheta1^2 * etheta1 + htheta);
    altheta2 = altheta2 + dt * (2*wotheta1 * etheta1);
    htheta = wctheta^2 * (d_theta - altheta1) - altheta2;
    % htheta = k1theta * (d_theta - altheta1) - altheta2;
    etheta2 = actual_output(10) - betheta1;
    betheta1 = betheta1 + dt * (betheta2 + wotheta2^2 * etheta2 + uutheta);
    betheta2 = betheta2 + dt * (2*wotheta2 * etheta2);
    % uutheta = 2*wctheta * (htheta-betheta1)-betheta2;
    % uutheta = k2theta * (htheta-betheta1)-betheta2;
    thetatheta = (actual_output(9) - d_theta)/ptheta;
    rtheta = (ovlamtheta+unlamtheta)/(2*ptheta*(thetatheta+unlamtheta)*(ovlamtheta-thetatheta));
    eptheta = 1/2 * log((thetatheta+unlamtheta)/(ovlamtheta-thetatheta));
    uutheta = 2*wctheta * (htheta-betheta1)-betheta2 - 1/rtheta * ka * eptheta + thetatheta*pthetadot;
    % uutheta = 2*wctheta * (htheta-betheta1)-betheta2 + thetatheta*pthetadot;
    % uutheta = k2theta * (htheta-betheta1)-betheta2 + thetatheta*pthetadot;

    %Psi    
    epsi1 = actual_output(11) - alpsi1;
    alpsi1 = alpsi1 + dt * (alpsi2 + wopsi1^2 * epsi1 + hpsi);
    alpsi2 = alpsi2 + dt * (2*wopsi1 * epsi1);
    hpsi = wcpsi^2 * (desired_output(4) - alpsi1) - alpsi2;
    % hpsi = k1psi * (desired_output(4) - alpsi1) - alpsi2;
    epsi2 = actual_output(12) - bepsi1;
    bepsi1 = bepsi1 + dt * (bepsi2 + wopsi2^2 * epsi2 + uupsi);
    bepsi2 = bepsi2 + dt * (2*wopsi2 * epsi2);
    % uupsi = 2*wcpsi * (hpsi-bepsi1)-bepsi2;
    % uupsi = k2psi * (hpsi-bepsi1)-bepsi2;
    psipsi = (actual_output(11) - desired_output(4))/ppsi;
    rpsi = (ovlampsi+unlampsi)/(2*ppsi*(psipsi+unlampsi)*(ovlampsi-psipsi));
    eppsi = 1/2 * log((psipsi+unlampsi)/(ovlampsi-psipsi));
    uupsi = 2*wcpsi * (hpsi-bepsi1)-bepsi2 - 1/rpsi * ka * eppsi + psipsi*ppsidot;
    % % uupsi = 2*wcpsi * (hpsi-bepsi1)-bepsi2 + psipsi*ppsidot;
    % uupsi = k2psi * (hpsi-bepsi1)-bepsi2 + psipsi*ppsidot;

    % output = [uux; hx; alx1; alx2; bex1; bex2; 
    %           uuy; hy; aly1; aly2; bey1; bey2; 
    %           uuz; hz; alz1; alz2; bez1; bez2];
    % 
    % output = [uuphi; hphi; alphi1; alphi2; bephi1; bephi2;
    %           uutheta; htheta; altheta1; altheta2; betheta1; betheta2;
    %           uupsi; hpsi; alpsi1; alpsi2; bepsi1; bepsi2;
    %           d_phi;d_theta];
 
    output = [uux; hx; alx1; alx2; bex1; bex2; 
              uuy; hy; aly1; aly2; bey1; bey2; 
              uuz; hz; alz1; alz2; bez1; bez2; 
              uuphi; hphi; alphi1; alphi2; bephi1; bephi2;  
              uutheta; htheta; altheta1; altheta2; betheta1; betheta2;  
              uupsi; hpsi; alpsi1; alpsi2; bepsi1; bepsi2;
              d_phi;d_theta];
end