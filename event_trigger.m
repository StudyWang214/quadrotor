function [output, flag] = event_trigger(uav, uav_tp, ctrller, desird)
x = uav(1); vx = uav(2); 
y = uav(3); vy = uav(4);
z = uav(5); vz = uav(6);
phi = uav(7); vphi = uav(8);
theta = uav(9); vtheta = uav(10);
psi = uav(11); vpsi = uav(12);
aphi = uav(13); atheta = uav(14); apsi = uav(15);
U1 = uav(16); U2 = uav(17); U3 = uav(18); U4 = uav(19);

xtp = uav_tp(1); vxtp = uav_tp(2); 
ytp = uav_tp(3); vytp = uav_tp(4);
ztp = uav_tp(5); vztp = uav_tp(6);
phitp = uav_tp(7); vphitp = uav_tp(8);
thetatp = uav_tp(9); vthetatp = uav_tp(10);
psitp = uav_tp(11); vpsitp = uav_tp(12);
U1tauq = uav_tp(16); U2tauq = uav_tp(17); U3tauq = uav_tp(18); U4tauq = uav_tp(19);

zx1 = ctrller(3); zy1 = ctrller(9); zz1 = ctrller(15);
zphi1 = ctrller(21); ztheta1 = ctrller(27); zpsi1 = ctrller(33);

xd = desird(1); yd = desird(2); zd = desird(3);
phid = ctrller(37); thetad = ctrller(38); psid = desird(4);

if abs(x-xtp) > 0.01;    xtp = x;    xtpf = 1; else;    xtpf = 0; end
if abs(y-ytp) > 0.01;    ytp = y;    ytpf = 2; else;    ytpf = 0; end
if abs(z-ztp) > 0.01;    ztp = z;    ztpf = 3; else;    ztpf = 0; end
if abs(phi-phitp) > 0.010;    phitp = phi;    phitpf = 4; else;    phitpf = 0; end
if abs(theta-thetatp) > 0.010;    thetatp = theta;    thetatpf = 5; else;    thetatpf = 0; end
if abs(psi-psitp) > 0.010;    psitp = psi;    psitpf = 6; else;    psitpf = 0; end

if abs(vx-vxtp) > 0.03;    vxtp = vx;    vxtpf = 1; else;    vxtpf = 0; end
if abs(vy-vytp) > 0.03;    vytp = vy;    vytpf = 2; else;    vytpf = 0; end
if abs(vz-vztp) > 0.03;    vztp = vz;    vztpf = 3; else;    vztpf = 0; end
if abs(vphi-vphitp) > 0.01;    vphitp = vphi;    vphitpf = 4; else;    vphitpf = 0; end
if abs(vtheta-vthetatp) > 0.01;    vthetatp = vtheta;    vthetatpf = 5; else;    vthetatpf = 0; end
if abs(vpsi-vpsitp) > 0.01;    vpsitp = vpsi;    vpsitpf = 6; else;    vpsitpf = 0; end

a = abs(U1-U1tauq);
b =5*exp(-10*sqrt((zx1-xd)^2+(zy1-yd)^2+(zz1-zd)^2));
% c = abs(U1-U1tauq);
% d =5*exp(-10*sqrt((zx1-xd)^2+(zy1-yd)^2+(zz1-zd)^2));
if abs(U1-U1tauq) > 5*exp(-10*sqrt((zx1-xd)^2+(zy1-yd)^2+(zz1-zd)^2)) ;    U1tauq = U1;    U1tpf = 1;else ;    U1tpf = 0;end
if abs(U2-U2tauq) > 1000*exp(-10*abs(zphi1-phid)) ;    U2tauq = U2;    U2tpf = 2;else ;    U2tpf = 0;end
if abs(U3-U3tauq) > 1000*exp(-10*abs(ztheta1-thetad)) ;    U3tauq = U3;    U3tpf = 3;else ;    U3tpf = 0;end
if abs(U4-U4tauq) > 10*exp(-10*abs(zpsi1-psid)) ;    U4tauq = U4;    U4tpf = 4;else ;    U4tpf = 0;end

output = [xtp;vxtp;ytp;vytp;ztp;vztp;phitp;vphitp;thetatp;vthetatp;psitp;vpsitp;aphi;atheta;apsi;U1tauq;U2tauq;U3tauq;U4tauq];
flag = [xtpf;ytpf;ztpf;phitpf;thetatpf;psitpf;vxtpf;vytpf;vztpf;vphitpf;vthetatpf;vpsitpf;U1tpf;U2tpf;U3tpf;U4tpf;a;b];

end