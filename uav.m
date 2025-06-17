% 被控对象
function output = uav(u,act_out_his,dt,load,dis,ab)
    ux = u(1); uy = u(2); uz = u(3);
    U2 = u(4); U3 = u(5); U4 = u(6); 
    x = act_out_his(1); vx = act_out_his(2);
    y = act_out_his(3); vy = act_out_his(4);
    z = act_out_his(5); vz = act_out_his(6);
    phi = act_out_his(7); vphi = act_out_his(8);
    theta = act_out_his(9); vtheta = act_out_his(10);
    psi = act_out_his(11); vpsi = act_out_his(12);
    aphi = act_out_his(13); atheta = act_out_his(14); apsi = act_out_his(15);

    w1 = dis(1); w2 = dis(2); w3 = dis(3); w4 = dis(4);

    a1 = ab(1); a2 = ab(2); a3 = ab(3); a4 = ab(4);
    b1 = ab(5); b2 = ab(6); b3 = ab(7); b4 = ab(8); 

    m = 2.5;    g = 9.8;    l = 0.6;
    kc = 0.06;    ka = 0.02;
    jx = 1.4;    jy = 1.4;    jz = 2.8;
    A = [ 1   1   1   1;
     -(sqrt(2)/2)*l    (sqrt(2)/2)*l   (sqrt(2)/2)*l  -(sqrt(2)/2)*l;
      (sqrt(2)/2)*l    (sqrt(2)/2)*l  -(sqrt(2)/2)*l  -(sqrt(2)/2)*l;
      kc -kc  kc  -kc];

    mm = load(1);
    lx = load(2);
    ly = load(3);
    lz = load(4);

    jxx = jx + mm*(ly^2 + lz^2);
    jyy = jy + mm*(lx^2 + lz^2);
    jzz = jz + mm*(lx^2 + ly^2);
    jxy = mm*lx*ly;
    jxz = mm*lx*lz;
    jyz = mm*ly*lz;

    U1 = uz/(cos(phi)*cos(theta));

    U = [U1;U2;U3;U4];
    f = A\U;
    ff(1) = a1*f(1) + b1;
    ff(2) = a2*f(2) + b2;
    ff(3) = a3*f(3) + b3;
    ff(4) = a4*f(4) + b4;
    UU = A*[ff(1);ff(2);ff(3);ff(4)];
    fff = A\UU;
    ffff = fff(1)+fff(2)+fff(3)+fff(4);
    % ff = f(1)+f(2)+f(3)+f(4);
    ux = ffff*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi));
    uy = ffff*(sin(psi)*sin(theta)*cos(phi)-sin(phi)*cos(psi));
    uz = ffff*(cos(theta)*cos(phi));


    ax = (ux-ka*vx)/(m+mm)+w2;%+wx(k);
    % ax = (ux-ka*vx)/(m);
    vx = vx+dt*(ax+w1);
    x = x+dt*vx;

    % ay = (uy-ka*vy)/(m);
     ay = (uy-ka*vy)/(m+mm)+w2;%+wy(k); 
    vy = vy+dt*(ay+w1);
    y = y+dt*vy;

    % az = (uz-ka*vz)/(m)-g;
    az = (uz-ka*vz)/(m+mm)-g+w2;%+wz(k); 
    vz = vz+dt*(az+w1);
    z = z+dt*vz;

    aphi = (UU(2)+jyz*(vtheta^2-vpsi^2)+(jyy-jzz)*vtheta*vpsi+jxz*(vphi*vtheta+apsi)+jxy*(atheta-vphi*vpsi))/jxx+ w4; 
    atheta = (UU(3)+jxz*(vpsi^2-vphi^2)+(jzz-jxx)*vphi*vpsi+jxz*(vtheta*vpsi+apsi)+jyz*(apsi-vphi*vtheta))/jyy+ w4; 
    apsi = (UU(4)+jxy*(vphi^2-vtheta^2)+(jxx-jyy)*vphi*vtheta+jyz*(vphi*vpsi+atheta)+jxz*(aphi-vtheta*vpsi))/jzz+ w4; 

    vphi = vphi+dt*(aphi + w3); 
    phi = phi+dt*vphi;

    vtheta = vtheta+dt*(atheta + w3); 
    theta = theta+dt*vtheta;

    vpsi = vpsi+dt*(apsi + w3); 
    psi = psi+dt*vpsi;

    % output = [x;vx;y;vy;z;vz];
    % output = [phi;vphi;theta;vtheta;psi;vpsi];
    output = [x;vx;y;vy;z;vz;phi;vphi;theta;vtheta;psi;vpsi;aphi;atheta;apsi;U1;U2;U3;U4;f(1);f(2);f(3);f(4)];
end