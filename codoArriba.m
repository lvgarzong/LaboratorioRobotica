function [q1, q2, q3, q4, q5, q6] = codoArriba(xe, ye, ze, roll, pitch, yaw, L1, L2, L3, L4, L5, L, Robot)
    R0T = rpy2r(roll, pitch, yaw, 'deg');

    pe = [xe; ye; ze];
    pw = pe - L5*R0T(:,3);
    xw = pw(1);
    yw = pw(2);
    zw = pw(3);

    r = sqrt(xw^2 + yw^2);
    Lxy = r + L2;
    Lz = zw - L1;
    Lp = sqrt(Lxy^2 + Lz^2);

    q1 = atan2(-yw,-xw);

    alpha = atan2(Lz, Lxy);
    cosB = (L3^2 + Lp^2 - L4^2)/(2*L3*Lp);
    senB = sqrt(1 - cosB^2);
    beta = atan2(senB, cosB);
    q2 = ((pi/2) - (alpha + beta));

    cosY = (L3^2 + L4^2 - Lp^2)/(2*L3*L4);
    senY = sqrt(1 - cosY^2);
    gama = atan2(senY, cosY);
    phi = pi - gama;
    q3 = (pi/2) + phi;

    MTH03 = L(1).A(q1)*L(2).A(q2)*L(3).A(q3);
   % MTH03_eval = eval(R03_sim)
    [R03, P03] = tr2rt(MTH03);
    [RT6, PT6] = tr2rt(Robot.tool^(-1));
    [R6T, P6T] = tr2rt(Robot.tool);

    R06 = R0T*RT6;
    R36 = (R03^(-1))*R06;
    R3T = (R03^(-1))*R0T;

    q4 = atan2(R36(3,3),R36(1,3));

    c5 = -R36(2,3);
    s5 = sqrt(1 - c5^2);
    q5 = atan2(s5,c5);

    q6 = atan2(-R36(2,2),R36(2,1));

%     disp('-------------')
%     rad2deg(q1)
%     rad2deg(q2)
%     rad2deg(q3)
%     rad2deg(q4)
%     rad2deg(q5)
%     rad2deg(q6)

    %Robot.teach([q1 q2 q3 q4 q5 q6]);
end