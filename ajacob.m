function Jac = ajacob(q, pe, Ta)

    % extract each angle from q(:,i)   
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6); q7 = q(7);

    AB1 = [cos(q1) 0 -sin(q1) 0; sin(q1) 0 cos(q1) 0; 0 -1 0 0.34; 0 0 0 1];
    A12 = [cos(q2) 0 sin(q2) 0; sin(q2) 0 -cos(q2) 0; 0 1 0 0; 0 0 0 1];
    AB2 = AB1*A12;
    A23 = [cos(q3) 0 sin(q3) 0; sin(q3) 0 -cos(q3) 0; 0 1 0 0.4; 0 0 0 1];
    AB3 = AB2*A23;
    A34 = [cos(q4) 0 -sin(q4) 0; sin(q4) 0 cos(q4) 0; 0 -1 0 0; 0 0 0 1];
    AB4 = AB3*A34;
    A45 = [cos(q5) 0 -sin(q5) 0; sin(q5) 0 cos(q5) 0; 0 -1 0 0.4; 0 0 0 1];
    AB5 = AB4*A45;
    A56 = [cos(q6) 0 sin(q6) 0; sin(q6) 0 -cos(q6) 0; 0 1 0 0; 0 0 0 1];
    AB6 = AB5*A56;
    z0 = [0 0 1]'; p0 = [0 0 0]';
    z1 = AB1(1:3,3); p1 = AB1(1:3,4);
    z2 = AB2(1:3,3); p2 = AB2(1:3,4);
    z3 = AB3(1:3,3); p3 = AB3(1:3,4);
    z4 = AB4(1:3,3); p4 = AB4(1:3,4);
    z5 = AB5(1:3,3); p5 = AB5(1:3,4);
    z6 = AB6(1:3,3); p6 = AB6(1:3,4);
    
    Jg = [cross(z0,pe-p0) cross(z1,pe-p1) cross(z2,pe-p2) cross(z3,pe-p3) cross(z4,pe-p4) cross(z5,pe-p5) cross(z6,pe-p6);
           z0 z1 z2 z3 z4 z5 z6];
    
    Jac = inv(Ta)*Jg;
end