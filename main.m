clc;
clear all;
close all;
qc = [deg2rad(-77.26), deg2rad(-38.76), deg2rad(26.22), deg2rad(93.29), deg2rad(-56.69), deg2rad(-59.94), deg2rad(118)];
dh_params = [0.340, qc(1), 0, -pi/2; 
             0, qc(2),0 , pi/2;
             0.400, qc(3), 0, pi/2; 
             0, qc(4), 0, -pi/2; 
             0.400, qc(5), 0, -pi/2; 
             0, qc(6), 0, pi/2; 
             0.126, qc(7), 0, 0]; 
T = forward_kinematics(dh_params);
%disp(T); 
Tec = [0, 1, 0, 0;
       -1, 0, 0, -0.0662;
       0, 0, 1, 0.0431;
       0, 0, 0, 1];
position = [-0.14195360128424114; -0.06062556383004704; 0.3528046636209403];
Rzyx = eul2rotm([deg2rad(68.70697922863141),deg2rad(-27.847557028831005),deg2rad(-172.95718336855933)],'zyx');
Tcar = [Rzyx, position; 0, 0, 0, 1];
Tart = [1, 0, 0, 0.103975;
       0, 1, 0, -0.103975;
       0, 0, 1, 0;0,0,0,1];
%Transformation matrix of target w.r.t base
T_target = T* Tec* Tcar* Tart;
%disp(T_target);
Te_obj = [0 -1 0 0.0455;
          -1 0 0 0;
          0 0 -1 0.060;%Te_obj matrix represent the transformation by which endeffector must offset for the object to reach target
          0 0 0 1];
T_targeto = T_target*(Te_obj);%Transformation matrix from base to end effector when object sits on the target
R0E_final = T_targeto(1:3,1:3);
angles = rotm2eul(R0E_final,'zyz');
phi_final = angles(1);
theta_final = angles(2);
psi_final = angles(3);
x_pos = T_targeto(1,4);
y_pos = T_targeto(2,4);
z_pos = T_targeto(3,4);
% Desired pose of end effector wrt to base
pd = [x_pos; y_pos; z_pos];
phid = [phi_final; theta_final; psi_final];
% Inverse kinematics
q1 = [58.2686 75.3224 11.7968 45.9029 -22.1081 -31.2831 -42.3712]; 
q1=deg2rad(q1);
steps = 1000000;
q = zeros(7, steps); 
e = zeros(6, steps);
J = zeros(6,7);
% Ta(phi) matrix to go from geometric to analytic Jacobian
q(:,1) = q1';
K = 10; 
for i=1:steps
    xe = fkin(q(:,i));
    pe = xe(1:3,1);
    Ta = geo2ana(q(:,i));
    J = ajacob(q(:,i), pd, Ta);
    e(:,i) = [pd; phid]-xe;
    qdot = pinv(J)*(K*e(:,i));
    q(:,i+1) = q(:,i) + qdot*0.0001;
    if (max(abs(e(1:3,i))) < 0.000001&&max(abs(e(4:6,i))) < 0.0001) 
        break;
    end 
end
z=q(:,i);
zd=rad2deg(z);
for y=1:7
    if zd(y)>(180)          %To remove the complete rotations
       while  zd(y)>(180)
           zd(y)=zd(y)-180;
       end
    elseif zd(y)<-180
        while zd(y)<(-180)
            zd(y)=zd(y)+180;
        end
    end
end
fprintf("Computed configuration:\n");
disp(zd);
zr=deg2rad(zd);
ZZ = zr';
dh_parameters = [0.340, ZZ(1), 0, -pi/2; 
             0, ZZ(2),0 , pi/2;
             0.400, ZZ(3), 0, pi/2; 
             0, ZZ(4), 0, -pi/2; 
             0.400, ZZ(5), 0, -pi/2; 
             0, ZZ(6), 0, pi/2; 
             0.126, ZZ(7), 0, 0]; 
Tv = forward_kinematics(dh_parameters);
fprintf("Final Transformation Matrix for Computed configuration:\n");
disp(Tv);
% Trajectory planning
fileID = fopen('Tekumudi_Yashwanth.txt','w');
fmt = '%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n';
tt=10;
q1=q1';
a0=q1;
a1=0;
a3=(q1-zr)/500;
a2=-15*a3;
for t=0:0.005:tt
    qt=(a3)*(t^3)+(a2)*(t^2)+(a1)*(t)+(a0);
    qt=qt';
    fprintf(fileID,fmt,qt);
end
%Txt file of velocities to make sure the are meeting their constraints
fileID2 = fopen('Tekumudi_Yashwanth_Velocity.txt','w');
fmt = '%.4f %.4f %.4f %.4f %.4f %.4f %.4f\n';
for t=0:0.005:tt
    qv=3*(a3)*(t^2)+2*(a2)*(t)+(a1);
    qv=qv';
    fprintf(fileID2,fmt,qv);
end
limits_verify;
function T = forward_kinematics(dh_params)
T1 = DH_transform(dh_params(1,1), dh_params(1,2), dh_params(1,3), dh_params(1,4));
T2 = DH_transform(dh_params(2,1), dh_params(2,2), dh_params(2,3), dh_params(2,4));
T3 = DH_transform(dh_params(3,1), dh_params(3,2), dh_params(3,3), dh_params(3,4));
T4 = DH_transform(dh_params(4,1), dh_params(4,2), dh_params(4,3), dh_params(4,4));
T5 = DH_transform(dh_params(5,1), dh_params(5,2), dh_params(5,3), dh_params(5,4));
T6 = DH_transform(dh_params(6,1), dh_params(6,2), dh_params(6,3), dh_params(6,4));
T7 = DH_transform(dh_params(7,1), dh_params(7,2), dh_params(7,3), dh_params(7,4));
%fprintf("Transformation matrix of the end effector\n");
T = T1 * T2 * T3 * T4 * T5 * T6 * T7;
end
function T = DH_transform(d, theta, a, alpha)
T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
     sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0, sin(alpha), cos(alpha), d;
     0, 0, 0, 1];
end
