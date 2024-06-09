function limits_verify
load Tekumudi_Yashwanth.txt
q1 = Tekumudi_Yashwanth(:,1);
q2 = Tekumudi_Yashwanth(:,2);
q3 = Tekumudi_Yashwanth(:,3);
q4 = Tekumudi_Yashwanth(:,4);
q5 = Tekumudi_Yashwanth(:,5);
q6 = Tekumudi_Yashwanth(:,6);
q7 = Tekumudi_Yashwanth(:,7);
if (max(abs(q1)>deg2rad(170))||min(abs(q1)<(-170))||max(abs(q2)>deg2rad(120))||min(abs(q2)<(-120))||max(abs(q3)>deg2rad(170))||min(abs(q3)<(-170))||max(abs(q4)>deg2rad(120))||min(abs(q4)<(-120))||max(abs(q5)>deg2rad(170))||min(abs(q5)<(-170))||max(abs(q6)>deg2rad(120))||min(abs(q6)<(-120))||max(abs(q7)>deg2rad(175))||min(abs(q7)<(-175)))
    fprintf("Joint limits reached");
else
    fprintf("Joint limits are safe");
end

load Tekumudi_Yashwanth_Velocity.txt
qv1 = Tekumudi_Yashwanth_Velocity(:,1);
qv2 = Tekumudi_Yashwanth_Velocity(:,2);
qv3 = Tekumudi_Yashwanth_Velocity(:,3);
qv4 = Tekumudi_Yashwanth_Velocity(:,4);
qv5 = Tekumudi_Yashwanth_Velocity(:,5);
qv6 = Tekumudi_Yashwanth_Velocity(:,6);
qv7 = Tekumudi_Yashwanth_Velocity(:,7);
if (max(abs(qv1))>deg2rad(98)||max(abs(qv2))>deg2rad(98)||max(abs(qv3))>deg2rad(100)||max(abs(qv4))>deg2rad(130)||max(abs(qv5))>deg2rad(140)||max(abs(qv6))>deg2rad(180)||max(abs(qv7))>deg2rad(180))
    fprintf("\nvelocity limits reached");
    disp(max(abs(qv1)));
    disp(max(abs(qv1)));
else
    fprintf("\nvelocity limits are safe\n");
end
ZZ = Tekumudi_Yashwanth(2000,:);
dh_parameters = [0.340, ZZ(1), 0, -pi/2;
             0, ZZ(2),0 , pi/2;
             0.400, ZZ(3), 0, pi/2; 
             0, ZZ(4), 0, -pi/2; 
             0.400, ZZ(5), 0, -pi/2; 
             0, ZZ(6), 0, pi/2; 
             0.126, ZZ(7), 0, 0]; 
Tverifyat2000 = forward_kinematics(dh_parameters);
fprintf("Final Transformation Matrix after Trajectory Planning:\n");
disp(Tverifyat2000);
fprintf("Configuration Achieved after Trajectory Planning:\n");
disp(rad2deg((Tekumudi_Yashwanth(2000,:))'))
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
end