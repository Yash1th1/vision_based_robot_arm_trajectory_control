function Ta= geo2ana(q)
dh_parameters = [0.340, q(1), 0, -pi/2; 
                  0, q(2),0 , pi/2;
                 0.400, q(3), 0, pi/2; 
                  0, q(4), 0, -pi/2; 
                 0.400, q(5), 0, -pi/2; 
                 0, q(6), 0, pi/2; 
                 0.126, q(7), 0, 0];
         T0E = forward_kinematics(dh_parameters);
    % get euler angles from T0E
    R0E = T0E(1:3,1:3);
    eulerzyz = rotm2eul(R0E,'zyz');
    phi = eulerzyz(1);
    theta = eulerzyz(2);
    Ta = [1 0 0 0 0 0; 
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 -sin(phi) cos(phi)*sin(theta);
      0 0 0 0 cos(phi) sin(phi)*sin(theta);
      0 0 0 1 0 cos(theta)];
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
