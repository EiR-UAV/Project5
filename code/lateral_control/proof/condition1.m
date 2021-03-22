% %% Lateral motion
% %% 1. Linearlized Model
% syms y v phi psi p q r real
% syms g Ix Iy Iz l d b real
% x = [y v phi psi p r];
% f = [v -g*phi p r (Iy-Iz)*q*r/Ix (Ix-Iy)*p*q/Iz]';
% g1 = [0 0 0 0 0 -d/(b*Iz)]';
% g2 = [0 0 0 0 l/Ix d/(b*Iz)]';
% g3 = [0 0 0 0 0 -d/(b*Iz)]'; %Identical to g1, drop it
% g4 = [0 0 0 0 -l/Ix d/(b*Iz)]';
% 
% r1 = rank([g1 g2 g4]); %only g1,g2 are linearly independent; need 4 Lie Brackets
% 
% fg1=myLieBracket(x,f,g1)
% fg2=myLieBracket(x,f,g2)
% 
% r2 = rank([g1 g2 fg1 fg2]);
% 
% ffg1=myLieBracket(x,f,fg1)
% ffg2=myLieBracket(x,f,fg2)
% 
% fffg1=myLieBracket(x,f,ffg1)
% fffg2=myLieBracket(x,f,ffg2)
% 
% r3 = rank([g1 g2 fg1 fg2 fffg1 fffg2]); %results in rank 6 (M)
% det_final= det([g1,g2,fg1,fg2,fffg1,fffg2]);

%% 2. Non- Linearlized Model
% 2A. just y direction
syms y y_dot phi theta psi phi_dot theta_dot psi_dot real
syms Ix Iy Iz l d b m real
s = [y y_dot phi theta psi phi_dot theta_dot psi_dot]; %state vector

f = [
    y_dot;
    0;
    phi_dot;
    theta_dot;
    psi_dot;
    theta_dot*psi_dot*(Iy-Iz)/Ix;
    phi_dot*psi_dot*(Iz-Ix)/Iy;
    phi_dot*theta_dot*(Ix-Iy)/Iz;
];

g1 = [
    0;
    -(sin(psi)*sin(theta)*cos(phi) - sin(phi)*cos(psi))/m;
    0;
    0;
    0;
    0;
    l/Iy;
    -d/(b*Iz)
];

g2 = [
    0;
    -(sin(psi)*sin(theta)*cos(phi) - sin(phi)*cos(psi))/m;
    0;
    0;
    0;
    l/Ix;
    0;
    d/(b*Iz)
];

g3 = [
    0;
    -(sin(psi)*sin(theta)*cos(phi) - sin(phi)*cos(psi))/m;
    0;
    0;
    0;
    0;
    -l/Iy;
    -d/(b*Iz)
];

g4 = [
    0;
    -(sin(psi)*sin(theta)*cos(phi) - sin(phi)*cos(psi))/m;
    0;
    0;
    0;
    -l/Ix;
    0;
    d/(b*Iz)
];

r1 = rank([g1 g2 g3 g4]); %g1,g2, g3, g4 are linearly independent; need 6 more Lie Brackets

fg1=myLieBracket(s,f,g1)
fg2=myLieBracket(s,f,g2)
fg3=myLieBracket(s,f,g3)
fg4=myLieBracket(s,f,g4)
r2 = rank([g1 g2 g3 g4 fg1 fg2 fg3 fg4]); %all independent

det_final= det([g1,g2,g3,g4,fg1,fg2,fg3,fg4]);