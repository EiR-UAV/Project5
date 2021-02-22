%% Lateral motion
syms y v phi psi p q r real
syms g Ix Iy Iz l d b real
x = [y v phi psi p r];
f = [v -g*phi p r (Iy-Iz)*q*r/Ix (Ix-Iy)*p*q/Iz]';
g1 = [0 0 0 0 0 -d/(b*Iz)]';
g2 = [0 0 0 0 l/Ix d/(b*Iz)]';
g3 = [0 0 0 0 0 -d/(b*Iz)]'; %Identical to g1, drop it
g4 = [0 0 0 0 -l/Ix d/(b*Iz)]';

r1 = rank([g1 g2 g4]); %only g1,g2 are linearly independent; need 4 Lie Brackets

fg1=myLieBracket(x,f,g1);
fg2=myLieBracket(x,f,g2);

r2 = rank([g1 g2 fg1 fg2]);

ffg1=myLieBracket(x,f,fg1);
ffg2=myLieBracket(x,f,fg2);

fffg1=myLieBracket(x,f,ffg1);
fffg2=myLieBracket(x,f,ffg2);

r3 = rank([g1 g2 fg1 fg2 fffg1 fffg2]); %results in rank 6 (M)
det_final= det([g1,g2,fg1,fg2,fffg1,fffg2]);