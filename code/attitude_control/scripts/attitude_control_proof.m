syms Ix Iy Iz phi_dot psi_dot theta_dot l b d phi theta psi

x = [phi; theta; psi; phi_dot; theta_dot; psi_dot];

f = [
    phi_dot;
    theta_dot;
    psi_dot;
    theta_dot*psi_dot*(Iy-Iz)/Ix;
    psi_dot*phi_dot*(Iz-Ix)/Iy;
    phi_dot*theta_dot*(Ix-Iy)/Iz
];

g1 = [0 0 0 0 l/Iy -d/(b*Iz)].';
g2 = [0 0 0 l/Ix 0 d/(b*Iz)].';
g3 = [0 0 0 0 -l/Iy -d/(b*Iz)].';
g4 = [0 0 0 -l/Ix 0 d/(b*Iz)].';

l1 = lieBracket(f, g1, x);
l2 = lieBracket(f, g2, x);
l3 = lieBracket(f, g3, x);
l4 = lieBracket(f, g4, x);

nabla = [g1 g2 g3 l1 l2 l3]
rank(nabla)
simplify(det(nabla))

latex(simplify(l1))
latex(simplify(l2))
latex(simplify(l3))
latex(simplify(l4))