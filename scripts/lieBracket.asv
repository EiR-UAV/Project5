function res = lieBracket(g1,g2,x)
    g1_dot = jacobian(g1, x);
    g2_dot = jacobian(g2, x);
    
    res = g2_dot * g1 - g1_dot * g2;
end
