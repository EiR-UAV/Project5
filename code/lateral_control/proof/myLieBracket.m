function V=myLieBracket(Q,f,g)

for i=1:length(f)
    for j=1:length(f)
        J_f(i,j)=diff(f(i),Q(j));
        J_g(i,j)=diff(g(i),Q(j));
    end
end
V = J_g*f-J_f*g;