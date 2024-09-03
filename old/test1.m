syms u v
eqns = [2*u + v == 0; u - v == 1];
S = solve(eqns,[u v]);