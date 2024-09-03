syms E miu_s nu r_s h r_c r Fz
% rc = ((4 * Fz * (1 - nu^2) * r_s * h) / (pi * E))^0.25;
p(r) = E / (2 * (1 - nu^2) * r_s * h) * (r_c^2 - r^2);
Fx = int(miu_s * p(r) * 2 * pi * r, r, 0, r_c);