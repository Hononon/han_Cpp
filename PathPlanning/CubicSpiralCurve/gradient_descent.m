clear; clc;

syms x y;
f(x, y) = 0.5 * x ^ 3 + 2 * y ^ 3 + 1;

step = 50;
tol = 1e-6;
x0 = 0;
y0 = 0.8;
coeffs = [x0; y0];

for iter = 1:step
    % disp(iter);
    f_val = double(f(coeffs(1), coeffs(2)));

    if abs(f_val) < tol
        break;
    end

    grad_sym = gradient(f, [x, y]);
    grad = double(subs(grad_sym,[x,y],[x0,y0]));

    J_sym = jacobian(f, [x, y]);
    % 下降方向
    J = double(subs(J_sym, [x, y], [x0, y0]));

    t = 1;
    alpha = 0.5;
    beta = 0.5;

    while f(x0 + t * J(1), y0 + t * J(2)) > (f(x0, y0) + alpha * t * J * J')
        t = beta * t;
    end

    disp(t);

    coeffs = coeffs - 0.1 * J';
    x0 = coeffs(1);
    y0 = coeffs(2);

end

disp('最终解:');
disp(coeffs);
