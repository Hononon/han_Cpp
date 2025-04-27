clear; clc;

syms x y;
f(x, y) = 0.5*x ^ 2 + 2*y ^ 2;

step = 50;
tol = 1e-6;
x0 = 2;
y0 = 1;
coeffs = [x0;y0];

for iter = 1:step
    disp(iter);
    f_val = double(f(coeffs(1), coeffs(2)));

    if abs(f_val) < tol
        break;
    end
    H_sym = hessian(f, [x, y]);
    H = double(subs(H_sym, [x, y], [x0,y0]));
    J_sym = jacobian(f,[x,y]);
    % 求梯度, 多元标量函数的雅可比矩阵的逆就是梯度
    J = double(subs(J_sym,[x,y],[x0,y0]));
    delta = inv(H)*J';

    coeffs = coeffs - delta;
    x0 = coeffs(1);
    y0 = coeffs(2);

end

disp('最终解:');
disp(coeffs);
