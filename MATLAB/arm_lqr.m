
function fx = arm_lqr(x)
  Q = diag([1, 1, 1, 1, 1, 1]);
  R = 1;

  [A, B, C, D] = ss_model(x(1), x(2), x(3), x(4), x(5), x(6))
  system = ss(A, B, C, D);
  K = lqr(system, Q, R);

  x_des = [0, 0, 0, 0, 0, 0];
  fx = K * (x - x_des);
end