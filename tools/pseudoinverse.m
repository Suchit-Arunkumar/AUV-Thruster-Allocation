clc; clear;

% ---- Step 1: Forward matrix B (6x8), tau = B * T ----
B = [     0         0         0         0    0.7070   -0.7070    0.7070    0.7070;
         0         0         0         0   -0.7070    0.7070    0.7070   -0.7070;
    1.0000    1.0000    1.0000    1.0000         0         0         0         0;
    0.1700   -0.1200    0.1200   -0.1200   -0.0140    0.0140   -0.0140    0.0140;
   -0.1200   -0.1200    0.1500    0.1500   -0.0140   -0.0140   -0.0140   -0.0140;
         0         0         0         0    0.0346    0.0346   -0.3250    0.3250];

% ---- Step 2: SVD decomposition ----
[U, S, V] = svd(B);
disp('rank(B):'); disp(rank(B));
disp('Singular values of B:'); disp(svd(B));

% ---- Step 3: Sigma+ ----
[m, n] = size(B);      % m=6, n=8
S_pinv = zeros(n, m);  % 8x6
tol = 1e-10;
for i = 1:min(m,n)
    if S(i,i) > tol
        S_pinv(i,i) = 1 / S(i,i);
    end
end

% ---- Step 4: B+ (8x6), T = B+ * U ----
B_pinv = V * S_pinv * U';

% ---- Step 5: Display and verify ----
disp('B_pinv:'); disp(B_pinv);
disp('Check B*B+*B ~ B:');   disp(norm(B * B_pinv * B - B));
disp('Check B+*B*B+ ~ B+:'); disp(norm(B_pinv * B * B_pinv - B_pinv));
