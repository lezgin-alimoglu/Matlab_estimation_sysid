close all
clear all
%% Sistem tanımı (model mismatch senaryosu)
A = [4.7 0.4 -3.1;
     3.9 0   3.8;
     0   0   1.1];      % Gerçek sistem
B = [1; 1; 1];
C = [1 1 0];

A_hat = [5 0 -3;
         4 0 4;
         0 0 1];        % Tasarım modeli

K_matrix = [-2 -1-j -1+j];     % State feedback için hedef kutuplar
L_matrix = [-3 -2 -1];         % Observer için hedef kutuplar

K = place(A_hat,B,K_matrix);        % State feedback gain
L = place(A_hat',C',L_matrix)';     % Observer gain

N = -inv(C * inv(A_hat - B*K) * B);

%% Simülasyon parametreleri
t = 0:0.01:15;
sigma_w = 0.2;                          % (opsiyonel) process noise
w = sigma_w * randn(size(t));           % şimdilik referansa ekleniyor

r = 10*ones(size(t)) + w;               % step + ufak process noise
x0 = [10; 10; -10];                     % gerçek state başlangıcı
xhat0 = [0; 0; 0];                      % observer başlangıcı


%% 4️⃣ Augmented sistem (gerçek sistem + observer)
A_real = [A - B*K        B*K];
A_est = [0*ones(size(A,1),size(A,2))   (A_hat - L*C)];

B_aug = [B*N];
        
sys_real = ss(A_real, B_aug, eye(3), zeros(3,1));
sys_est =  ss(A_est, B_aug, eye(3), zeros(3,1));

%% 6️⃣ Simülasyon çalıştırma
y = lsim(sys_real, r, t, [x0; xhat0]); 
y_m = y + w;

% Write a new system such that x_dot_hat = Ax_hat - BKx_hat + L(y_m - Cx_hat)
error = y;
xhat = x - error ;      % observer tahmini

%% 7️⃣ Ölçüm (noiseli)
sigma_v = 0.5;                           % ölçüm noise std
v = sigma_v * randn(length(t),1);        % measurement noise
y_measured = (C*x')' + v;                % y = C*x + noise

%% 8️⃣ Grafikler
figure;
plot(t, x(:,1),'r', 'LineWidth', 2); hold on;
plot(t, xhat(:,1),'r--', 'LineWidth', 1.5);
plot(t, x(:,2),'g', 'LineWidth', 2);
plot(t, xhat(:,2),'g--', 'LineWidth', 1.5);
plot(t, x(:,3),'b', 'LineWidth', 2);
plot(t, xhat(:,3),'b--', 'LineWidth', 1.5);
legend('x1','x1-hat','x2','x2-hat','x3','x3-hat');
xlabel('Zaman (s)'); ylabel('State');
title('State Feedback + Observer (noisy y)');
grid on;

figure;
plot(t, y_measured,'k');
xlabel('Zaman (s)');
ylabel('y (noisy ölçüm)');
title('Sensör Ölçümü (noisy)');

%% 9️⃣ Steady-state error
r0 = 10;  % step referans
x_ss = -(A - B*K)^(-1) * B * r0;   % steady-state state
y_ss = C * x_ss;                   % steady-state output
e_ss = r0 - y_ss;                  % steady-state error
disp(['Steady-state error = ', num2str(e_ss)]);
    