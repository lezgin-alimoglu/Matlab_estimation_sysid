close all
clear all

%% Sistem Parametreleri
A = [4.9 0.4 -3.1;
     3.9 0   3.9;
     0   0   1.1];      
B = [1; 1; 1];
C = [1 1 0];

A_hat = A;
x0 = [10; 10; -10];                     
xhat0 = [0; 0; 0]; 

L_matrix = [-9 -10 -11];

% LQR Kazanç Matrisini Hesapla
Q = eye(3);  % Durum değişkenleri için ağırlık matrisi
R = 1;       % Kontrol girdisi için ağırlık matrisi
K_lqr = lqr(A, B, Q, R);  % LQR Kazancı

% Observer Kazancı Hesapla
L = place(A', C', L_matrix)';  % Observer gain

N = -inv(C * inv(A - B*K_lqr) * B); 

ref = N*10*ones(3,1);

rank(obsv(A,C))
rank(ctrb(A,B))

%% Kalman Filtresi için Parametreler
Q_kf = 0.01 * eye(3);  % Process noise covariance
R_kf = 0.25 * eye(1);  % Measurement noise covariance
P0 = eye(3);  % Başlangıç kovaryansı

%% Augmented Sistem (gerçek sistem + observer)
A_aug = [A-B*K_lqr        B*K_lqr;
         A-A_hat   (A_hat - L*C)];

B_aug = [B*N zeros(3,1);
         zeros(3,1) -L];

sys_aug = ss(A_aug, B_aug, eye(6), zeros(6,2));

%% Simülasyon Parametreleri
t = 0:0.01:20;

sigma_w = 0.2;                         % process noise
w = sigma_w * randn(size(t));           

sigma_v = 0.5;                           
v = sigma_v * randn(size(t));          % measurement noise

r = 10*ones(size(t)) + w;               % step input + process noise

%% Kalman Filtresi Uygulaması ve Simülasyon
x_hat = zeros(length(t), 3);           % Durum tahmini
P_kalman = P0;                          % Covariance matrix başlangıçta birim

for k = 2:length(t)
    % Time Update
    x_pred = A * x_hat(k-1, :)' + B * r(k-1);   
    P_pred = A * P_kalman * A' + Q_kf;  

    % Kalman Kazancı Hesaplama
    K_kalman = P_pred * C' / (C * P_pred * C' + R_kf);

    % Measurement Update
    y_measured = (C * x_pred) + v(k);  
    x_hat(k, :) = (x_pred + K_kalman * (y_measured - C * x_pred))';  
    P_kalman = (eye(3) - K_kalman * C) * P_pred;
end

%% Simülasyon Sonuçları
y_t = lsim(sys_aug, [r' v'], t, [x0; xhat0]); 
x = y_t(:,1:3);      
error = y_t(:,4:6);      
x_hat = x - error;

% Ölçüm (noiseli)
y_measured = (C * x')' + v;                % y = C*x + noise

%% Grafikler
figure;
plot(t, x(:,1),'r', 'LineWidth', 2); hold on;
plot(t, x_hat(:,1),'r--', 'LineWidth', 1.5);
plot(t, x(:,2),'g', 'LineWidth', 2);
plot(t, x_hat(:,2),'g--', 'LineWidth', 1.5);
plot(t, x(:,3),'b', 'LineWidth', 2);
plot(t, x_hat(:,3),'b--', 'LineWidth', 1.5);
legend('x1','x1-hat','x2','x2-hat','x3','x3-hat');
xlabel('Zaman (s)'); ylabel('State');
title('State Feedback + Observer + Kalman Filter + LQR (noisy y)');
grid on;

figure;
plot(t, y_measured,'k');
xlabel('Zaman (s)');
ylabel('y (noisy ölçüm)');
title('Sensör Ölçümü (noisy)');

%% Steady-state error
r0 = 10;    % step referans
x_ss = -(A - B*K_lqr)^(-1) * B * r0;   % steady-state state
y_ss = C * x_ss;                   % steady-state output
e_ss = r0 - y_ss;                  % steady-state error
disp(['Steady-state error = ', num2str(e_ss)]);
