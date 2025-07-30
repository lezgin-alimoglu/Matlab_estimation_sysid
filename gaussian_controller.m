close all
clear all

%% System
A = [4.9 0.4 -3.1;
     3.9 0   3.9;
     0   0   1.1];      
B = [1; 1; 1];
C = [1 1 0];

A_hat = [5 0 -3;
         4 0 4;
         0 0 1];        

x0 = [10; 10; -10];                     
xhat0 = [0; 0; 0]; 


K_matrix = [-5 -6 -7];     % State feedback için hedef kutuplar
L_matrix = [-9 -10 -11];
         

K = place(A,B,K_matrix);        % State feedback gain
L = place(A',C',L_matrix)';     % Observer gain

N = -inv(C * inv(A - B*K) * B);


ref = N*10*ones(3,1);

%% Augmented sistem (gerçek sistem + observer)

A_aug = [A-B*K        B*K;
         A-A_hat   (A_hat - L*C)];

B_aug = [B*N zeros(3,1);
         zeros(3,1) -L];

sys_aug = ss(A_aug, B_aug, eye(6), zeros(6,2));


%% Simülasyon parametreleri
t = 0:0.01:15;

sigma_w = 0.2;                         % process noise
w = sigma_w * randn(size(t));           

sigma_v = 0.5;                           
v = sigma_v * randn(size(t));          % measurement noise

r = 10*ones(size(t)) + w;               % step + ufak process noise

%% Simülasyon çalıştırma
y_t = lsim(sys_aug, [r' v'], t, [x0; xhat0]); 

x    = y_t(:,1:3);      
error = y_t(:,4:6);      
x_hat = x - error;

%% Ölçüm (noiseli)
    
y_measured = (C*x')' + v;                % y = C*x + noise

%% 8️⃣ Grafikler
figure;
plot(t, x(:,1),'r', 'LineWidth', 2); hold on;
plot(t, x_hat(:,1),'r--', 'LineWidth', 1.5);
plot(t, x(:,2),'g', 'LineWidth', 2);
plot(t, x_hat(:,2),'g--', 'LineWidth', 1.5);
plot(t, x(:,3),'b', 'LineWidth', 2);
plot(t, x_hat(:,3),'b--', 'LineWidth', 1.5);
legend('x1','x1-hat','x2','x2-hat','x3','x3-hat');
xlabel('Zaman (s)'); ylabel('State');
title('State Feedback + Observer (noisy y)');
grid on;

figure;
plot(t, y_measured,'k');
xlabel('Zaman (s)');
ylabel('y (noisy ölçüm)');
title('Sensör Ölçümü (noisy)');

%% Steady-state error
r0 = 10;    % step referans
x_ss = -(A - B*K)^(-1) * B * r0;   % steady-state state
y_ss = C * x_ss;                   % steady-state output
e_ss = r0 - y_ss;                  % steady-state error
disp(['Steady-state error = ', num2str(e_ss)]);