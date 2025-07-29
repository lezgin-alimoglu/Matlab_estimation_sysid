close all
clear all
%% Sistem (örnek unstable)
A = [5 0 -3;
     4 0 4;
     0 0 1];
B = [1; 1; 1];
C = [1 1 0];

eig(A)
%% 1️⃣ K ve L tasarımı
L_matrix = [-2 -3 -4];
K_matrix = [-6 -7 -8]; 
K = place(A,B,K_matrix);        % State feedback gain
L = place(A',C',L_matrix)';     % Observer gain


%% 2️⃣ Augmented sistem (x ve xhat birlikte)
N = -inv(C * inv(A - B*K) * B);

A_aug = [A      -B*K;
         L*C        A-L*C-B*K];

B_aug = [B*N;
         N*B];
    
sys_aug = ss(A_aug, B_aug, eye(6), zeros(6,1));

%% 3️⃣ Simülasyon
t = 0:0.01:15;
r = 100*ones(size(t));              % reference
x0 = [10; 10; -10];             % gerçek state başlangıcı
xhat0 = [0; 0; 0];              % observer başlangıcı

y = lsim(sys_aug, r, t, [x0; xhat0]);

x = y(:,1:3);        % gerçek state
xhat = y(:,4:6);     % observer state tahmini

%% 4️⃣ Plot
figure;
plot(t, x(:,1),'r', 'LineWidth', 2); hold on;
plot(t, xhat(:,1),'r--', 'LineWidth', 1.5);
plot(t, x(:,2),'g', 'LineWidth', 2);
plot(t, xhat(:,2),'g--', 'LineWidth', 1.5);
plot(t, x(:,3),'b', 'LineWidth', 2);
plot(t, xhat(:,3),'b--', 'LineWidth', 1.5);

legend('x1','x1-hat','x2','x2-hat','x3','x3-hat');
xlabel('Zaman (s)'); ylabel('State');
title('State Feedback + Observer');
grid on;
%%
figure;
plot(t, C*xhat.');

r0 = 10;  % step referans
x_ss = -(A - B*K)^(-1) * B * r0;   % steady-state state
y_ss = C * x_ss;                   % steady-state output
e_ss = r0 - y_ss;                  % steady-state error

disp(['Steady-state error = ', num2str(e_ss)]);
