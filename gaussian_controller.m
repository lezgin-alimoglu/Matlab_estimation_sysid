%% UNSTABLE SYSTEM
A = [-4, 0, -3;
     4, 0, 4;
     2, 0, 1];
B = [1; 1; 1];
C = [1 1 0];
D = 0;

%% FEEDBACK VE OBSERVER TASARIMI
K = place(A,B,[-4 -5 -6]);       % state feedback
L = place(A',C',[-10 -11 -12])';    % observer

%% DOĞRU AUGMENTED MATRİS
A_aug = [A-B*K   B*K;
         L*C     A-L*C-B*K];
B_aug = [B;
         B];

sys_aug = ss(A_aug, B_aug, eye(6), zeros(6,1));

%% SIMULATION
t = 0:0.01:5;
u = ones(size(t));              % step input
x0 = [10; 10; -10];             % plant başlangıcı
xhat0 = [0; 0; 0];              % observer başlangıcı

y = lsim(sys_aug, u, t, [x0; xhat0]);

x = y(:,1:3);        % gerçek state'ler
xhat = y(:,4:6);     % observer state tahminleri

%% PLOT
figure;
plot(t,x(:,1),'r','LineWidth',2); hold on;
plot(t,xhat(:,1),'r--','LineWidth',1.5);
plot(t,x(:,2),'g','LineWidth',2);
plot(t,xhat(:,2),'g--','LineWidth',1.5);
plot(t,x(:,3),'b','LineWidth',2);
plot(t,xhat(:,3),'b--','LineWidth',1.5);

legend('x1','x1-hat','x2','x2-hat','x3','x3-hat');
xlabel('Zaman (s)'); ylabel('State');
title('Doğru Kurulmuş State Feedback + Observer');
grid on;
