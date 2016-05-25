%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matlab M-file                Author: Yuchun Li
%
% simulation of event based observer with 2 bidirectional connected agents
% the plant to be simulated is a 3d plant 
% note that when timer10 and timer20 have same initial conditions, 
% it reduces to the scenario of synchronuous event times 
%
% Name: run.m
%
% Description: run script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% global data -----------
clear all
global G A H1 H2 K11 K12 K21 K22 gamma T1 T2

% plant information 
A  = [0 1 0;-1 0 0;0 0 0];
H1 = [1 1 0];               % measurement at agent 1
H2 = [0 0 1];               % measurement at agent 2

%%% parameters 
T1 = 0.2; T2 = 0.4;         % lower and upper bound for the event times
K11 = [-0.5 -0.2 -0.1]';    % gain at agent 1
K12 = [-0.2 -0.2 -0.5]';    % gain at agent 1
K21 = [0.2 0.3 0.3]';       % gain at agent 2
K22 = [-0.1 -0.5 0.2]';     % gain at agent 2
gamma = -0.4;               % gain for consensus

%%%% Graph (1) - 2 agents
G = ones(2,2);

%%% -----------------------
% IC for plant states
xp0 = [1 1 1]';

% IC for agent1;
xo10 = [1 0 6]';
eta10 = [1 1 1]';
timer10 = 0.2;

% IC for agent2;
xo20 = [-1 0 3.5]';
eta20 = [-1 -1 -1]';
timer20 = 0.2;

y0 = [xp0; xo10; xo20; eta10; eta20; timer10; timer20]; 

% simulation horizon
TSPAN = [0 30];
JSPAN = [0 20000];

% rule for jumps
% rule = 1 -> priority for jumps
% rule = 2 -> priority for flows
rule = 1;

options = odeset('RelTol',1e-1,'MaxStep',1e-2);

% simulate
[t y j] = hybridsolver(@f,@g,@C,@D,y0,TSPAN,JSPAN,rule,options,1);

%% - phase plot in 3d space
figure
rate = 1;
plot3(y(1:rate:end,1),y(1:rate:end,2),y(1:rate:end,3),'r','linewidth',2)
hold on 
plot3(y(1:rate:end,4),y(1:rate:end,5),y(1:rate:end,6),'k--','linewidth',1)
plot3(y(1:rate:end,7),y(1:rate:end,8),y(1:rate:end,9),'b-.','linewidth',1)
grid on 
set(gca,'FontSize',20)
legend('x0', 'x1', 'x2')
% axis equal
%%


%% 
figure
subplot(3,1,1)      % trajectories for the third component
plot(t, y(:,3),'r','linewidth',2)
hold on
plot(t, y(:,6),'k--','linewidth',1.5)
plot(t, y(:,9),'b-.','linewidth',1.5)
grid on 
set(gca,'FontSize',20)
legend('x0', 'x1', 'x2')
axis([0 30 -3, 7])
xlabel('t')
subplot(3,1,2)      % third component of memory state
plotHarcT(t, j, y(:,12), 'k--')
hold on 
plotHarcT(t, j, y(:,15), 'b-.')
grid on 
set(gca,'FontSize',20)
legend('e1', 'e2')
axis([0 5 -3, 2])
subplot(3,1,3)      % timer 
plotHarcT(t, j, y(:,17), 'b')
grid on 
set(gca,'FontSize',20)
legend('tau')
axis([0 5 0, 0.5])
%%