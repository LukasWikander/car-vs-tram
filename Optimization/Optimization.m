clc; clear all; close all; 

%% Genetic algorithm

costFun = @costFunction;                    % Cost function of the optimization problem
nVars=2;                                    % Number of decision variables
A = [];                                     % Linear inequalities: A*x <= b
b = [];                                     % Linear inequalities: A*x <= b
Aeq = [];                                   % Linear inequalities: Aeq*x = beq
beq = [];                                   % Linear inequalities: Aeq*x = beq
lb = [];                                    % Lower boundaries: lb <= x <= ub
ub = [];                                    % Upper boundaries: lb <= x <= ub
nonlcon = [];                               % Function nonlcon(x): returns C and Ceq and ga minimizes costFun so that C(x) <= 0 and Ceq(x) = 0                             
IntCon = 1;                                 % Variables which take integer values
options = optimoptions('ga','ConstraintTolerance',1e-6,...
    'MaxGenerations',200*nVars,'PopulationSize',200,'PlotFcn', @gaplotbestf);
[x,fval,exitflag,output] = ga(costFun,nVars,A,b,Aeq,beq,lb,ub,nonlcon,IntCon,options);

%% Fmincon

costFun = @costFunction;                    % Cost function of the optimization problem
x0 = [1 2];                                 % Initial values of decision variables
A = [];                                     % Linear inequalities: A*x <= b
b = [];                                     % Linear inequalities: A*x <= b
Aeq = [];                                   % Linear inequalities: Aeq*x = beq
beq = [];                                   % Linear inequalities: Aeq*x = beq
lb = [];                                    % Lower boundaries: lb <= x <= ub
ub = [];                                    % Upper boundaries: lb <= x <= ub
nonlcon = [];                               % Function nonlcon(x): returns C and Ceq and ga minimizes costFun so that C(x) <= 0 and Ceq(x) = 0                             
options = optimoptions('fmincon','ConstraintTolerance',1e-6,...
    'MaxIterations',1200,'Display','iter','Algorithm','interior-point','PlotFcn', @optimplotfval);
[x,fval,exitflag,output] = fmincon(costFun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);