function task = dynprog_settings(task, vehicle_params, general_params, dc)
%DYNPROG_SETTINGS Summary of this function goes here
%   Detailed explanation goes here
V = struct;

datapath='probdata/';  
load([datapath,'chassis/chs_truck'],'chs');           % chassis      
load([datapath,'em/em_155kW_3000rpm'],'em');         % electric machine

%% Cost function
task.accpenalty=vehicle_params.accpenalty;              % acceleration penalty
task.traveltimepenalty=vehicle_params.traveltimepenalty;   % [MW] penalty for travel time
task.energypenalty=vehicle_params.energypenalty;

%% Constraints
% Assumptions made here:
%	Maximum time taken for round trip including stops is 1h
%	Time constraint is to be split equally between AtoB and BtoA
task.tmax = (60*60 - 2*vehicle_params.t_unload)/2;

%% Drive cycle
task.N=numel(dc.s);                             % number of samples
task.dc=dc;
task.ds=dc.s(2)-dc.s(1); 

%% Environment
task.env.airdensity = general_params.rho_air;
task.env.gravity = general_params.acc_gravity;

%% Electric machine
% scale the electric machine
s=task.Pemmax/em.Pmax; % scaling factor

V.em.Pmax=task.Pemmax;
V.em.Tix=em.Tix*s;
V.em.wix = em.wix*s;
V.em.Tmax=em.Tmax*s;    
V.em.Tmin=em.Tmin*s;
V.em.Ploss=em.Ploss*s;  

%% Chassis
V.chs.m=task.mass;
V.chs.cd=task.cd;
V.chs.Af=task.Af;
V.chs.cr=task.cr;
V.chs.Rw=task.Rw;

% Just took these parameters from the given file...
V.chs.finalgear = chs.finalgear;
V.chs.Paux = chs.Paux;

V.R=V.chs.Rw/V.chs.finalgear/task.gearratio; % wheel radius divided by gearing

%% Save vehicle struct
task.V = V;
end

