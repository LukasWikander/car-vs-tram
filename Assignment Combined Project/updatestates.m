function [vupd,Tm,Pb]=updatestates(task,tix,v,Tm)
% State transititon given current vehicle velocity and electric machine
% torque.
%   [vupd,Tm,Pb]=updatestates(task,tix,v,Tm) computes the vehicle speed
%   vupd at the next instant tix+1, for a vector of electric machine
%   torques Tm. Besides the next vehicle speed vupd, the function returns
%   the feasible control actions Tm and the battery power Pb.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2017-05.

V=task.V;                               % vehicle structure
w=v/V.R;                                % EM speed
Tmax=interp1(V.em.wix, V.em.Tmax, w);   % max EM torque
Tmin=interp1(V.em.wix, V.em.Tmin, w);   % min EM torque
Tm(Tm > Tmax | Tm < Tmin) = [];         % remove infeasible control

Fair = V.chs.cd*V.chs.Af*task.env.airdensity/2*v^2;         % aerodynamic drag
Fslope = V.chs.m*task.env.gravity*sin(task.dc.slope(tix));  % force due to road inclination
Froll = V.chs.m*task.env.gravity*V.chs.cr*cos(task.dc.slope(tix)); % rolling resistance
vupd = v + task.ds*(Tm/V.R - Fair - Fslope - Froll)/(V.chs.m*v);   % state update

vupd = min(vupd, task.Vmax/3.6);        % limit to max allowed speed. The exces torque is the mechanical braking torque
ok = vupd >= task.Vmin/3.6;             % feasible transitions 
vupd(~ok)=[]; Tm(~ok) = [];             % remove infeasible transitions

% battery power as output
if any(ok)
    % The battery powers the EM (including the EM losses) and the auxiliary
    % devices
    Pb=Tm*w + interp2(V.em.wix, V.em.Tix, V.em.Ploss, w, Tm) + V.chs.Paux; 
else
    Pb=[];
end
