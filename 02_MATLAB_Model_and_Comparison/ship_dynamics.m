function [zdot,loads] = ship_dynamics(t,z,par)
%SHIP_DYNAMICS All model equations in ONE file, with local force functions.
% z=[x;y;psi;u;v;r], SI units. zdot has the same state ordering.
% Optional loads: [Fx,Fy,Mz] about the body origin for PR and sail separately.
% No input CSV states are used here to drive or correct the simulation.
P=par.P; R=par.R; S=par.S;
psi=z(3); u=z(4); v=z(5); r=z(6);

% 1. Hull polynomials + signed calm-water resistance (always active).
[XH,YH,NH,resistance]=hull_forces(z,P,par.Rpp);
pr=zeros(5,1); sail=zeros(5,1); sailOrigin=zeros(1,3);

% 2. Enable or remove each complete actuator group.
if par.usePR
    [Xpr,Ypr,Kpr,Mpr,Npr]=propeller_rudder_loads(t,u,v,R);
    pr=[Xpr;Ypr;Kpr;Mpr;Npr];
end
if par.useSail
    [Xs,Ys,Ns]=sail_force(z,S);
    lever=S.positionBody-[0;0;P.zG];
    mom=cross(lever,[Xs;Ys;0]);
    sail=[Xs;Ys;mom(1);mom(2);Ns+mom(3)];
    originMoment=cross(S.positionBody,[Xs;Ys;0]);
    sailOrigin=[Xs,Ys,Ns+originMoment(3)];
end

% 3. Source force/moment assembly and rigid-body convective terms.
Xtot=-resistance+XH+pr(1)+sail(1); Ytot=YH+pr(2)+sail(2);
Ktot=P.zG*YH+pr(3)+sail(3);
Mtot=P.zG*resistance-P.zG*XH+pr(4)+sail(4);
Ntot=NH+P.xarm*YH+pr(5)+sail(5);
tauE=[Xtot;Ytot;0;Ktot;Mtot;Ntot];
tauC=[P.m*v*r;-P.m*u*r;0;P.J(2,3)*r^2;-P.J(1,3)*r^2;0];
a=P.M6\(tauE+tauC);

% 4. Body velocity -> NED position rates; retain horizontal accelerations.
% This is the ORIGINAL full 6x6 solve with a(1),a(2),a(6) retained, not a newly
% derived constrained 3x3 system. Roll/pitch moments affect the full solve.
zdot=[u*cos(psi)-v*sin(psi); u*sin(psi)+v*cos(psi); r; a(1);a(2);a(6)];
if nargout>1
    loads.propellerRudder=[pr(1),pr(2),pr(5)];
    loads.sail=sailOrigin;
end
end

% -------------------------------------------------------------------------
function [XH,YH,NH,resistance] = hull_forces(z,P,Rpp)
%HULL_FORCES MMG polynomial forces at the hull calculation point.
% z=[x;y;psi;u;v;r]. XH/YH [N], NH [N*m]. Signed resistance is separate.
% The source's local velocity offset and force/moment scaling are unchanged.

u=z(4); v=z(5); r=z(6);
vm=v-P.xGc*r; U=hypot(u,vm);
if U<=sqrt(eps)
    XH=0; YH=0; NH=0;
else
    vp=vm/U; rp=r*P.Lpp/U;
    Xp=P.Xvv*vp^2+P.Xrr*rp^2+P.Xvr*vp*rp+P.Xvvvv*vp^4;
    Yp=P.Yv*vp+P.Yr*rp+P.Yvvv*vp^3+P.Yvrr*vp*rp^2+P.Yrrr*rp^3+P.Yrvv*rp*vp^2;
    Np=P.Nv*vp+P.Nr*rp+P.Nvvv*vp^3+P.Nvrr*vp*rp^2+P.Nrrr*rp^3+P.Nrvv*rp*vp^2;
    sc=0.5*P.rho*U^2*P.Lpp*P.draft;
    XH=sc*Xp; YH=sc*Yp; NH=sc*P.Lpp*Np;
end
resistance=ppval(Rpp,u); % The spline extrapolates outside its tabulated speeds
end

% -------------------------------------------------------------------------
function beta = rudder_command(t,R)
%RUDDER_COMMAND Rudder angle [rad] from editable time/degree arrays.
% Linear interpolation; hold the first/last value outside the time schedule.
% Defaults reproduce the source's 0 -> 10 deg ramp over 299.9..300 s.
tQuery=min(max(t,R.commandTime(1)),R.commandTime(end));
beta=deg2rad(interp1(R.commandTime,R.rudderDeg,tQuery,'linear'));
end

% -------------------------------------------------------------------------
function [X,Y,Kroll,Mpitch,Nyaw] = propeller_rudder_loads(t,u,v,R)
%PROPELLER_RUDDER_LOADS Original propeller/rudder force and CG moments.
% X,Y [N]; Kroll,Mpitch,Nyaw [N*m]. Body CG is [0;0;R.zG].
% With CG x=y=0, yaw moment about the CG equals that about the body origin.

beta=rudder_command(t,R);
[pn,Xprop,Kprop]=propeller_force(u,R);
[Frx,Fry]=rudder_force(u,v,beta,pn,R);

% Combine the forces, then transfer their moments using the source geometry.
X=Xprop+Frx;
Y=Fry;
Kroll=Kprop-(R.zrud-R.zG)*Fry;
Mpitch=(R.zprop-R.zG)*Xprop+(R.zrud-R.zG)*Frx;
Nyaw=R.xrud*Fry;
end

% -------------------------------------------------------------------------
function [wn,Xprop,Kprop] = propeller_force(u,R)
%PROPELLER_FORCE Original polynomial thrust and reaction roll torque.
% Outputs: wn (legacy rudder-input scalar), Xprop [N], Kprop [N*m].
% Coefficients/exponents live in model_parameters.m, not in this equation file.

if R.rpm<=0
    error('MMG:RPM','Active propeller requires R.rpm>0; set usePropellerRudder=false to disable it.');
end

% 1. Advance ratio, clipped exactly as in the uploaded source.
n=R.rpm/60;
J=(1-R.w)*abs(u)/(n*R.Dprop);
J=min(max(J,0),1.5);

% 2. Thrust/torque polynomials evaluated using the separate coefficient tables.
Kt=sum(R.ct.*J.^R.st.*R.PD.^R.tt.*R.AE_A0.^R.ut.*R.Zblades.^R.vt);
Kq=sum(R.cq.*J.^R.sq.*R.PD.^R.tq.*R.AE_A0.^R.uq.*R.Zblades.^R.vq);
tauX=(1-R.tdeduct)*R.rho*n^2*R.Dprop^4*Kt;
tauK=R.etaR*R.rho*n^2*R.Dprop^5*Kq;

% 3. SOURCE CONVENTION - preserved, not physically re-derived:
% wn mixes thrust [N] and roll torque [N*m] through hypot, and is supplied
% to the rudder model as T. It requires independent physical review; do not
% interpret wn as a dimensionally established thrust magnitude.
Kprop=-tauK;
wn=hypot(tauX,Kprop);
Xprop=tauX;
end

% -------------------------------------------------------------------------
function [Fx,Fy] = rudder_force(u,v,beta,T,R)
%RUDDER_FORCE Rudder panels inside/outside the propeller slipstream.
% Fx,Fy are body-axis forces [N]. T is the inherited propeller scalar wn.
% Original low-speed branch and slipstream corrections are retained unchanged.

% 1. Axial approach velocity and original slipstream correction.
Va=(1-R.w)*u;
DVa=R.Dprop*Va;
if abs(DVa)<1e-10
    CTh=8e20/pi*T/R.rho;
else
    CTh=8/pi*T/(R.rho*DVa^2);
end
if CTh>13.71742
    rf=0.5;
else
    rf=1-0.135*sqrt(CTh);
end

dV=sqrt(Va^2+8/pi*T/(R.rho*R.Dprop^2))-Va;
VinX=(Va+R.Kr*dV)*rf;
VinY=v;
VoutX=Va;
VoutY=v;

% 2. Split the rudder area by the contracted slipstream width.
Cj=1+R.Kr*(sqrt(1+CTh)-1);
wd=R.Dprop*sqrt((1+0.5*(sqrt(1+CTh)-1))/Cj);
aIn=min(R.Ar,R.chord*wd);
aOut=R.Ar-aIn;

% 3. Add the body-axis lift/drag components from both panels.
[Fxi,Fyi]=rudder_panel(VinX,VinY,aIn,beta,R);
[Fxo,Fyo]=rudder_panel(VoutX,VoutY,aOut,beta,R);
Fx=Fxi+Fxo;
Fy=Fyi+Fyo;
end

% -------------------------------------------------------------------------
function [Fx,Fy] = rudder_panel(Vx,Vy,area,beta,R)
%RUDDER_PANEL Lift/drag of one panel, resolved into body axes [N].
% beta and alpha are radians. Original lift/friction laws are retained.
% No additional low-Reynolds-number clipping is introduced by the refactor.

if area<=0
    Fx=0; Fy=0; return;
end
speed=hypot(Vx,Vy);
if speed<=sqrt(eps)
    Fx=0; Fy=0; return;
end

% Relative-flow angle and rudder incidence.
fa=atan2(Vy,Vx);
alpha=beta-fa;
Cl=2*pi*R.lambda*(R.lambda+1)/(R.lambda+2)^2*sin(alpha);
lift=0.5*R.rho*area*speed^2*Cl*cos(alpha)*R.liftc;

% Source viscous/induced drag law, with its original low-Re limitations.
Re=speed*R.chord/R.nu;
Cf=0.075/(log10(Re)-2)^2;
Cd=1.1*Cl^2/(pi*R.lambda)+2.5*Cf;
drag=0.5*R.rho*area*speed^2*Cd*R.dragc;

Fx=-lift*sin(fa)-drag*cos(fa);
Fy=lift*cos(fa)-drag*sin(fa);
end

% -------------------------------------------------------------------------
function [Fx,Fy,Mz] = sail_force(z,S)
%SAIL_FORCE Horizontal aerodynamic force [N] at S.positionBody.
% z=[x;y;psi;u;v;r], angles in rad. Wind is an explicit NED velocity vector.
% Mz is INTRINSIC yaw moment (zero in the source), not the lever-arm moment.
% ship_dynamics adds r x F once, including when the sail position is edited.

psi=z(3); u=z(4); v=z(5); r=z(6);

% 1. Sail-point velocity includes yaw-induced motion about the body origin.
omega=[0;0;r];
vO=[u;v;0];
vP=vO-cross(S.positionBody,omega);

% 2. Rotate true wind to body axes and subtract the sail-point velocity.
rot=[cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
aw=rot*S.windNED-vP;
asp=hypot(aw(1),aw(2));
beta=atan2(-aw(2),-aw(1));
alpha=mod(beta,2*pi);

% 3. Mirror the polar angle when symmetry is enabled, then use the splines.
if S.symmetry && alpha>pi
    pa=2*pi-alpha;
else
    pa=alpha;
end
CL=ppval(S.CLpp,pa);
CD=ppval(S.CDpp,pa);
lift=0.5*CL*S.rhoAir*asp^2*S.referenceArea;
drag=0.5*CD*S.rhoAir*asp^2*S.referenceArea;

% 4. Resolve lift/drag according to the source's wind-side convention.
if alpha<=pi
    Fx=-drag*cos(beta)+lift*sin(beta);
    Fy=-drag*sin(beta)-lift*cos(beta);
else
    Fx=-drag*cos(beta)-lift*sin(beta);
    Fy=-drag*sin(beta)+lift*cos(beta);
end
Mz=0;
end
