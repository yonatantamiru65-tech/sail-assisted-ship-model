function par = model_parameters(cfg)
%MODEL_PARAMETERS All hull data, actuator geometry, tables and interpolation.
% Called once per run, AFTER reading cfg in RUN_MODEL.m.
% P=hull/inertia, R=propeller/rudder, S=sail. Units are SI unless labelled.
% Fixed values and default equations are from combined_model_compare(1).m.

validateattributes(cfg.usePropellerRudder,{'logical','numeric'},{'scalar','binary'});
validateattributes(cfg.useSail,{'logical','numeric'},{'scalar','binary'});
if ~cfg.usePropellerRudder && ~cfg.useSail
    error('MMG:Mode','Enable propeller/rudder, sail, or both.');
end
validateattributes(cfg.rpm,{'numeric'},{'scalar','real','finite'});
if cfg.usePropellerRudder && cfg.rpm<=0
    error('MMG:RPM','Active propeller requires positive rpm; disable it with the switch.');
end
validateattributes(cfg.rudderAngleDeg,{'numeric'},{'scalar','real','finite'});
validateattributes(cfg.rudderChangeTime,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(cfg.rudderRampTime,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(cfg.windSpeed,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(cfg.windDirectionDeg,{'numeric'},{'scalar','real','finite'});
validateattributes(cfg.tEnd,{'numeric'},{'scalar','real','finite'});

% HULL / RIGID BODY - default values from the uploaded combined model.
P.rho = 1025;              % Water density [kg/m^3]
P.nu = 1.18e-6;            % Water kinematic viscosity [m^2/s]
P.Lpp = 320;               % Length between perpendiculars [m]
P.draft = 20.8;            % Draft [m]
P.m = 3.101981e8;          % Mass [kg]
P.J = [2.087872e11,1.201552e9,2.187367e10; ...
       1.201552e9,2.421491e12,5.394442e8; ...
       2.187367e10,5.394442e8,2.464725e12]; % CG inertia tensor [kg*m^2]
P.zG = 2.2;                % CG z-coordinate; body +z points downward
% IMPORTANT: xGc is the source's legacy HULL velocity offset, not a newly
% inferred CG x-coordinate. Preserve both offsets unless re-deriving the model.
P.xGc = 11.1;              % v at hull evaluation point = v - xGc*r
P.xarm = -11.1;            % Hull yaw-moment arm [m]

% Non-dimensional MMG hull coefficients (same ordering/values as the source).
P.Xvv=-0.04; P.Xrr=0.011; P.Xvr=0.002; P.Xvvvv=0.771;
P.Yv=-0.315; P.Yr=0.083; P.Yvvv=-1.607;
P.Yrvv=0.379; P.Yvrr=-0.391; P.Yrrr=0.008;
P.Nv=-0.137; P.Nr=-0.049; P.Nvvv=-0.03;
P.Nrvv=-0.294; P.Nvrr=0.055; P.Nrrr=-0.013;
% No added-mass matrix is included by this MATLAB source. Zero added mass
% does NOT establish that added-mass effects are contained in these polynomials.

% PROPELLER / RUDDER - fixed rpm and pitch ratio; time-varying rudder angle.
R.Dprop=9.326; R.Ar=136.7; R.b=15.8; % Diameter, rudder area/height [m,m^2,m]
R.asp=1.7; R.liftc=2.1; R.dragc=1.0; % Aspect, lift and drag factors
R.w=0.576; R.tdeduct=0.7; R.etaR=1.0;
R.Zblades=5; R.AE_A0=0.431; R.PD=0.79; R.rpm=cfg.rpm;
R.xprop=-163.82; R.xrud=-170.0;       % Longitudinal positions [m]
R.zprop=15.0; R.zrud=10.0;           % Vertical positions [m]
% Rudder command built from the simple settings in RUN_MODEL.m.
% The positive-duration ramp ends at rudderChangeTime; outside it values hold.
if cfg.rudderChangeTime==0
    R.commandTime=[0 max(1,cfg.tEnd)];
    R.rudderDeg=[cfg.rudderAngleDeg cfg.rudderAngleDeg];
else
    if cfg.rudderRampTime>cfg.rudderChangeTime
        error('MMG:Rudder','rudderRampTime must not exceed rudderChangeTime.');
    end
    R.commandTime=[0 cfg.rudderChangeTime-cfg.rudderRampTime ...
                  cfg.rudderChangeTime max(cfg.tEnd,cfg.rudderChangeTime+1)];
    R.rudderDeg=[0 0 cfg.rudderAngleDeg cfg.rudderAngleDeg];
    [R.commandTime,keep]=unique(R.commandTime,'stable');
    R.rudderDeg=R.rudderDeg(keep); % Remove a repeated zero when the ramp starts at 0
end
% ADVANCED: replace R.commandTime and R.rudderDeg here for a custom schedule.
% Use increasing times [s] and an equally long list of rudder angles [deg].

% SAIL - one horizontal aerodynamic force, no intrinsic yaw moment.
S.rhoAir=1.225; S.referenceArea=1000;
S.positionBody=[0;0;-21];           % Force application point [m] in body axes
S.symmetry=true;                   % Fold polar angles above pi
% Convert the user wind speed/direction into the original NED velocity vector.
S.windNED=cfg.windSpeed*[cosd(cfg.windDirectionDeg);sind(cfg.windDirectionDeg);0];
% Relative air motion includes vessel velocity, even for zero true wind.


% PROPELLER POLYNOMIAL TABLES - coefficients and exponents in source order.
R.ct=[0.00880496 -0.204554 0.166351 0.158114 -0.147581 -0.481497 0.415437 ...
    0.0144043 -0.0530054 0.0143481 0.0606826 -0.0125894 0.0109689 -0.133698 ...
    0.00638407 -0.00132718 0.168496 -0.0507214 0.0854559 -0.0504475 0.010465 ...
    -0.00648272 -0.00841728 0.0168424 -0.00102296 -0.0317791 0.018604 -0.00410798 ...
    -0.000606848 -0.0049819 0.0025983 -0.000560528 -0.00163652 -0.000328787 ...
    0.000116502 0.000690904 0.00421749 0.0000565229 -0.00146564];
R.st=[0 1 0 0 2 1 0 0 2 0 1 0 1 0 0 2 3 0 2 3 1 2 0 1 3 0 1 0 0 1 2 3 1 1 2 0 0 3 0];
R.tt=[0 0 1 2 0 1 2 0 0 1 1 0 0 3 6 6 0 0 0 0 6 6 3 3 3 3 0 2 0 0 0 0 2 6 6 0 3 6 3];
R.ut=[0 0 0 0 1 1 1 0 0 0 0 1 1 0 0 0 1 2 2 2 2 2 0 0 0 1 2 2 0 0 0 0 0 0 0 1 1 1 2];
R.vt=[0 0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 2 2 2 2 2 2 2 2 2 2 2];
R.cq=[0.00379368 0.00886523 -0.032241 0.00344778 -0.0408811 -0.108009 -0.0885381 ...
    0.188561 -0.00370871 0.00513696 0.0209449 0.00474319 -0.00723408 0.00438388 ...
    -0.0269403 0.0558082 0.0161886 0.00318086 0.015896 0.0471729 0.0196283 -0.0502782 ...
    -0.030055 0.0417122 -0.0397722 -0.00350024 -0.0106854 0.00110903 -0.000313912 ...
    0.0035985 -0.00142121 -0.00383637 0.0126803 -0.00318278 0.00334268 -0.00183491 ...
    0.000112451 -0.0000297228 0.000269551 0.00083265 0.00155334 0.000302683 -0.0001843 ...
    -0.000425399 0.0000869243 -0.0004659 0.0000554194];
R.sq=[0 2 1 0 0 1 2 0 1 0 1 2 2 1 0 3 0 1 0 1 3 0 3 2 0 0 3 3 0 3 0 1 0 2 0 1 3 3 1 2 0 0 0 0 3 0 1];
R.tq=[0 0 1 2 1 1 1 2 0 1 1 1 0 1 2 0 3 3 0 0 0 1 1 2 3 6 0 3 6 0 6 0 2 3 6 1 2 6 0 0 2 6 0 3 3 6 6];
R.uq=[0 0 0 0 1 1 1 1 0 0 0 0 1 1 1 1 1 1 2 2 2 2 2 2 2 2 0 0 0 1 1 2 2 2 2 0 0 0 1 1 1 1 2 2 2 2 2];
R.vq=[0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 2 2 2 2 2 2 2 2 2 2 2 2];


% Sail polar. All three arrays must have equal length and increasing angles.
polarAngleDeg = [0 7 9 12 28 60 90 120 150 180];
S.CL = [0 0.94828 1.13793 1.25 1.42681 1.38319 1.26724 0.93103 0.38793 -0.11207];
S.CD = [0.03448 0.01724 0.01466 0.01466 0.02586 0.11302 0.3825 0.96888 1.31578 1.34483];

% Signed resistance: negative at negative speed, positive at positive speed.
% ship_dynamics subtracts the interpolated resistance in the surge equation.
resistanceSpeedKnots = [-5 0 5 10 12 14 16 18 20];
resistanceForceN = [-2.1134e5 0 2.1134e5 7.8256e5 1.1067e6 1.4945e6 1.9784e6 2.6336e6 3.3000e6];

% DERIVED QUANTITIES - rebuilt each run; normally no edits needed below.
validateattributes(P.m,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(P.J,{'numeric'},{'size',[3 3],'real','finite'});
if norm(P.J-P.J.','fro')>1e-12*norm(P.J,'fro')
    error('MMG:Inertia','P.J must be symmetric.');
end
[~,flag]=chol(P.J);
if flag~=0, error('MMG:Inertia','P.J must be positive definite.'); end
validateattributes([P.rho P.nu P.Lpp P.draft],{'numeric'},{'real','finite','positive'});
P.M6=blkdiag(P.m*eye(3),P.J); % Retain full 6x6 solve, including cross-inertia

validateattributes([R.Dprop R.Ar R.b R.asp R.PD R.AE_A0 R.Zblades], ...
    {'numeric'},{'real','finite','positive'});
validateattributes([P.zG P.xGc P.xarm R.xprop R.xrud R.zprop R.zrud], ...
    {'numeric'},{'real','finite'});
validateattributes(R.w,{'numeric'},{'scalar','real','finite','>=',0,'<',1});
validateattributes(R.tdeduct,{'numeric'},{'scalar','real','finite','>=',0,'<=',1});
validateattributes([R.liftc R.dragc R.etaR],{'numeric'},{'real','finite','nonnegative'});
R.rho=P.rho; R.nu=P.nu; R.zG=P.zG;
R.chord=R.Ar/R.b; R.lambda=R.asp*R.b^2/R.Ar;
dst=abs(R.xprop-R.xrud);
if dst==0, R.Kr=0.5; else, R.Kr=0.5+0.5/(1+0.15/abs(dst/R.Dprop)); end
R.commandTime=R.commandTime(:).'; R.rudderDeg=R.rudderDeg(:).';
check_table(R.commandTime,R.rudderDeg,'Rudder command');

% Check that coefficient and exponent arrays stay aligned after edits.
fields={'ct','st','tt','ut','vt','cq','sq','tq','uq','vq'};
for k=1:numel(fields)
    value=R.(fields{k});
    validateattributes(value,{'numeric'},{'vector','real','finite'});
    R.(fields{k})=value(:).';
end
nT=[numel(R.ct) numel(R.st) numel(R.tt) numel(R.ut) numel(R.vt)];
nQ=[numel(R.cq) numel(R.sq) numel(R.tq) numel(R.uq) numel(R.vq)];
if any(nT~=nT(1)) || any(nQ~=nQ(1)) || nT(1)==0 || nQ(1)==0
    error('MMG:Table','Propeller coefficient/exponent lengths must agree.');
end
validateattributes(S.referenceArea,{'numeric'},{'scalar','real','finite','nonnegative'});
validateattributes(S.rhoAir,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(S.positionBody,{'numeric'},{'vector','numel',3,'real','finite'});
S.positionBody=S.positionBody(:);
S.CL=S.CL(:).'; S.CD=S.CD(:).'; polarAngleDeg=polarAngleDeg(:).';
check_table(polarAngleDeg,S.CL,'Sail lift polar');
check_table(polarAngleDeg,S.CD,'Sail drag polar');
S.polarAngle=deg2rad(polarAngleDeg);
S.CLpp=spline(S.polarAngle,S.CL); S.CDpp=spline(S.polarAngle,S.CD);
check_table(resistanceSpeedKnots,resistanceForceN,'Resistance curve');
Rpp=interp1(resistanceSpeedKnots*(1852/3600),resistanceForceN,'spline','pp');
% Cubic spline interpolation AND extrapolation are retained from the source.
% Switching to linear/pchip changes the model; do not do it just for formatting.

par.P=P; par.R=R; par.S=S; par.Rpp=Rpp;
par.usePR=logical(cfg.usePropellerRudder); par.useSail=logical(cfg.useSail);
par.resistanceSpeedKnots=resistanceSpeedKnots;
par.resistanceForceN=resistanceForceN;
end

function check_table(x,y,label)
% Only a local table check; no additional file is required.
if numel(x)<2 || numel(x)~=numel(y) || ~isvector(x) || ~isvector(y) || ...
        ~isreal(x) || ~isreal(y) || any(~isfinite(x)) || any(~isfinite(y)) || any(diff(x)<=0)
    error('MMG:Table','%s: use finite equal-length lists with increasing knots.',label);
end
end
