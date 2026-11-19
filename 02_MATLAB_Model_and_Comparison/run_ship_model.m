function results = run_ship_model(cfg,par)
%RUN_SHIP_MODEL Choose time grid, run RK4, then optionally compare with XDYN.
% RUN_MODEL.m supplies cfg and par. CSV parsing and RK4 are local below.
% Only matching reference data should be used after changing operating inputs.

usePR=par.usePR; useSail=par.useSail;
if usePR && useSail
    cfg.mode='combined'; defaultCSV='combined.csv';
    caseTitle='MMG - propeller + rudder + sail';
elseif usePR
    cfg.mode='propeller_rudder'; defaultCSV='mmg_updated.csv';
    caseTitle='MMG - propeller + rudder';
elseif useSail
    cfg.mode='sail_only'; defaultCSV='sail_only.csv';
    caseTitle='MMG - sail only';
else
    error('MMG:Mode','Enable propeller/rudder, sail, or both.');
end
validateattributes(cfg.compareToXDYN,{'logical','numeric'},{'scalar','binary'});
D=[];
if cfg.compareToXDYN
    if isempty(cfg.referenceCSV)
        cfg.referenceCSV=fullfile(cfg.dataDir,defaultCSV);
    elseif ~isfile(cfg.referenceCSV)
        cfg.referenceCSV=fullfile(cfg.dataDir,cfg.referenceCSV);
    end
    D=read_xdyn_reference(cfg.referenceCSV,cfg,usePR,useSail);
    t=D.t; z=D.states(1,:).'; runType='comparison';
    % Source comparison strategy: use CSV times and initial state only.
    % All remaining MATLAB states are integrated independently by RK4.
else
    validateattributes(cfg.dt,{'numeric'},{'scalar','real','finite','positive'});
    validateattributes(cfg.tStart,{'numeric'},{'scalar','real','finite'});
    validateattributes(cfg.tEnd,{'numeric'},{'scalar','real','finite','>',cfg.tStart});
    validateattributes(cfg.initialState,{'numeric'},{'vector','numel',6,'real','finite'});
    t=(cfg.tStart:cfg.dt:cfg.tEnd).';
    if numel(t)==1 || cfg.tEnd-t(end)>10*eps(max(1,abs(cfg.tEnd)))
        t(end+1,1)=cfg.tEnd; % Shorter final step when duration is not a dt multiple
    else
        t(end)=cfg.tEnd;
    end
    z=cfg.initialState(:); runType='simulation';
end
if isempty(cfg.outputDir)
    cfg.outputDir=fullfile(cfg.dataDir,'results',[cfg.mode '_' runType]);
end
n=numel(t);
if n<2 || any(~isfinite(t)) || any(diff(t)<=0)
    error('MMG:Time','Need at least two finite, strictly increasing time samples.');
end
fprintf('\n%s | %d samples | %.6g .. %.6g s\n',caseTitle,n,t(1),t(end));

% Independent integration. Disabled actuators are zero in ship_dynamics.
M.states=zeros(n,6); M.states(1,:)=z.';
for k=1:n-1
    z=rk4_step(t(k),z,t(k+1)-t(k),par);
    if ~isreal(z) || any(~isfinite(z))
        error('MMG:Integration','Invalid state at t=%.9g s; review inputs/time step.',t(k+1));
    end
    M.states(k+1,:)=z.';
end
M.propellerRudder=zeros(n,3); M.sail=zeros(n,3);
for k=1:n
    [~,loads]=ship_dynamics(t(k),M.states(k,:).',par);
    M.propellerRudder(k,:)=loads.propellerRudder;
    M.sail(k,:)=loads.sail;
end
allLoads=[M.propellerRudder M.sail];
if ~isreal(allLoads) || any(~isfinite(allLoads(:)))
    error('MMG:Loads','Invalid actuator load; review parameter validity.');
end
results.title=caseTitle; results.settings=cfg; results.parameters=par;
results.active=struct('propellerRudder',usePR,'sail',useSail);
results.t=t; results.model=M; results.reference=D; results.comparison=[];
if cfg.compareToXDYN
    results.comparison=compare_ship_results(results);
end
end

function next = rk4_step(t,z,h,par)
% Classical fourth-order Runge-Kutta; h is the current time interval.
k1=ship_dynamics(t,z,par);
k2=ship_dynamics(t+h/2,z+h*k1/2,par);
k3=ship_dynamics(t+h/2,z+h*k2/2,par);
k4=ship_dynamics(t+h,z+h*k3,par);
next=z+h*(k1+2*k2+2*k3+k4)/6;
end

function D = read_xdyn_reference(csvFile,cfg,usePR,useSail)
%READ_XDYN_REFERENCE Read named columns using the uploaded model's conventions.
% Standard comma/semicolon CSVs are read by readtable. Headers containing
% commas must be quoted when comma is the delimiter (normal CSV convention).
% Required: six states and the THREE load columns for each ACTIVE actuator.
% Inactive load columns may be absent; absence is NaN, not a measured zero.
% No time resampling, missing-row deletion or unit guessing is performed.

if ~isfile(csvFile)
    error('MMG:MissingCSV','CSV not found: %s. Use compareToXDYN=false to simulate without it.',csvFile);
end
T=readtable(csvFile,'VariableNamingRule','preserve');
D.t=get_column(T,'t',true);
stateNames={'x(ship)','y(ship)','psi(ship)','u(ship)','v(ship)','r(ship)'};
D.states=zeros(height(T),6);
for j=1:6, D.states(:,j)=get_column(T,stateNames{j},true); end
D.propellerRudder=get_loads(T,'propellerAndRudder',usePR);
D.sail=get_loads(T,'sail_1',useSail);
if numel(D.t)<2 || any(~isfinite(D.t)) || any(diff(D.t)<=0)
    error('MMG:Time','CSV times must contain at least two finite, increasing samples.');
end
if any(~isfinite(D.states(:)))
    error('MMG:CSVData','A required state column contains a missing/nonfinite value.');
end
switch lower(char(cfg.csvAngleUnit))
    case 'rad'
    case 'deg', D.states(:,3)=deg2rad(D.states(:,3));
    otherwise, error('MMG:Units','csvAngleUnit must be rad or deg.');
end
switch lower(char(cfg.csvYawRateUnit))
    case 'rad/s'
    case 'deg/s', D.states(:,6)=deg2rad(D.states(:,6));
    otherwise, error('MMG:Units','csvYawRateUnit must be rad/s or deg/s.');
end
if cfg.unwrapHeading
    D.states(:,3)=unwrap(D.states(:,3));
end
% Avoid comparing a disabled actuator with a reference in which it was active.
if ~usePR && any(abs(D.propellerRudder(:))>1e-8)
    error('MMG:ReferenceMode','CSV contains nonzero propeller/rudder loads, but that actuator is disabled.');
end
if ~useSail && any(abs(D.sail(:))>1e-8)
    error('MMG:ReferenceMode','CSV contains nonzero sail loads, but the sail is disabled.');
end
% These optional columns, when present, must agree with the horizontal model.
locked={'w(ship)','p(ship)','q(ship)','phi(ship)','theta(ship)'};
for j=1:numel(locked)
    value=get_column(T,locked{j},false);
    if any(abs(value)>1e-8)
        error('MMG:ReferenceDOF','CSV column %s is nonzero; this MATLAB model is horizontal only.',locked{j});
    end
end
D.csvFile=csvFile;
end

function values=get_loads(T,actuator,required)
components={'Fx','Fy','Mz'}; values=NaN(height(T),3);
for j=1:3
    key=sprintf('%s(%s,ship,ship)',components{j},actuator);
    values(:,j)=get_column(T,key,required);
end
end

function x=get_column(T,wanted,required)
% Ignore punctuation/case only; reject ambiguous matches rather than guessing.
names=T.Properties.VariableNames;
key=lower(regexprep(wanted,'[^a-zA-Z0-9]',''));
keys=cellfun(@(s) lower(regexprep(s,'[^a-zA-Z0-9]','')),names,'UniformOutput',false);
idx=find(strcmp(keys,key));
if isempty(idx)
    if ~required, x=NaN(height(T),1); return; end
    error('MMG:CSVColumn','Required column "%s" not found. Available: %s',wanted,strjoin(names,', '));
end
if numel(idx)~=1
    error('MMG:CSVColumn','Ambiguous CSV columns for "%s".',wanted);
end
x=T{:,idx};
if ~isnumeric(x) || ~isreal(x) || size(x,2)~=1 || any(~isfinite(x(:)))
    error('MMG:CSVData','Column "%s" must contain one finite numeric value per row.',wanted);
end
x=x(:);
end
