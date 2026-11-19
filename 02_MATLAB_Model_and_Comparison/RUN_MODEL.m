%% RUN_MODEL - Open this file and press Run.
% Keep the SIX .m files in the same folder, separate from the older package.
% This is an independent MATLAB simulation. It does NOT run XDYN or YAML.

%% 1. SELECT ACTUATORS AND ENTER OPERATING INPUTS
cfg = struct();
cfg.usePropellerRudder = true;
cfg.useSail            = true;
% Propeller + rudder only: true,  false
% Sail only:              false, true
% Combined:               true,  true
% Hull resistance and MMG hull forces remain active in all three cases.

% PROPELLER + RUDDER INPUTS (ignored when usePropellerRudder=false)
cfg.rpm                = 110;   % Shaft speed [rpm]; positive values only
cfg.rudderAngleDeg     = 10;    % Rudder command after the change [deg]
cfg.rudderChangeTime   = 300;   % [s]; use 0 for a CONSTANT angle from the start
cfg.rudderRampTime     = 0.1;   % [s]; positive ramp duration, ending at changeTime
% Defaults: 0 deg until 299.9 s, then a linear ramp to 10 deg at 300 s.
% Example constant rudder: rudderAngleDeg=5 and rudderChangeTime=0.
% For a straight command, use rudderAngleDeg=0. Do not use rpm=0 to disable PR;
% use the actuator switch instead (zero/reverse rpm is not modelled here).

% SAIL INPUTS (ignored when useSail=false)
cfg.windSpeed          = 10;    % Horizontal true-wind speed [m/s]
cfg.windDirectionDeg   = 90;    % Direction wind travels TOWARD: 0=North, 90=East
% This MATLAB input is NOT the meteorological direction the wind comes FROM.

%% 2. TIME, INITIAL STATE AND OPTIONAL XDYN COMPARISON
cfg.dt           = 0.1;         % RK4 time step [s]
cfg.tStart       = 0;           % [s]
cfg.tEnd         = 2500;        % [s]
cfg.initialState = [0;0;0;7;0;0]; % [x;y;psi;u;v;r], m, rad, m/s, rad/s

cfg.compareToXDYN = false;      % false: no CSV needed; true: compare to XDYN
cfg.referenceCSV  = '';         % Empty: choose matching filename automatically
% PR only -> mmg_updated.csv | sail only -> sail_only.csv | combined -> combined.csv
% Put a MATCHING reference CSV here; changing MATLAB inputs does not change it.
% In comparison mode, CSV times/first state REPLACE dt/tStart/tEnd/initialState.
cfg.csvAngleUnit   = 'rad';     % Explicit assumption: 'rad' or 'deg' for psi
cfg.csvYawRateUnit = 'rad/s';   % 'rad/s' or 'deg/s' for r
cfg.unwrapHeading = true;      % Assumes <180 deg heading change per CSV interval

cfg.dataDir   = fileparts(mfilename('fullpath'));
cfg.outputDir = '';            % Empty -> results/<case>_<simulation/comparison>
addpath(cfg.dataDir);          % Use the six files from this folder

%% 3. BUILD PARAMETERS AND RUN (no plots inside the simulation)
% Everyday inputs are above. Hull/geometry/tables/splines: model_parameters.m.
par = model_parameters(cfg);
results = run_ship_model(cfg,par);

%% 4. RESULTS ONLY - rerun this section without repeating the simulation
makePlots   = true;
saveResults = true;            % CSV/MAT results + PNG/SVG/FIG plots
figures = show_results(results,makePlots,saveResults);
