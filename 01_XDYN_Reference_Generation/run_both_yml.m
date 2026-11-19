%% run_both_yml_named.m
% Simple MATLAB script to run the two XDYN YML models.
%
% YML 1:
%   Model name : mmg_prop_rudder
%   YML file   : mmg_prop_rudder.yml
%   CSV output : mmg_updated.csv
%
% YML 2:
%   Model name : sail_only
%   YML file   : sail_only.yml
%   CSV output : sail_only.csv
%
% HOW TO USE:
% Put this MATLAB file in the same folder as the two YML files.
% Then press Run in MATLAB.
%
% To run both:
%   runMMG  = true;
%   runSail = true;
%
% To run only mmg_prop_rudder:
%   runMMG  = true;
%   runSail = false;
%
% To run only sail_only:
%   runMMG  = false;
%   runSail = true;

clear; clc;

%% USER SETTINGS
xdynExe = 'xdyn';   % Replace with full path to xdyn.exe if needed

% YML 1: mmg_prop_rudder
mmgYML = 'mmg_prop_rudder.yml';
mmgCSV = 'mmg_updated.csv';

% YML 2: sail_only
sailYML = 'sail_only.yml';
sailCSV = 'sail_only.csv';

% Choose which model(s) to run
runMMG  = true;
runSail = true;

% Simulation settings
dt     = 0.1;
tStart = 0;
tEnd   = 2500;
solver = 'rk4';

%% RUN YML 1: mmg_prop_rudder
if runMMG
    fprintf('\nRunning YML 1: mmg_prop_rudder\n');

    cmd = sprintf('"%s" "%s" --solver %s --dt %.6g --tstart %.6g --tend %.6g', ...
        xdynExe, mmgYML, solver, dt, tStart, tEnd);

    status = system(cmd);

    if status == 0
        fprintf('Finished: mmg_prop_rudder\n');
        fprintf('CSV saved as: %s\n', mmgCSV);
    else
        fprintf('ERROR running mmg_prop_rudder.yml\n');
    end
end

%% RUN YML 2: sail_only
if runSail
    fprintf('\nRunning YML 2: sail_only\n');

    cmd = sprintf('"%s" "%s" --solver %s --dt %.6g --tstart %.6g --tend %.6g', ...
        xdynExe, sailYML, solver, dt, tStart, tEnd);

    status = system(cmd);

    if status == 0
        fprintf('Finished: sail_only\n');
        fprintf('CSV saved as: %s\n', sailCSV);
    else
        fprintf('ERROR running sail_only.yml\n');
    end
end

fprintf('\nDone.\n');
