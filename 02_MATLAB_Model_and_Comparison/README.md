# 02 — MATLAB Model and XDYN Comparison

Set this folder as the MATLAB **Current Folder** and start with:

```text
RUN_MODEL.m
```

All required MATLAB files and all three XDYN reference CSVs are already in this folder.

## Reference CSVs supplied

| MATLAB case | Reference CSV |
|---|---|
| Propeller + rudder | `mmg_updated.csv` |
| Sail only | `sail_only.csv` |
| Combined propeller + rudder + sail | `combined.csv` |

## Select the model in `RUN_MODEL.m`

```matlab
% Propeller + rudder only
cfg.usePropellerRudder = true;
cfg.useSail            = false;

% Sail only
cfg.usePropellerRudder = false;
cfg.useSail            = true;

% Combined
cfg.usePropellerRudder = true;
cfg.useSail            = true;
```

The main operating inputs are also in `RUN_MODEL.m`: propeller RPM, rudder angle/time, wind speed and wind direction.

## Compare MATLAB with XDYN

Set:

```matlab
cfg.compareToXDYN = true;
cfg.referenceCSV  = '';
```

With `referenceCSV=''`, MATLAB automatically selects:

- `mmg_updated.csv` for propeller/rudder
- `sail_only.csv` for sail only
- `combined.csv` for the combined case

Then run the **whole script**. Comparison mode uses the XDYN time grid and first state, integrates the MATLAB model independently, and compares states, trajectory and actuator loads.

## Code map

| File | Purpose |
|---|---|
| `RUN_MODEL.m` | Select case and edit operating inputs |
| `model_parameters.m` | Fixed model parameters, tables and interpolation data |
| `ship_dynamics.m` | Hull, propeller/rudder and sail dynamics |
| `run_ship_model.m` | RK4 simulation and XDYN CSV loading |
| `compare_ship_results.m` | MATLAB–XDYN errors and metrics |
| `show_results.m` | One tabbed results window with overlays, errors and tables |

If the model inputs are changed, regenerate the corresponding XDYN CSV in folder `01` before using the comparison as validation.
