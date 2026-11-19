# 01 — XDYN Reference Generation

This folder contains the three XDYN input cases used as reference data for the MATLAB validation.

## Files

| YAML file | Case | Output CSV |
|---|---|---|
| `mmg_prop_rudder.yml` | MMG hull + propeller + rudder | `mmg_updated.csv` |
| `sail_only.yml` | MMG hull + aerodynamic sail | `sail_only.csv` |
| `mmg_combined_sail.yml` | MMG hull + propeller + rudder + sail | `combined.csv` |
| `run_both_yml.m` | Simple launcher for the two individual YAML cases | — |

## Run the individual cases

Open `run_both_yml.m`, set the XDYN executable path if required, and choose:

```matlab
runMMG  = true;   % propeller + rudder
runSail = true;   % sail only
```

Set one switch to `false` to run only the other case.

## Run the combined case

The current simple launcher is for the two individual cases. Run the combined YAML directly with XDYN:

```text
xdyn mmg_combined_sail.yml --solver rk4 --dt 0.1 --tstart 0 --tend 2500
```

This writes:

```text
combined.csv
```

### Main supplied operating conditions

- Initial surge speed: `7 m/s`
- Propeller: `110 rpm`, `P/D = 0.79`
- Rudder: `0°` then `10°` at approximately `300 s`
- Sail wind: `10 m/s`, direction `90°`
- Simulation: `0–2500 s`, RK4, `dt = 0.1 s`

The three reference CSV files are already supplied in folder `02_MATLAB_Model_and_Comparison` for direct comparison. If a YAML input is modified, regenerate its CSV so MATLAB and XDYN use the same conditions.
