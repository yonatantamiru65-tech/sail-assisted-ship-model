# Sail-Assisted Ship Model — MATLAB/XDYN Handover

This package contains the **XDYN reference cases** and the **independent MATLAB model used for MATLAB–XDYN comparison**.

## Folder structure

| Folder | Contents | Start here |
|---|---|---|
| `01_XDYN_Reference_Generation` | Three XDYN YAML cases: propeller + rudder, sail only, and combined | `README.md` / `run_both_yml.m` |
| `02_MATLAB_Model_and_Comparison` | MATLAB model, comparison functions, plots, and the three matching XDYN CSV files | `RUN_MODEL.m` |

## Available cases

| Case | XDYN YAML | XDYN CSV | MATLAB selection |
|---|---|---|---|
| Propeller + rudder | `mmg_prop_rudder.yml` | `mmg_updated.csv` | `usePropellerRudder=true`, `useSail=false` |
| Sail only | `sail_only.yml` | `sail_only.csv` | `usePropellerRudder=false`, `useSail=true` |
| Combined | `mmg_combined_sail.yml` | `combined.csv` | `usePropellerRudder=true`, `useSail=true` |

## Recommended procedure

1. **To inspect or regenerate XDYN reference data:** open `01_XDYN_Reference_Generation`.
2. **To run the MATLAB model and compare it with XDYN:** open `02_MATLAB_Model_and_Comparison`, set it as the MATLAB Current Folder, and open `RUN_MODEL.m`.
3. Select the required actuator case in section 1 of `RUN_MODEL.m`.
4. For comparison, set:

```matlab
cfg.compareToXDYN = true;
cfg.referenceCSV  = '';   % automatically selects the matching CSV
```

5. Run the **whole** `RUN_MODEL.m`. The results window overlays MATLAB and XDYN states/trajectory/loads and provides error metrics.

The supplied CSV files correspond to the supplied XDYN YAML cases. If operating inputs are changed, regenerate the matching XDYN CSV before interpreting the comparison.

## Project report

The accompanying PDF documents the nonlinear modelling and validation work. Numerical validation results must be interpreted using the operating inputs and actuator configuration documented in that report.

## Requirements and attribution

MATLAB is required for the independent model and comparisons. Regenerating the reference data also requires a separately installed XDYN executable and its dependencies. XDYN is an external reference simulator; this repository does not claim authorship of XDYN or redistribute its executable.

Engineering project by Yonatan Tamiru Asfaw. Existing files and attribution are retained. No new software license has been assigned, and the models have not been rerun during repository preparation.
