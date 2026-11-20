# Model inputs, references and figure provenance

[Project overview](../README.md) | [Figure gallery](RESULTS.md) | [Full thesis](../Nonlinear_Modelling_and_Validation_3DOF_Sail_Assisted_Ship%20(1).pdf)

## Numerical inputs versus method references

The immediate source of the implemented vessel parameters, rigid-body data, resistance samples, force-model arrays and sail polar is the **XDYN/WISAMO configuration and C++ implementation** used in the project. The literature below explains the underlying methods; it should not be read as an independent measurement source for every value. This distinction follows thesis Table A.1 (p. 31).

| Component | Input provenance in this project | Method/reference | Thesis location |
|---|---|---|---|
| Rigid-body dynamics and inertia | XDYN vessel configuration and rigid-body data | Fossen (2011); XDYN | Chapter 2; Appendix A.2; Tables A.3-A.4 |
| MMG hull loads | XDYN hydrodynamic coefficients | Yasukawa and Yoshimura (2015) | Section 3.1; Table A.5 |
| Calm-water resistance | XDYN tabulated speed/resistance samples | Cubic-spline interpolation in MATLAB | Appendix A.4; Table A.6; Fig. A.1 |
| Propeller thrust and torque | XDYN configuration and regression arrays | Oosterveld and van Oossanen (1975) | Appendix A.5 and A.9; Fig. A.2 |
| Rudder/slipstream interaction | XDYN configuration and C++ force model | XDYN; MMG rudder modelling | Section 3.2.1; Appendix A.6; Fig. 3.1 |
| Apparent wind and wing-sail forces | XDYN/WISAMO sail geometry and aerodynamic polar | Charlou et al. (2023); XDYN | Section 3.2.2; Appendix A.7; Tables A.9-A.10 |
| Integration and comparisons | MATLAB implementation and matching XDYN CSV exports | Fourth-order Runge-Kutta; metrics defined in the thesis | Chapter 4; Appendices A.8 and B |

## Core references cited by the thesis

1. **Fossen, T. I. (2011).** *Handbook of Marine Craft Hydrodynamics and Motion Control.* Wiley. [DOI: 10.1002/9781119994138](https://doi.org/10.1002/9781119994138).
2. **Yasukawa, H. and Yoshimura, Y. (2015).** Introduction of MMG standard method for ship maneuvering predictions. *Journal of Marine Science and Technology*, 20, 37-52. [DOI: 10.1007/s00773-014-0293-y](https://doi.org/10.1007/s00773-014-0293-y).
3. **Oosterveld, M. W. C. and van Oossanen, P. (1975).** Further computer-analyzed data of the Wageningen B-screw series. *International Shipbuilding Progress*, 22(251), 251-262. [DOI: 10.3233/ISP-1975-2225102](https://doi.org/10.3233/ISP-1975-2225102).
4. **Charlou, M., Wackers, J., Guillerm, P.-E. and Alessandrini, B. (2023).** A new validated open-source numerical tool for the evaluation of the performance of wind-assisted ship propulsion systems. *Mechanics & Industry*, 24, 30. [DOI: 10.1051/meca/2023027](https://doi.org/10.1051/meca/2023027).
5. **SIREHNA (Naval Group).** *XDYN: A lightweight ship simulator.* [Official repository](https://github.com/sirehna/xdyn). The thesis cites the software as 2020; the executable is not distributed in this repository.

These are the modelling references cited in the supplied thesis, not newly claimed sources for the project. The thesis bibliography contains the wider background and control literature.

## Figure provenance

All published PNGs were copied unchanged from the supplied `some result for 3DOF ship` folder. Descriptive filenames replace the local filenames below. Captions and result values were matched to the existing project thesis. Figure/page references use printed thesis pages.

| Published figure (`results/figures/`) | Original supplied filename | Thesis cross-reference |
|---|---|---|
| `kvlcc2-hull.png` | `hull (1).png` | Fig. 1.1, p. 3 |
| `propeller-rudder-interaction.png` | `rudder (1).png` | Fig. 3.1, p. 15; credited to SIREHNA / XDYN |
| `true-and-apparent-wind.png` | `true_vs_apparent_wind (1).png` | Fig. 3.2(a), p. 17 |
| `sail-lift-and-drag.png` | `sail_lift_and_drag (1).png` | Fig. 3.2(b), p. 17 |
| `propeller-rudder-states.png` | `full MATLAB vs XDYN.png` | Fig. 4.1, p. 21 |
| `propeller-rudder-trajectory.png` | `Trajectory.png` | Fig. 4.2, p. 23 |
| `sail-load-comparison.png` | `MATLAB_vs_XDYN.png` | Fig. 4.3, p. 24 |
| `sail-only-states.png` | `6_states_MATLAB_vs_XDYN.png` | Fig. 4.4, p. 25 |
| `sail-only-trajectory.png` | `Sail_trajectory-top_view (1).png` | Fig. 4.5, p. 26 |
| `calm-water-resistance.png` | `resistance_curve (1).png` | Fig. A.1, p. 36 |
| `wageningen-propeller-curves.png` | `wageningen_curves (1).png` | Fig. A.2, p. 37 |
| `sail-aerodynamic-polar.png` | `sail_polar (1).png` | Fig. A.3, p. 42 |

`full MATLAB vs XDYN (1).png` and `sail_polar (2).png` are byte-identical duplicates of the selected copies and were omitted.

The hull and wind/sail explanatory illustrations are reproduced as supplied and used in the thesis. Where that thesis does not record a separate original image creator, this guide does not assign one. Method references do not by themselves establish image authorship. Existing source attribution is retained; no blanket licence is applied to third-party figures or XDYN material.
