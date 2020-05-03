# Chapter 8

The code for chapter 8 is included in this folder.

## Simulation study

The code for simulation study is in the folder of `simluation_study_code`. It mainly contains the following scripts:

* `Optimization_gait2dp_CL_simulation_2dData_SJC_50HZ.py` generate the simulation data through a trajectory optimization.
* `Optimization_gait2dp_CL_Simulation_Id_2dData_SJC_0.py` identifies joint impedance properties from the simulated data with the same simulation model.
* `Optimization_gait2dp_CL_Simulation_Id_2dData_SJC_SN.py` identifies joint impedance properties from the simulated data with the same simulation model with (pink) sensor noise.
* `Optimization_gait2dp_CL_Simulation_Id_2dData_SJC_PC.py` identifies joint impedance properties from the simulated data with the simulation model that changed -+ 10% mass properties.
* `Optimization_gait2dp_CL_Simulation_Id_2dData_SJC_FZ.py` identifies joint impedance properties from the simulated data with the simulation model that changed -+ 10% foot size.

* `gait2dpi_model_CL_simulation_SelectedJointsControl.py` is the walking simulation model in the direct collocation optimization format. The code is generalized so that different joints can be controlled by impedance controllers and others are controlled by open-loop torques.

* `result_analysis_CL_simulation_SJC_xx.py` analyzes the identified results of each condition.
* `Identified_Impedance_Controller_Comparison_largerRange_OverallComparison.py` generates the overall comparison plots that included in the dissertation.

## Experimental Study

Both code and data are included in the folder of `result_of_pilot_study`. Since the optimization code is almost the same as the optimization code in the simulation study, they were not included. Only the identification results were included.
