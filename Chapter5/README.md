# Chapter 5

The code for chapter 5 is included in this folder. Two study directions were included in this study: the simulation data identification and the experimental data identification.

## Simluation Study

* The simulation data was generated using the script of `DoublePendulum_ForwardSimulation.py`. 
* The script of `Pink_noise.py` was used to generate the pink noise. 
* `StandingModel_GoodWorth_Opt2.py` includes the objecitve function, gradient of the objective function, dynamic constraints, and the jacobian matrix of the human simulation model in the direct collocation format.
* `StandingOptimizationIpopt_GoodWorth2_SimulationId.py` is the main script for the motion controller identification on simulated data.

## Experimental Study

* `StandingOptimizationIpopt_xx_Eps_Noise.py` is the main script for the motion controller identification on experimental data of controller type `xx`.
* `StandingModel_FPD_Eps_Noise.py` contains the objecitve function, gradient of the objective function, dynamic constraints, and the jacobian matrix of the human simulation model with the controller type `xx` in the direct collocation format.
* `ResultAnalysis_xx_GenerateBestGainsForEachParticipant.py` analyzes the identified results of each participant, including select the best controller (with the lowest RMS).
* `xx_PaperPlot.py` generates the plot of controller type `xx` for the dissertation.
