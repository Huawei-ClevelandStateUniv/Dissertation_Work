# Chapter 4

Both the code and the data for chapter 4 is included in this folder.

1. Code in the `controller_identification` folder is to identify postural controllers from motion data through a trajectory optimization method. In which,

* `StandingOptimizationIpopt_xx_Eps_Noise025_SameInit.py` is the main code of the trajectory optimization for the `xx` type of controller.

* `StandingModel_xx_Eps_Noise_SameInit.py` contains the direct collocation setup of the objective function, gradient of the objective function, dynamic constraints, and the jacobian matrix.

2. Code in the `stability_check` folder is to check practical stability through forward simulation and eigenvalue tests.

* `ForwardSimulation_2DoF_xx_GeneralCount.py` check the practically stable forward simulations of the controller type `xx`.

* `EigenvalueCheck_PD_Count.py` checks the percentage of all nagative eigenvalues of a meshed poses of the controller type `xx`.

3. Code in the `plot_generation` generates the plots for the Chapter 4 of the dissertation.






