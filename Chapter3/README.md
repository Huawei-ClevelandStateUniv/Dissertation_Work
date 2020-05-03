# Chapter 3
The code for chapter 3 is included in this folder.

The `Data_Processing_Main.m` file is the main file for the data processing. 

In general, six steps are included:

1. Fill the gaps in the raw marker data
2. Inertia compensation of the grf in perturbation trials
3. Calculate joint angles and torques based on the gap filled marker data and joint motion and the ground reaction forces.
	 Joints: hip, knee, ankle
   Sign convention for angles and moments: hip flexion, knee extension, ankle Dorsiflexion are positive
4. Calculate model parameters based on segment length, height and weight 
5. Statistical analysis of the calculated joint motion
6. Save processed data into files

processed data will be write to the 'processing_data' folder.

The raw data used by this code is stored in the Zenodo (DOI 10.5281/zenodo.3767611) (Chapter 3).
