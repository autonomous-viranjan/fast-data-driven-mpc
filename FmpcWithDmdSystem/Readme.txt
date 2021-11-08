###################################################
MultipleTrafficLights code - For the 2018 research Computationally-Efficient Fuel-Economic High-Level Controller Design for a Group of Connected Vehicles in Urban Roads
DSCC2018
Author of the code: Alejandro Fernandez Canosa
Research Supervisor: Baisravan HomChaudhuri
####################################################

To run the code: go to:
	- TwoCarsScenario1
	- MultipleCarsScenario2
Depending on the Scenario you want to simulate


###############################################
1) MPC-based controller:
	- Main Program: ClosedLoopOptimalMultipleTrafficLights.m 
	- Cost Function to minimize: CostFunction.m
	- Constraints: constraint.m
the main program calls the fmincon.m function to minimize the CostFunction.m in an MPC-based approach subject to the constraints specified in constraint.m
################################################


################################################
2) RSBC:
	- Main Program: ClosedLoopRandomMultipleTrafficLights.m
	- 1 Iteration: RandomMethodFunction.m
The main program calls the RandomMethodFunction.m to calculate the trajectory by using a random sampling-based approach
#################################################


#################################################
3) GSBC:
	- Main Program: ClosedLoopGaussianMultipleTrafficLights.m
	- 1 Iteratiom: GaussianMethodFunction.
The main program calls the GaussianMethodFunction.m to calculate the trajectory by using a Gaussian sampling-based approach
#################################################

#################################################
4) MPC-GSBC as initial solution:
	- Main Program: ClosedLoopOptimalGaussianInitialPointMultipleTrafficLights.m
	- How to run it:
		1) Run ClosedLoopGaussianMultipleTrafficLights.m to generate initial solutions and call 
		variables manually:
			sHostGaussianIn = sHostTotal
			vHostGaussianIn = vHostTotal
			uHostGaussianIn = uHostTotal
		2) Run ClosedLoopOptimalGaussianInitialPointMultipleTrafficLights.m
		"Note that you need to run a simulation with one traffic light less in this case because
		the initial solution for the last traffic light must consider the future solution"
##################################################

FuelEfficiency.m calculates the fuel efficiency of the car and Figures.m makes different plots. If desired, the GSBC solution can be used to initialize the MPC-based controller by invoking the ClosedLoopOptimalGaussianInitialPointMultipleTrafficLights.m script