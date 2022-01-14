// Libraries
#include <stdio.h>
#include "ControllerTilt.h" /* 20-sim submodel class include file */

// Main function
int main()
{
	XXDouble u [2 + 1];
	XXDouble y [1 + 1];

	// Initialize the inputs and outputs with correct initial values
	u[0] = 0.0;		/* MeasuredTilt */
	u[1] = 0.0;		/* SetPointTilt */

	y[0] = 0.0;		/* SteeringTilt */


	ControllerTilt my20simSubmodel;

	// Initialize the submodel itself and calculate the outputs for t = 0.0
	my20simSubmodel.Initialize(u, y, 0.0);
		//printf("Time: %f\n", my20simSubmodel.GetTime() );

	// Simple loop: the time is incremented by the integration method
	while (my20simSubmodel.state != ControllerTilt::finished)
	{
		// Call the submodel to calculate the output
		my20simSubmodel.Calculate (u, y);
			//printf("Time: %f\n", my20simSubmodel.GetTime() );
	}

	// Perform the final calculations
	my20simSubmodel.Terminate (u, y);

	// Return
	return 0;
}

