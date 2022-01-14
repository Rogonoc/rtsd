/**********************************************************
 * This file is generated by the 20-sim C++ Code Generator
 *
 *  file:  ControllerPan.cpp
 *  subm:  ControllerPan
 *  model: ControllerPan
 *  expmt: Jiwy-with-controller
 *  date:  January 14, 2022
 *  time:  9:32:51 AM
 *  user:  20-sim 4.8 Campus License
 *  from:  Universiteit Twente
 *  build: 4.8.3.10415
 **********************************************************/

/* Standard include files */
#include <stdio.h>
#include <math.h>
/* Include the header for memcpy and memset
 * You may need to change this into <memory.h> for older compilers
 */
#include <string.h>

/* 20-sim include files */
#include "ControllerPan.h"

/* Delta margin used for end time checking */
const XXDouble c_delta = 1.0e-7;

/* this PRIVATE function sets the input variables from the input vector */
void ControllerPan::CopyInputsToVariables (XXDouble *u)
{
	/* copy the input vector to the input variables */
	m_V[5] = u[0];		/* MeasuredPan */
	m_V[7] = u[1];		/* SetPointPan */

}

/* this PRIVATE function uses the output variables to fill the output vector */
void ControllerPan::CopyVariablesToOutputs (XXDouble *y)
{
	/* copy the output variables to the output vector */
	y[0] = 	m_V[6];		/* SteeringPan */

}

ControllerPan::ControllerPan(void)
{
	m_number_constants = 0;
	m_number_parameters = 4;
	m_number_initialvalues = 2;
	m_number_variables = 8;
	m_number_states = 2;
	m_number_rates = 2;
	m_number_matrices = 0;
	m_number_unnamed = 0;

	/* the variable arrays */
	m_C = new XXDouble[0 + 1];		/* constants */
	m_P = new XXDouble[4 + 1];		/* parameters */
	m_I = new XXDouble[2 + 1];		/* initial values */
	m_V = new XXDouble[8 + 1];		/* variables */
	m_s = new XXDouble[2 + 1];		/* states */
	m_R = new XXDouble[2 + 1];		/* rates (or new states) */
	m_M = new XXMatrix[0 + 1];		/* matrices */
	m_U = new XXDouble[0 + 1];		/* unnamed */
	m_workarray = new XXDouble[0 + 1];

	Reset(0.0);
	m_finish_time = 8.0;
}

void ControllerPan::Reset(XXDouble starttime)
{
	m_start_time = starttime;
	m_step_size = 0.001;
	m_time = starttime;
	m_major = true;
	m_stop_run = false;

	/* Clear the allocated variable memory */
	memset(m_C, 0, (0 + 1) * sizeof(XXDouble));
	memset(m_P, 0, (4 + 1) * sizeof(XXDouble));
	memset(m_I, 0, (2 + 1) * sizeof(XXDouble));
	memset(m_V, 0, (8 + 1) * sizeof(XXDouble));
	memset(m_s, 0, (2 + 1) * sizeof(XXDouble));
	memset(m_R, 0, (2 + 1) * sizeof(XXDouble));
	memset(m_M, 0, (0 + 1) * sizeof(XXDouble));
	memset(m_U, 0, (0 + 1) * sizeof(XXDouble));
	memset(m_workarray, 0, (0 + 1) * sizeof(XXDouble));


	state = initialrun;
}

bool ControllerPan::IsFinished(void)
{
	return (state == finished);
}

ControllerPan::~ControllerPan(void)
{
	/* free memory */
	delete[] m_C;
	delete[] m_P;
	delete[] m_I;
	delete[] m_V;
	delete[] m_s;
	delete[] m_R;
	delete[] m_M;
	delete[] m_U;
	delete[] m_workarray;
}

/* the initialization function for submodel */
void ControllerPan::Initialize (XXDouble *u, XXDouble *y, XXDouble t)
{
	/* initialization phase (allocating memory) */
	m_initialize = true;
	m_stop_run = false;

	/* set the constants */


	/* set the parameters */
	m_P[0] = 0.3;		/* PID_Pan\kp {} */
	m_P[1] = 0.1;		/* PID_Pan\tauD {s} */
	m_P[2] = 0.1;		/* PID_Pan\beta {} */
	m_P[3] = 1000.0;		/* PID_Pan\tauI {s} */


	/* set the initial values */
	m_I[0] = 0.0;		/* PID_Pan\pdstate_initial */
	m_I[1] = 0.0;		/* PID_Pan\pistate_initial */


	/* set the states */
	m_s[0] = m_I[0];		/* PID_Pan\pdstate */
	m_s[1] = m_I[1];		/* PID_Pan\pistate */


	/* set the matrices */


	/* (re-)initialize the integration method */
	myintegmethod.Initialize(this);
	
	/* copy the inputs */
	m_time = t;
	CopyInputsToVariables (u);

	/* calculate initial and static equations */
	CalculateInitial ();
	CalculateStatic ();
	CalculateInput ();
	CalculateDynamic ();
	CalculateOutput ();

	/* Set the outputs */
	CopyVariablesToOutputs (y);

	/* end of initialization phase */
	m_initialize = false;

	state = mainrun;
}

/* the function that calculates the submodel */
void ControllerPan::Calculate (XXDouble *u, XXDouble *y /*, XXDouble t*/)
{
	switch (state)
	{
		case initialrun:	/* calculate the model for the first time */
			Initialize(u, y, 0.0);
			break;
		case mainrun:	/* calculate the model */
			if ( ( m_time <= (m_finish_time - m_step_size  + c_delta )) || ( m_finish_time == 0.0 ) )
			{
				/* another precessor submodel could determine the parameters of this submodel
				   and therefore the static parameter calculations need to be performed. */
				CalculateStatic ();
				CopyInputsToVariables (u);
				CalculateInput ();
				myintegmethod.Step();
				CalculateOutput ();
				CopyVariablesToOutputs (y);
			}
			else
			{
				state = finished;
			}

			if ( ( m_stop_run == true ) || (( m_finish_time != 0.0 ) && ( m_time + c_delta >= m_finish_time)) )
			{
				state = finished;
			}
			break;
		case finished:
			break;
		default:
			break;
	}
}

/* the termination function for submodel */
void ControllerPan::Terminate (XXDouble *u, XXDouble *y /*, XXDouble t */)
{
	/* copy the inputs */
	CopyInputsToVariables (u);

	/* calculate the final model equations */
	CalculateFinal ();

	/* set the outputs */
	CopyVariablesToOutputs (y);
}


/* This function calculates the initial equations of the model.
 * These equations are calculated before anything else
 */
void ControllerPan::CalculateInitial (void)
{

}

/* This function calculates the static equations of the model.
 * These equations are only dependent from parameters and constants
 */
void ControllerPan::CalculateStatic (void)
{

}

/* This function calculates the input equations of the model.
 * These equations are dynamic equations that must not change
 * in calls from the integration method (like random and delay).
 */
void ControllerPan::CalculateInput (void)
{

}

/* This function calculates the dynamic equations of the model.
 * These equations are called from the integration method
 * to calculate the new model rates (that are then integrated).
 */
void ControllerPan::CalculateDynamic (void)
{
	/* PlusMinus1\minus1 = MeasuredPan; */
	m_V[4] = m_V[5];

	/* PlusMinus1\plus1 = SetPointPan; */
	m_V[3] = m_V[7];

	/* PlusMinus1\output = PlusMinus1\plus1 - PlusMinus1\minus1; */
	m_V[2] = m_V[3] - m_V[4];

	/* PID_Pan\pdout = PID_Pan\pdstate + ((PID_Pan\kp * PlusMinus1\output) / PID_Pan\beta); */
	m_V[1] = m_s[0] + ((m_P[0] * m_V[2]) / m_P[2]);

	/* PID_Pan\pirate = PID_Pan\pdout / PID_Pan\tauI; */
	m_R[1] = m_V[1] / m_P[3];

	/* PID_Pan\output = PID_Pan\pistate + PID_Pan\pdout; */
	m_V[0] = m_s[1] + m_V[1];

	/* PID_Pan\pdrate = (PID_Pan\kp * PlusMinus1\output - PID_Pan\pdout) / (PID_Pan\beta * PID_Pan\tauD); */
	m_R[0] = (m_P[0] * m_V[2] - m_V[1]) / (m_P[2] * m_P[1]);

}

/* This function calculates the output equations of the model.
 * These equations are not needed for calculation of the rates
 * and are kept separate to make the dynamic set of equations smaller.
 * These dynamic equations are called often more than one time for each
 * integration step that is taken. This makes model computation much faster.
 */
void ControllerPan::CalculateOutput (void)
{
	/* SteeringPan = PID_Pan\output; */
	m_V[6] = m_V[0];

}

/* This function calculates the final equations of the model.
 * These equations are calculated after all the calculations
 * are performed
 */
void ControllerPan::CalculateFinal (void)
{

}



bool ControllerPan::SetFinishTime(XXDouble newtime)
{
	if ((newtime <= 0.0) || ( newtime > m_time))
	{
		m_finish_time = newtime;
		return true;
	}

	return false;
}

