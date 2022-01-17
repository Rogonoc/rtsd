/* Libraries */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iterator>
#include <signal.h>
#include <vector>
#include <sys/syscall.h>

/* Meijer framework */
#include "framework/multiCommClass.h"
#include "framework/runnableClass.h"
#include "framework/superThread.h"
#include "framework/icoCommClass.h"

// #include <Your 20-sim-code-generated h-file?> Don't forget to compile the cpp file by adding it to CMakeLists.txt [DONE]
#include "Controller/Controller.h" // 20-sim submodel class include file 

// pi-definition
#define PI 3.1459

volatile bool exitbool = false;

/* SIGNAL CATCHER */
void exit_handler(int s)
{
    printf("Caught signal %d\n", s);
    exitbool = true;
}

/* ReadConvert */
void ReadConvert(const double* src, double *dst)
{
    static double encoderCount = 16383;
    static double initialCount0 = 0;
    static double initialCount1 = 0;
    static double countPerRev0 = 5000;
    static double countPerRev1 = 2000;
    static double lastKnownGoodValue0 = 0;
	static double lastKnownGoodValue1 = 0;

	/* Scaling + Filtering (upper motor) */
	if ( (src[0] > encoderCount) || (src[0] < 0) )
	{
        // Negative turn
        if ( src[0] > ((encoderCount/2) + 1) )
        {
            dst[0] = ( initialCount0 - (encoderCount - abs(src[0] - lastKnownGoodValue0)) ) * (2 * (double)PI / countPerRev0);              // in rad
            lastKnownGoodValue0 = ( initialCount0 - (encoderCount - abs(src[0] - lastKnownGoodValue0)) ) * (2 * (double)PI / countPerRev0); // in rad
            initialCount0 = initialCount0 + lastKnownGoodValue0; // in rad
        }
        // Positive turn
        else if ( src[0] < ((encoderCount/2) + 1) )
        {
            dst[0] = ( initialCount0 + (encoderCount - abs(src[0] - lastKnownGoodValue0)) ) * (2 * (double)PI / countPerRev0);               // in rad
            lastKnownGoodValue0 = ( initialCount0 + (encoderCount - abs(src[0] - lastKnownGoodValue0)) ) * (2 * (double)PI / countPerRev0); // in rad
            initialCount0 = initialCount0 + lastKnownGoodValue0; // in rad
        }
	}
	else
	{
		dst[0] = initialCount0;
	}

	/* Scaling + Filtering (lower motor) */
	if ( (src[1] > encoderCount) || (src[1] < 0) )
	{
        // Negative turn
        if ( src[1] > ((encoderCount/2) + 1) )
        {
            dst[1] = ( initialCount1 - (encoderCount - abs(src[1] - lastKnownGoodValue1)) ) * (2 * (double)PI / countPerRev1);              // in rad
            lastKnownGoodValue1 = ( initialCount1 - (encoderCount - abs(src[1] - lastKnownGoodValue1)) ) * (2 * (double)PI / countPerRev1); // in rad
            initialCount1 = initialCount1 + lastKnownGoodValue1; // in rad
        }
        // Positive turn
        else if ( src[1] < ((encoderCount/2) + 1) )
        {
            dst[1] = ( initialCount1 + (encoderCount - abs(src[1] - lastKnownGoodValue1)) ) * (2 * (double)PI / countPerRev1);              // in rad
            lastKnownGoodValue1 = ( initialCount1 + (encoderCount - abs(src[1] - lastKnownGoodValue1)) ) * (2 * (double)PI / countPerRev1); // in rad
            initialCount1 = initialCount1 + lastKnownGoodValue1; // in rad
        }
	}
	else
	{
		dst[1] = initialCount1;
	}	
}

/* WriteConvert */
void WriteConvert(const double* src, double *dst)
{
    static double motorRange = 2047; // In number scale
    static double motorVolt = 12;    // In volt
	static double lastKnownGoodValue = 0;

	/* Scaling + Filtering (both motors) */
	if ( (src[0] > motorRange) || (src[0] < -motorRange) )
	{
		dst[0] = (src[0]/motorVolt) * motorRange;             // Motor 1 of 12 V, scaled to [-2047, 2047]
		dst[1] = (src[0]/motorVolt) * motorRange;             // Motor 2 of 12 V, scaled to [-2047, 2047]
		lastKnownGoodValue = (src[0]/motorVolt) * motorRange; // Motors  of 12 V, scaled to [-2047, 2047]
	}
	else
	{
		dst[0] = lastKnownGoodValue;
		dst[1] = lastKnownGoodValue;
	}
}

/* MAIN FUNCTION */
int main()
{
    /* CREATE CNTRL-C HANDLER */
    signal(SIGINT, exit_handler);

    printf("Press Ctrl-C to stop program\n"); // Note: this will 
        // not kill the program; just jump out of the wait loop. Hence,
        // you can still do proper clean-up. You are free to alter the
        // way of determining when to stop (e.g., run for a fixed time).
    
	/* PARAMETERS */
	int P1_PWM = 0;
	int P2_PWM = 0;
	int P1_ENC = 0;
	int P2_ENC = 1;

	/* CREATE PARAM FOR CONTROLLER */
	int _sendParameters [] = {P1_PWM, -1, P2_PWM, -1, -1, -1, -1, -1};
	int _receiveParameters [] = {P1_ENC, -1, -1, P2_ENC, -1, -1, -1, -1, -1, -1, -1, -1};

	icoComm = new IcoComm(_sendParameters, _receiveParameters);
	icoComm.SetReadConvertFcn(&ReadConvert);   // Scaling + filtering of input
	icoComm.SetWriteConvertFcn(&WriteConvert); // Scaling + filtering of output

	frameworkComm *controller_uPorts[] = 
	{
		new IDDPCom(_sendParameters),
		icoComm
	};

	frameworkComm *controller_yPorts[] =
	{
		new IDDPComm(_receiveParameters),
		icoComm
	};

	/* CREATE WRAPPER FOR CONTROLLER */
	Controller *controller_class = new Controller;
	runnable *controller_runnable = new wrapper<Controller>(
		controller_class, controller_uPorts, controller_yPorts, 2, 1) // Two inputs (setpoint, encoder), one output (control signal)

	/* INITIALIZE XENOTHREAD FOR CONTROLLER */
	xenoThread controllerClass(controller_runnable);
	controllerClass.init(1000000, 98, 0); // Frequency of 1 kHz = 1 ms = 1000000 ns; Priority of 98 out 120; Affinity of 0

	/* START THREADS */
	controllerClass.start("controller"); // Controller should receive topic about computed setpoint?

    /* WAIT FOR CNTRL-C */
    timespec t = {.tv_sec=0, .tv_nsec=100000000}; // 1/10 second
    while (!exitbool)
    {
        // Let the threads do the real work
        //nanosleep(&t, NULL);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
        // Wait for Ctrl-C to exit
    }
    printf("Ctrl-C was pressed: Stopping gracefully...\n");

    /* CLEANUP HERE */
	controllerClass.stopThread();
	controller_runnable->~xenoThread();
	controller_runnable->~runnable();

	/* FINISH */
    return 0;
}