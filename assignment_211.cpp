// Libraries
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>
#include <vector>
#include <numeric>

// Global parameters
std::vector<double>     timing(100);
double                  mean, var, rlt_std, min, max;
double                  curr_sec, curr_nsec, prev_sec, prev_nsec;
int                     i, j, k, n, m;

// Function that a thread has to execute
void *threadFunc(void *pArg) 
{
    // Time parameters
    struct timespec     tp, ts;
    ts.tv_nsec = 1000;                  // 1 ms for nanosleep

    // Other parameters
    float               test = 0.95;    // Parameter for computational load

    // Let the thread perform computations and time it
    for(i = 0; i < 101; i++)
    {
        /* ---------- COMPUTATIONAL LOAD ---------- */
        for (j = 0; j < 100000; j++)
        {
            test = pow(test, 0.95);
        }
        /* ---------------------------------------- */

        // Wait (until the sleep is finished)
        nanosleep(&ts, NULL);

        // Store timing of loop
        if (clock_gettime(CLOCK_MONOTONIC, &tp) == -1)
        {
            perror("clock gettime");
        }

        /* ---------------- LOG TIME ---------------- */
        curr_sec = tp.tv_sec;
        curr_nsec = tp.tv_nsec;

        if(i == 0)
        {
            // Do nothing: let times initialize
        }
        else if(curr_sec != prev_sec)
        {
            timing[i-1] = double(1000000000) - prev_nsec + curr_nsec;
        }
        else
        {
            timing[i-1] = curr_nsec - prev_nsec;
        }

        prev_sec = curr_sec;
        prev_nsec = curr_nsec;
        /* ------------------------------------------- */
    }

    return 0;
}

// Function that prints out logged data
void information()
{
    // Print timings when it is finished with loop
    for (k = 0; k < 100; k++)
    {
        printf("%lf\n", timing[k]);
    }

    // Mean
    mean = std::accumulate(timing.begin(), timing.end(), 0) / timing.size();

    // Variance
    var = 0;
    for (n = 0; n < timing.size(); n++)
    {
        var += (timing[n] - mean) * (timing[n] - mean);
    }
    var = var / timing.size();

    // Relative standard deviation
    rlt_std = (sqrt(var) / abs(mean)) * 100;

    // Find minimum and maximum
    min = timing[0];
    max = timing[0];
    for(m = 0; m < timing.size(); m++)
    {
        // Minimum
        if(timing[m] < min)
        {
            min = timing[m];
        }

        // Maximum
        if(timing[m] > max)
        {
            max = timing[m];
        }
    }

    // Print above parameters
    printf("\nMINIMUM: %lf \nMAXIMUM: %lf \nMEAN: %lf \nVARIANCE: %lf \nSTANDARD DEVIATION: %lf \nRELATIVE DEVIATION: %lf \n",
    min, max, mean, var, sqrt(var), rlt_std);

    return;
}

// Main function
int main(int argc, char **argv)
{
    // Initialize parameters
    pthread_t thread;

    // Creating a pthread
    pthread_create(&thread, NULL, threadFunc, NULL);

    // Join the thread and wait until it is done
    pthread_join(thread, NULL);

    // Print out recorded data
    information();

    return(EXIT_SUCCESS);
}