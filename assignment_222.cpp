#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>
#include <vector>

#define NUM_THREADS 1
#define BILLION 1000000000L;

// Global parameters
std::vector<double> accum(100);

// Function that a thread has to execute
void *threadFunc(void *pArg) 
{
    // Parameters
    int i, j, k;                        // Loop variables
    struct timespec start, stop, ts;    // Time variables
    ts.tv_nsec = 1000000;               // 1 msec
    float test = 0.95;                  // Parameter for computational load

    // Let the thread perform computations and time it
    for(i = 0; i < 100; i++)
    {
        // Start clock; exit if there is an error
        if (clock_gettime(CLOCK_REALTIME, &start) == -1)
        {
            perror("clock gettime");
            exit(EXIT_FAILURE);
        }

        /* ---------- COMPUTATIONAL LOAD ---------- */
        for (j = 0; j < 100000; j++)
        {
            test = pow(test, 0.95);
        }
        /* ---------------------------------------- */

        // Sleep
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &ts, NULL);

        // Stop clock; exit if there is an error
        if (clock_gettime(CLOCK_REALTIME, &stop) == -1)
        {
            perror("clock gettime");
            exit(EXIT_FAILURE);
        }

        // Accumulate measured time
        accum[i] = (stop.tv_nsec - start.tv_nsec);
        
    }

    // Print timings when it is finished with computational load
    for (k = 0; k < 100; k++)
    {
        printf("%lf\n", accum[k]);
    }

    return 0;
}


// Main function
int main(int argc, char **argv)
{
    // Initialize parameters
    int i;
    int tNum[NUM_THREADS];
    pthread_t tid[NUM_THREADS];

    // Lock memory to approximate real time behavior
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Creating threads
    for(i = 0; i < NUM_THREADS; i++)
    {
        tNum[i] = i;
        pthread_create(&tid[i], NULL, threadFunc, &tNum[i]);
    }

    // Waiting for the created threads to terminate
    for(i = 0; i < NUM_THREADS; i++)
    {
        pthread_join(tid[i], NULL);
    }

    // Wait for other threads to complete their work
    pthread_exit(NULL);

    return(EXIT_SUCCESS);
}