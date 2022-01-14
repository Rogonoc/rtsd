/**********************************************************
 * This file is generated by 20-sim ANSI-C Code Generator  
 *
 *  file:  common\xxinverse.h
 *  subm:  ControllerTilt
 *  model: ControllerTilt
 *  expmt: Jiwy-with-controller
 *  date:  January 14, 2022
 *  time:  9:33:17 AM
 *  user:  20-sim 4.8 Campus License 
 *  from:  Universiteit Twente
 *  build: 4.8.3.10415
 **********************************************************/

#ifndef XX_INVERSE_H
#define XX_INVERSE_H

/* 20-sim include files */
#include "xxtypes.h"
#include "xxmatrix.h"

/* 20-sim function prototypes */
void XXIndex (XXMatrix *v);
void XXPermute (XXMatrix *v, XXMatrix *p, XXDouble *workarray);
void XXSubstitute (XXMatrix *dest, XXMatrix *v);
void XXSwapRows (XXMatrix *dest, XXInteger row1, XXInteger row2);
XXInteger XXPivot (XXMatrix *dest, XXMatrix *p, XXInteger i);
XXDouble XXDecompose (XXMatrix *dest, XXMatrix *p);
XXDouble XXCrout1 (XXMatrix *dest, XXMatrix *v, XXDouble *workarray);
XXDouble XXCrout2 (XXMatrix *dest, XXMatrix *CroutMat, XXMatrix *x, XXMatrix *y, XXDouble *workarray);
XXDouble XXInverse (XXMatrix *mat_dest, XXMatrix *mat_source, XXDouble *workarray);

#endif

