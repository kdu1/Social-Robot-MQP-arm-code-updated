//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cubic_traj.h
//
// Code generation for function 'cubic_traj'
//

#ifndef CUBIC_TRAJ_H
#define CUBIC_TRAJ_H

// Include files
//#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <complex>
#include <valarray>
//#include <ros/ros.h>

typedef std::complex<float> Complex;
typedef std::valarray<Complex> CArray;


//has to do with CArray being the same one from robot
// Function Declarations
CArray cubic_traj(float tf, std::vector<float> vi, std::vector<float> vf,
                                        std::vector<float> pi, std::vector<float> pf);

#endif
// End of code generation (cubic_traj.h)
