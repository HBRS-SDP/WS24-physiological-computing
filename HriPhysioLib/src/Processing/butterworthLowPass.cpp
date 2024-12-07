/* ================================================================================
 * Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Processing/butterworthLowPass.h>

using namespace hriPhysio::Processing;


ButterworthLowPass::ButterworthLowPass(const unsigned int rate) : 
    Biquadratic(rate) {
    
}


void ButterworthLowPass::updateCoefficients(const double freq) {

    //-- Allocate some local variables.
    double c, cc;
    
    //-- Compute the coeff for the given center frequency.
    c  = 1.0 / tan(hriPhysio::Processing::pi * freq / sampling_rate);
    cc = c * c;
    a0 = 1.0 / (1.0 + (hriPhysio::Processing::sqrt2 * c) + cc);
    a1 = a0 * 2.0;
    a2 = a0;
    b1 = a0 * 2.0 * (1.0 - cc);
    b2 = a0 * (1.0 - (hriPhysio::Processing::sqrt2 * c) + cc);

    //-- Cache this center frequency, 
    //-- so that it is not recomputed.
    center_frequency = freq;
    
    return;
}
