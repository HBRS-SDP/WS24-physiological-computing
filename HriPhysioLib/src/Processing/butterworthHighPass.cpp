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

/* ================================================================================
 * Copyright: (C) 2024, Prachi Sheth,
 *     Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
 * 
 * Author: 
 *     Prachi Sheth <prachi.sheth@smail.inf.h-brs.de>
 * 
 * CopyPolicy: Released under the terms of the MIT License.
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Processing/butterworthHighPass.h>

using namespace hriPhysio::Processing;


ButterworthHighPass::ButterworthHighPass(const unsigned int rate) : 
    Biquadratic(rate) {
    
}


void ButterworthHighPass::updateCoefficients(const double freq) {
    
    //-- Allocate some local variables.
    constexpr double SQRT2 = 1.41421356237;  
    constexpr double TWO = 2.0;

    //-- Compute the coeff for the given center frequency.
    double c  =  tan(hriPhysio::Processing::pi * freq / sampling_rate);
    double cc =  c * c;

    a0 =  1.0 / (1.0 + (SQRT2 * c) + cc);
    a1 = -a0 * TWO;
    a2 =  a0;
    b1 =  a0 * TWO * (cc - 1.0);
    b2 =  a0 * (1.0 - (SQRT2 * c) + cc);

    //-- Cache this center frequency, 
    //-- so that it is not recomputed.
    center_frequency = freq;
    
    return;
}
