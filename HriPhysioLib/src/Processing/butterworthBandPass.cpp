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
#include <HriPhysio/Processing/butterworthBandPass.h>

using namespace hriPhysio::Processing;


ButterworthBandPass::ButterworthBandPass(const unsigned int rate, const double width) : 
    Biquadratic(rate, width) {
    
}


void ButterworthBandPass::updateCoefficients(const double freq) {

    //-- Allocate some local variables.
    double c, d;

    //-- Compute the coeff for the given center frequency.
    c  =  1.0 / tan( hriPhysio::Processing::pi * (band_width / sampling_rate) );
    d  =  2.0 * cos( 2.0 * hriPhysio::Processing::pi * (freq / sampling_rate) );
    a0 =  1.0 / (c + 1.0);
    a1 =  0.0;
    a2 = -a0;
    b1 = -a0 * c * d;
    b2 =  a0 * (c - 1.0);
    
    //-- Cache this center frequency, 
    //-- so that it is not recomputed.
    center_frequency = freq;
    
    return;
}
