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

#ifndef HRI_PHYSIO_PROCESSING_BUTTERWORTH_HIGH_PASS_H
#define HRI_PHYSIO_PROCESSING_BUTTERWORTH_HIGH_PASS_H

#include <cmath>
#include <memory>

#include <HriPhysio/Processing/biquadratic.h>
#include <HriPhysio/Processing/math.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Processing {
        class ButterworthHighPass;
    }
}

class hriPhysio::Processing::ButterworthHighPass : public hriPhysio::Processing::Biquadratic {
public:

    /* ============================================================================
    **  Main Constructor.
    ** 
    ** @param rate    Sampling-rate of the provided signal.
    ** ============================================================================ */
    ButterworthHighPass(const unsigned int rate);


    /* ============================================================================
    **  Calculates the coefficients for a butterworth high pass filter.
    **
    ** @param freq    The center frequency used to calculate filter coefficients.
    ** ============================================================================ */
    void updateCoefficients(const double freq);
};

#endif /* HRI_PHYSIO_PROCESSING_BUTTERWORTH_HIGH_PASS_H */
