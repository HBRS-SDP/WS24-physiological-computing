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

#ifndef HRI_PHYSIO_PROCESSING_BIQUADRATIC_H
#define HRI_PHYSIO_PROCESSING_BIQUADRATIC_H

#include <cmath>
#include <memory>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Processing {
        class Biquadratic;
    }
}

class hriPhysio::Processing::Biquadratic {
protected:

    /* ============================================================================
	**  Variables received from the constructor.
	** ============================================================================ */
	unsigned int sampling_rate;
    double band_width;
    double center_frequency;
	
    
    /* ============================================================================
    **  Filter Coefficients.
    ** ============================================================================ */ 
    double a0, a1, a2, b1, b2;

public:

    /* ============================================================================
    **  Main Constructor.
    ** 
    ** @param rate    Sampling-rate of the provided signal.
    ** @param width   Width for the band pass/notch.        [Optional arg]
    ** ============================================================================ */
    Biquadratic(const unsigned int rate, const double width=0.0);


    /* ============================================================================
    **  Filter the provided data by bilinear transformation. 
    **    Note: if provided a different freq, coefficients are recalculated.
    **
    ** @param source      Array of data to be filtered.
    ** @param target      Array of where the filtered data should go.
    ** @param numSamples  The number of samples in the input array.
    ** @param  freq       The center frequency to filter at.
    ** ============================================================================ */
    void filter(const double* source, double* target, const std::size_t numSamples, const double freq);


    /* ============================================================================
    **  Pure virtual function to be implemented by inheriter to set the following
    **  variables which get used in the bilinear transformation: a0, a1, a2, b1, b2.
    **
    ** @param freq    The center frequency used to calculate filter coefficients.
    ** ============================================================================ */
    virtual void updateCoefficients(const double freq) = 0;


    /* ============================================================================
    **  Sets the internal variable sampling_rate used for computing the coefficients.
    **    Note: Also clears the cached center_frequency variable, to force recompute.
    **
    ** @param rate    The sampling rate of the signal to be filtered.
    ** ============================================================================ */
    void setSamplingRate(const unsigned int rate);


    /* ============================================================================
    **  Sets the internal variable band_width used for computing the coefficients.
    **    Note: Also clears the cached center_frequency variable, to force recompute.
    **          Band Width is also only really used in band-pass/notch filters!!
    **
    ** @param rate    The sampling rate of the signal to be filtered.
    ** ============================================================================ */
    void setBandWidth(const double width);

protected:
    /* ============================================================================
    **  Performs a bilinear transformation on the source signal and places it in 
    **  the target array, according to the set internal coefficients.
    **    Note: target needs to be PRE-ALLOCATED with the same size as source!!
    **
    ** @param source      Array of data to be filtered.
    ** @param target      Array of where the filtered data should go.
    ** @param numSamples  The number of samples in the input array.
    ** ============================================================================ */
    void bilinearTransformation(const double* source, double* target, const std::size_t numSamples);
};

#endif /* HRI_PHYSIO_PROCESSING_BIQUADRATIC_H */
