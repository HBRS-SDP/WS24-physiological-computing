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

#ifndef HRI_PHYSIO_PROCESSING_SPECTROGRAM_H
#define HRI_PHYSIO_PROCESSING_SPECTROGRAM_H

#include <cmath>
#include <memory>

#include <PocketFFT/pocketfft.h>
#include <HriPhysio/Processing/math.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Processing {
        class Spectrogram;
    }
}

class hriPhysio::Processing::Spectrogram {
private:

    /* ============================================================================
	**  Variables received from the constructor.
	** ============================================================================ */
	std::size_t num_samples;

    
    /* ============================================================================
    **  PocketFFT variables.
    ** ============================================================================ */ 
    pocketfft::shape_t  shape;
    pocketfft::stride_t stride;
    pocketfft::shape_t  axes;

    std::vector< std::complex<double> > input;
    std::vector< std::complex<double> > data;
    std::vector< std::complex<double> > output;

    
public:
    /* ============================================================================
    **  Main Constructor.
    ** ============================================================================ */
    Spectrogram(std::size_t samples);


    /* ===========================================================================
	**  Destructor.
	** =========================================================================== */
    ~Spectrogram();


    /* ===========================================================================
	**  Process.
	** =========================================================================== */
    void process(const std::vector<double>& source, std::vector<std::vector<double>>& target, const double sample_rate, const double stride_ms=20.0, const double window_ms=20.0);


    /* ===========================================================================
	**  Resize.
	** =========================================================================== */
    void resize(const std::size_t samples);


private:
    /* ===========================================================================
    **  Hamming Window.
    ** =========================================================================== */
    void hammingWindow(std::vector<double>& buffer, const std::size_t length);


    /* ===========================================================================
    **  Resize Matrix.
    ** =========================================================================== */
    void resizeMatrix(std::vector<std::vector<double>>& mat, const std::size_t nrows, const std::size_t ncols);


    /* ===========================================================================
    **  Real to Complex.
    ** =========================================================================== */
    void realToComplex(const double* source, std::complex<double>* target, const std::size_t num_samples);
    

    /* ===========================================================================
    **  Complex to Real.
    ** =========================================================================== */
    void complexToReal(const std::complex<double>* source, double* target, const std::size_t num_samples);


    /* ===========================================================================
    **  Absolute Square.
    ** =========================================================================== */
    double absoluteSquare(const std::complex<double> value);
    
};

#endif /* HRI_PHYSIO_PROCESSING_SPECTROGRAM_H */
