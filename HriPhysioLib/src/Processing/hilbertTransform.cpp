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

#include <HriPhysio/Processing/hilbertTransform.h>
#include <cmath>  // For std::sqrt

using namespace hriPhysio::Processing;

namespace {
    constexpr double MULTIPLIER = 2.0;
}

HilbertTransform::HilbertTransform(std::size_t samples) 
    : num_samples(samples) {
    this->resize(samples);
}

HilbertTransform::~HilbertTransform() = default;

void HilbertTransform::process(const std::vector<double>& source, std::vector<double>& target) {
    //-- Error checking.
    if (source.size() != this->num_samples) {
        //-- Need to reset pocketfft elements.
        this->resize(source.size());
    }

    if (source.size() != target.size()) {
        //-- Make the target the expected output size.
        target.resize(this->num_samples);
    }

    //-- Copy the data into the complex buffer.
    this->realToComplex(source.data(), this->input.data(), this->num_samples);

    //-- Compute the forward complex-to-complex transform.
    pocketfft::c2c(
        /* shape      =*/ this->shape,
        /* stride_in  =*/ this->stride,
        /* stride_out =*/ this->stride,
        /* axes       =*/ this->axes,
        /* forward    =*/ pocketfft::FORWARD,
        /* data_in    =*/ this->input.data(),
        /* data_out   =*/ this->data.data(),
        /* fct        =*/ 1.0
    );

    //-- Double the first half.
    std::size_t half_samples = this->num_samples >> 1;
    for (std::size_t idx = 1; idx <= half_samples; ++idx) {
        this->data[idx] *= MULTIPLIER;
    }

    //-- Drop the second half.
    for (std::size_t idx = half_samples + 1; idx < num_samples; ++idx) {
        this->data[idx] = std::complex<double>(0.0, 0.0);
    }

    //-- Compute the backward complex-to-complex transform.
    pocketfft::c2c(
        /* shape      =*/ this->shape,
        /* stride_in  =*/ this->stride,
        /* stride_out =*/ this->stride,
        /* axes       =*/ this->axes,
        /* forward    =*/ pocketfft::BACKWARD,
        /* data_in    =*/ this->data.data(),
        /* data_out   =*/ this->output.data(),
        /* fct        =*/ 1.0 / static_cast<double>(this->num_samples)
    );

    //-- Get the magnitude of the complex elements in the vector.
    this->complexToReal(this->output.data(), target.data(), this->num_samples);
}

void HilbertTransform::resize(const std::size_t samples) {
    //-- Set the number of samples.
    this->num_samples = samples;

    //-- Set up the pocketfft members.
    this->shape = pocketfft::shape_t{this->num_samples};
    this->stride = pocketfft::stride_t(shape.size());

    std::size_t temp = sizeof(std::complex<double>);
    for (std::ptrdiff_t idx = static_cast<std::ptrdiff_t>(this->shape.size()) - 1; idx >= 0; --idx) {
        this->stride[static_cast<std::size_t>(idx)] = temp;
        temp *= this->shape[static_cast<std::size_t>(idx)];
    }

    this->axes.clear();
    for (std::size_t idx = 0; idx < this->shape.size(); ++idx) {
        this->axes.push_back(idx);
    }

    //-- Resize the complex buffers.
    this->input.resize(this->num_samples);
    this->data.resize(this->num_samples); 
    this->output.resize(this->num_samples); 
}

void HilbertTransform::realToComplex(const double* source, std::complex<double>* target, const std::size_t num_samples) {
    for (std::size_t idx = 0; idx < num_samples; ++idx) {
        target[idx] = source[idx];
    }
}

void HilbertTransform::complexToReal(const std::complex<double>* source, double* target, const std::size_t num_samples) {
    for (std::size_t idx = 0; idx < num_samples; ++idx) {
        target[idx] = this->absoluteSquare(source[idx]);
    }
}

double HilbertTransform::absoluteSquare(const std::complex<double>& value) {
    return std::sqrt(value.real() * value.real() + value.imag() * value.imag());
}
