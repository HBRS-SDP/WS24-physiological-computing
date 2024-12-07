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

#include <HriPhysio/Processing/hilbertTransform.h>

using namespace hriPhysio::Processing;


HilbertTransform::HilbertTransform(std::size_t samples) {
    this->resize(samples);
}


HilbertTransform::~HilbertTransform() {
}


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
        /* fct        =*/ 1.
    );


    //-- Double the first half.
    std::size_t half_samples  = this->num_samples >> 1;
    for (std::size_t idx = 1; idx < half_samples+1; ++idx) {
        this->data[idx] *= 2.0;
    }


    //-- Drop the second half.
    for (std::size_t idx = half_samples+1; idx < num_samples; ++idx) {
        this->data[idx] = (0.,0.);
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
        /* fct        =*/ 1. / this->num_samples
    );
    

    //-- Get the magnitude of the complex elements in the vector.
    this->complexToReal(this->output.data(), target.data(), this->num_samples);


    return;
}


void HilbertTransform::resize(const std::size_t samples) {

    //-- Set the number of samples.
    this->num_samples = samples;

    //-- Set up the pocketfft members.
    this->shape = pocketfft::shape_t{this->num_samples};
    this->stride = pocketfft::stride_t(shape.size());
    
    std::size_t temp = sizeof(std::complex<double>);
    for (int idx = this->shape.size()-1; idx >= 0; --idx) {
        this->stride[idx] = temp;
        temp *= this->shape[idx];
    }

    // TODO: REMOVE.
    //std::size_t ndata = 1;
    //for (std::size_t idx = 0; idx < this->shape.size(); ++idx) {
    //    ndata *= this->shape[idx];
    //}

    this->axes.clear();
    for (std::size_t idx = 0; idx < this->shape.size(); ++idx) {
        this->axes.push_back(idx);
    }

    //-- Resize the complex buffers.
    this->input.resize(this->num_samples);
    this->data.resize(this->num_samples); 
    this->output.resize(this->num_samples); 

    return;
}


void HilbertTransform::realToComplex(const double* source, std::complex<double>* target, const std::size_t num_samples) {
    
    for (std::size_t idx = 0; idx < num_samples; ++idx) {
        target[idx] = source[idx];
    }
    return;
}


void HilbertTransform::complexToReal(const std::complex<double>* source, double* target, const std::size_t num_samples) {
    
    for (std::size_t idx = 0; idx < num_samples; ++idx) {
        target[idx] = this->absoluteSquare(source[idx]);
    }
    return;
}


double HilbertTransform::absoluteSquare(const std::complex<double> value) {

    return sqrt(value.real()*value.real() + value.imag()*value.imag());
}

