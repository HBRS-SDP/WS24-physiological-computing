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

#include <HriPhysio/Processing/spectrogram.h>

using namespace hriPhysio::Processing;


Spectrogram::Spectrogram(std::size_t samples) {
    this->resize(samples);
}


Spectrogram::~Spectrogram() {
}

#include <iostream>
void Spectrogram::process(const std::vector<double>& source, std::vector<std::vector<double>>& target, const double sample_rate, const double stride_ms/*=20.0*/, const double window_ms/*=20.0*/) {

    const std::size_t stride_size = (0.001 * sample_rate * stride_ms);
    const std::size_t window_size = (0.001 * sample_rate * window_ms);

    std::cout << "stride: " << stride_size << std::endl
              << "window: " << window_size << std::endl;


    this->resize(window_size);
    this->resizeMatrix(target, sample_rate/2 + 1, window_size/2 + 1);

    std::vector<double> window;
    this->hammingWindow(window, window_size);

    ////////////////////////////////////
    std::cout << "Window:\n";
    for (int i = 0; i < window.size(); ++i) {
        if (i) std::cout << ",";
        std::cout << window[i] << " ";
    } std::cout << std::endl;
    ////////////////////////////////////

    //-- Copy the data into the complex buffer.
    //this->realToComplex(source.data(), this->input.data(), this->num_samples);

    int counter = 0;
    for (int idx = 0; idx < source.size(); idx += stride_size) {

        //std::cout << "\nInput:\n";
        for (int i = 0; i < window_size; ++i) {

            if (idx+i < source.size()) {
                this->input[i] = (source[idx+i] * window[i]);
            } else {
                this->input[i] = 0.;
            }

            //std::cout << this->input[i] << " ";
        }   //std::cout << std::endl;

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

        std::cout << "\nOutput:\n";
        for (int i = 0; i < data.size()/2 + 1; ++i) {
            std::cout << i << ": " << data[i].real() << "+" << data[i].imag() << std::endl;
        }

        std::cout << "Counter:" << ++counter << std::endl;
    }


//    //-- Error checking.
//
//    if (source.size() != this->num_samples) {
//        //-- Need to reset pocketfft elements.
//        this->resize(source.size());
//    }
//
//    if (source.size() != target.size()) {
//        //-- Make the target the expected output size.
//        target.resize(this->num_samples);
//    }
//
//
//    //-- Copy the data into the complex buffer.
//    this->realToComplex(source.data(), this->input.data(), this->num_samples);
//
//
//    //-- Compute the forward complex-to-complex transform.
//    pocketfft::c2c(
//        /* shape      =*/ this->shape,
//        /* stride_in  =*/ this->stride,
//        /* stride_out =*/ this->stride,
//        /* axes       =*/ this->axes,
//        /* forward    =*/ pocketfft::FORWARD,
//        /* data_in    =*/ this->input.data(),
//        /* data_out   =*/ this->data.data(),
//        /* fct        =*/ 1.
//    );
//
//
//    //-- Double the first half.
//    std::size_t half_samples  = this->num_samples >> 1;
//    for (std::size_t idx = 1; idx < half_samples+1; ++idx) {
//        this->data[idx] *= 2.0;
//    }
//
//
//    //-- Drop the second half.
//    for (std::size_t idx = half_samples+1; idx < num_samples; ++idx) {
//        this->data[idx] = (0.,0.);
//    }
//
//
//    //-- Compute the backward complex-to-complex transform.
//    pocketfft::c2c(
//        /* shape      =*/ this->shape,
//        /* stride_in  =*/ this->stride,
//        /* stride_out =*/ this->stride,
//        /* axes       =*/ this->axes,
//        /* forward    =*/ pocketfft::BACKWARD,
//        /* data_in    =*/ this->data.data(),
//        /* data_out   =*/ this->output.data(),
//        /* fct        =*/ 1. / this->num_samples
//    );
//    
//
//    //-- Get the magnitude of the complex elements in the vector.
//    this->complexToReal(this->output.data(), target.data(), this->num_samples);


    return;
}


void Spectrogram::resize(const std::size_t samples) {

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


void Spectrogram::hammingWindow(std::vector<double>& buffer, const std::size_t length) {
    
    //-- Allocate the buffer if needed.
    if (buffer.size() != length) {
        buffer.resize(length);
    }

    for (std::size_t idx = 0; idx < buffer.size(); ++idx) {
        //-- Apply the Hamming function.
        buffer[idx] = 0.54 - (0.46 * cos(2. * hriPhysio::Processing::pi * (idx / ((length - 1) * 1.))));
    }

    return;
}


void Spectrogram::resizeMatrix(std::vector<std::vector<double>>& mat, const std::size_t nrows, const std::size_t ncols) {
    mat.resize(nrows);
    for (std::size_t idx = 0; idx < mat.size(); ++idx) {
        mat[idx].resize(ncols);
    }
}


void Spectrogram::realToComplex(const double* source, std::complex<double>* target, const std::size_t num_samples) {
    
    for (std::size_t idx = 0; idx < num_samples; ++idx) {
        target[idx] = source[idx];
    }
    return;
}


void Spectrogram::complexToReal(const std::complex<double>* source, double* target, const std::size_t num_samples) {
    
    for (std::size_t idx = 0; idx < num_samples; ++idx) {
        target[idx] = this->absoluteSquare(source[idx]);
    }
    return;
}


double Spectrogram::absoluteSquare(const std::complex<double> value) {

    return sqrt(value.real()*value.real() + value.imag()*value.imag());
}

