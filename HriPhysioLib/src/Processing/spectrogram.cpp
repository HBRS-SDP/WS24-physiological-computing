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


Spectrogram::Spectrogram(std::size_t samples) : num_samples(0) {
    this->resize(samples);
}


Spectrogram::~Spectrogram() = default;

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

        // Store the absolute square of FFT results into `target`
        for (std::size_t i = 0; i < data.size() / 2 + 1; ++i) {
            target[counter][i] = absoluteSquare(data[i]);
        }

        std::cout << "\nOutput:\n";
        for (int i = 0; i < data.size()/2 + 1; ++i) {
            std::cout << i << ": " << data[i].real() << "+" << data[i].imag() << std::endl;
        }

        std::cout << "Counter:" << ++counter << std::endl;
    }



    return;
}


void Spectrogram::resize(std::size_t samples) {
    num_samples = samples;
    shape = pocketfft::shape_t{num_samples};
    stride = pocketfft::stride_t(shape.size());
    
    std::size_t temp = sizeof(std::complex<double>);
    for (std::size_t idx = shape.size(); idx-- > 0;) {
        stride[idx] = temp;
        temp *= shape[idx];
    }

    axes.clear();
    for (std::size_t idx = 0; idx < shape.size(); ++idx) {
        axes.push_back(idx);
    }

    input.resize(num_samples);
    data.resize(num_samples); 
    output.resize(num_samples);
}


void Spectrogram::hammingWindow(std::vector<double>& buffer, const std::size_t length) {
    
    constexpr double alpha = 0.54;
    constexpr double beta = 0.46;
    constexpr double two = 2.0; // Avoiding magic number warning

    //-- Allocate the buffer if needed.
    if (buffer.size() != length) {
        buffer.resize(length);
    }

    for (std::size_t idx = 0; idx < buffer.size(); ++idx) {
        //-- Apply the Hamming function.
        buffer[idx] = alpha - (beta * cos(two * hriPhysio::Processing::pi * (idx / static_cast<double>(length - 1))));
    }
}



void Spectrogram::resizeMatrix(std::vector<std::vector<double>>& mat, std::size_t nrows, std::size_t ncols) {
    mat.assign(nrows, std::vector<double>(ncols, 0.0));  
}


void Spectrogram::realToComplex(const double* source, std::complex<double>* target, std::size_t num_samples) {
    for (std::size_t idx = 0; idx < num_samples; ++idx) {
        target[idx] = source[idx];
    }
}


void Spectrogram::complexToReal(const std::complex<double>* source, double* target, std::size_t num_samples) {
    for (std::size_t idx = 0; idx < num_samples; ++idx) {
        target[idx] = absoluteSquare(source[idx]);
    }
}

double Spectrogram::absoluteSquare(const std::complex<double>& value) {
    return sqrt(value.real() * value.real() + value.imag() * value.imag());
}

