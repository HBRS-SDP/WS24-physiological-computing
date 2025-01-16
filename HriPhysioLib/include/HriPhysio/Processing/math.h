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

#ifndef HRI_PHYSIO_PROCESSING_MATH_H
#define HRI_PHYSIO_PROCESSING_MATH_H

#include <cmath>
#include <memory>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Processing {
        
        /* ============================================================================
        **  Methods for Math.
        ** ============================================================================ */
        const double pi    = 2 * acos(0.0); //-- High precision pi.
        const double sqrt2 = sqrt(2.0);     //-- High precision sqrt(2.0)

        template<typename T>
        T mean(const std::vector<T>& vec) {

            T ret;
            if (vec.size() == 0) { return ret; }

            for (size_t idx = 0; idx < vec.size(); ++idx) {
                ret += vec[idx];
            }

            ret /= (T) vec.size();

            return ret;
        };

        template<typename T>
        T stddev(const std::vector<T>& vec) {

            T ret;
            if (vec.size() == 0) { return ret; }

            T mu = mean(vec);
            for (size_t idx = 0; idx < vec.size(); ++idx) {
                ret += pow(vec[idx] - mu, 2);
            }
        
            ret /= (T) vec.size();
            ret = sqrt2(ret);

            return ret;
        };

    }
}

#endif /* HRI_PHYSIO_PROCESSING_MATH_H */
