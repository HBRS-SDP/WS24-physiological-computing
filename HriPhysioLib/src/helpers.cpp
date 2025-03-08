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
 * Copyright: (C) 2024, Yash Somaiya, 
 *       Hochschule Bonn-Rhein-Sieg (H-BRS), All rights reserved.
 * 
 * Authors: Yash Somaiya yash.somaiya@smail.inf.h-brs.de
 * 
 * CopyPolicy: Released under the terms of the MIT License.
 *      See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/helpers.h>


std::vector< std::string > hriPhysio::parseString(const std::string& str) {

    std::stringstream ss(str);
    std::vector< std::string > ret;

    std::string s;
    while (ss >> s) {
        ret.push_back(s);
    }

    return ret;
}


std::vector< double > hriPhysio::toVecDouble(const std::vector< std::string >& source, size_t idx/*=0*/) {

    std::vector< double > ret;

    while (idx < source.size()) {
        ret.push_back( std::stod(source[idx]) );
        ++idx;
    }

    return ret;
}


std::string hriPhysio::combineString(const std::vector< std::string >& source, size_t idx/*=0*/) {
    
    std::string ret = "";
    
    while (idx < source.size()) {
        if (ret.length()) ret += " ";
        ret += source[idx];
        ++idx;
    }

    return ret;
} 


void stringTransform(std::string& str, int (*func)(int)) {
    std::string::iterator it = str.begin();
    while (it != str.end()) {
        *it = (*func)(*it);
        ++it;
    }
}


void hriPhysio::toLower(std::string& str) { 
    stringTransform(str, std::tolower);
}


void hriPhysio::toUpper(std::string& str) { 
    stringTransform(str, std::toupper);
}
