//
// Created by zlc on 2021/5/9.
//

#ifndef _G2O_CUSTOMBUNDLE_RANDOM_H_
#define _G2O_CUSTOMBUNDLE_RANDOM_H_

#include <math.h>
#include <stdlib.h>


inline double RandDouble()
{
    double r = static_cast<double>( rand() );       // int强制转换为double

    return r / RAND_MAX;
}

inline double RandNormal()
{
    double x1, x2, w;

    do {
        x1 = 2.0 * RandDouble() - 1.0;
        x2 = 2.0 * RandDouble() - 1.0;
        w = x1 * x1 + x2 * x2;
    } while( w>=1.0 || w==0.0 );

    w = sqrt( (-2.0 * log(w)) / w );

    return x1 * w;
}




#endif // _G2O_CUSTOMBUNDLE_RANDOM_H_
