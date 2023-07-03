//
// Created by Kurosu Chan on 2023/6/25.
//

#ifndef SIMPLE_CNL_DEF_H
#define SIMPLE_CNL_DEF_H

#include <cnl/all.h>

// https://chummersone.github.io/qformat.html#
// used to store speed...
// 0 - 256 with 0.004 precision is more than enough
using fixed_8_8 = cnl::scaled_integer<uint16_t, cnl::power<-8>>;
using fixed_16_16 = cnl::scaled_integer<uint32_t, cnl::power<-16>>;
using fixed_24_8 = cnl::scaled_integer<uint32_t, cnl::power<-8>>;

#endif // SIMPLE_CNL_DEF_H
