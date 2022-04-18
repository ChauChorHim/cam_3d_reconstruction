#ifndef _GPS2UTM_H_
#define _GPS2UTM_H_

#include "utm.h"

namespace aw {
namespace coord {

void GPS2UTM(double latitude, double longitude, double altitude,
             double* utm_east, double* utm_north, double* utm_up) {
    char utm_zone_buf[64] = "";
    LLtoUTM(latitude, longitude, *utm_north, *utm_east, utm_zone_buf);
    *utm_up = altitude;
}

}  // namespace coord
}  // namespace aw

#endif // _GPS2UTM_H_
