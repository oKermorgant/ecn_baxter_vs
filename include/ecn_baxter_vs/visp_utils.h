#ifndef ECN_VISP_UTILS
#define ECN_VISP_UTILS

#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>

namespace ecn
{
inline void putAt(vpMatrix &_J, const vpMatrix &_Jsub, const unsigned int r, const unsigned int c)
{
    vpSubMatrix Js(_J, r, c, _Jsub.getRows(), _Jsub.getCols());
    Js = _Jsub;
}

// put a vector inside another
inline void putAt(vpColVector &_e, const vpColVector &_esub, const unsigned int r)
{
    vpSubColVector es(_e, r, _esub.getRows());
    es = _esub;
}
}

#endif
