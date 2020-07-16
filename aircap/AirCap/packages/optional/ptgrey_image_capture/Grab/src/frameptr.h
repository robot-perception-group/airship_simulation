/*
 * TODO: Add apropriate License and copyright header
 */

#ifndef FRAMEPTR_H
#define FRAMEPTR_H

#include "boost/thread/mutex.hpp"

namespace videograb {
class frameptr {
public:
   int64_t number;
   uint8_t * buffer;
   boost::posix_time::ptime timestamp;
};
}


#endif // ifndef FRAMEPTR_H
