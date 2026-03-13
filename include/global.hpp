#include "lidar.hpp"

// toggle for debug printf
#define DEBUG_LOG true
#if DEBUG_LOG
#include <stdio.h>
#define DBG(...) printf(__VA_ARGS__)
#else
#define DBG(...)
#endif

// inline makes it global variable
// inline LidarParser lidar_parser;