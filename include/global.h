#include "lidar.hpp"

// toggle for debug printf
#define DEBUG_PARSE false
#if DEBUG_PARSE
#define DBG(...) printf(__VA_ARGS__)
#else
#define DBG(...)
#endif

// inline makes it global variable
inline LidarParser lidar_parser;