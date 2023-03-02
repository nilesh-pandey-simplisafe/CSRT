#ifndef TRACKER_C_H
#define TRACKER_C_H

#include "opencv2/core/types_c.h"
#include <stdbool.h>
#include "trackerCSRTm.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CvRect2f {
    float x;
    float y;
    float width;
    float height;
} CvRect2f;

void* createTrackerCSRT();
int initTrackerCSRT(void* tracker, void* image, double x, double y, double width, double height);
int updateTrackerCSRT(void* tracker, void* image, double* x, double* y, double* width, double* height);

#ifdef __cplusplus
}
#endif

#endif // TRACKER_C_H
