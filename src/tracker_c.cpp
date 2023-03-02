#include "tracker_c.h"
extern "C" {

void* createTrackerCSRT() {
    // return reinterpret_cast<TrackerCSRT_>(new TrackerCSRT_());
    return static_cast<void*>(TrackerCSRT_::create());
}

int initTrackerCSRT(void* tracker, void* image, double x, double y, double width, double height) {
    Mat img = *(reinterpret_cast<Mat*>(image));
    Rect2d bbox(x,y,width,height);
    bool success = reinterpret_cast<TrackerCSRT_*>(tracker)->init(img, bbox);
    return success ? 1 : 0;
}

bool updateTrackerCSRT(void* tracker, void* image, double* x, double* y, double* width, double* height) {
    Mat img = *(reinterpret_cast<Mat*>(image));
    Rect2d bbox(x,y,width,height);    
    bool success = reinterpret_cast<TrackerCSRT_*>(tracker)->update(img, bbox);
    if (success) {
        *x = bbox.x;
        *y = bbox.y;
        *width = bbox.width;
        *height = bbox.height;
        return 1;
    }
    return 0;
}

}  // extern "C"
