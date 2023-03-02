#include "tracker_c.h"


void splitstring(const char* str, char separator, char** vec, int max_size)
{
    char* token;
    char* remainder;
    int i = 0;
    remainder = (char*)str;
    while ((token = strtok_r(remainder, &separator, &remainder)))
    {
        if (i >= max_size) break;
        vec[i] = token;
        i++;
    }
}


int main(int argc, char** argv)
{
    bool show_images   = false;
    bool save_video    = true;
    bool resize_imshow = false;
    bool debug         = true;

    // show help
    if (argc<2) {
        printf(
            " Usage: example_tracking_csrt <video_name>\n"
            " examples:\n"
            " example_tracking_csrt Bolt/img/%04.jpg\n"
            " example_tracking_csrt Bolt/img/%04.jpg Bolt/grouondtruth.txt\n"
            " example_tracking_csrt faceocc2.webm\n");
        return 0;
    }

    // create the tracker
    void* tracker = createTrackerCSRT();
    // set input video
    char* video = argv[1];
    char* ouput_path = argv[2];
    CvCapture* cap = cvCreateFileCapture(video);
    // and read first frame

    // and read first frame
    IplImage* ipl_frame = cvQueryFrame(cap);

    // target bounding box
    CvRect roi;
    if (argc > 3) {
        // read first line of ground-truth file
        char* groundtruthPath = argv[4];
        FILE *gtfile = fopen(groundtruthPath,"r")
        char[256] gtLine;
        fgets(gtLine, 256, gtFile);
        fclose(gtFile);

        // parse the line by elements
        char *element;
        int elements[1000];
        int i = 0;
        element = strtok(line, ",");
        while (element != NULL)
        {
            element_value = atoi(element);
            elements[i] = (int)element_value;
            i++;
            element = strtok(NULL, ",");
        }


        if (i == 4) {
            // ground-truth is rectangle
            double x = elements[0];
            double y = elements[1];
            double width = elements[2];
            double height = elements[3];
        }
        else {
            printf("Number of ground-truth elements is not 4 or 8.\n");
        }

    }
    else {
    if (argc == 3)
    {
        roi = cvSelectROI("tracker", frame, true, false);
        printf("ROI: x=%d y=%d w=%d h=%d\n", roi.x, roi.y, roi.width, roi.height);
    }

    }

    //quit if ROI was not selected
    if (roi.width == 0 || roi.height == 0)
        return 0;

    // initialize the tracker
    // int64 t1 = cv::getTickCount();
    tracker->init(frame, roi);
    // int64 t2 = cv::getTickCount();
    // int64 tick_counter = t2 - t1;

    IplImage* imshow_mat;

    int frame_width = (int)cvGetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH);
    int frame_height = (int)cvGetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT);
    CvVideoWriter* video_out = cvCreateVideoWriter(output_path, CV_FOURCC('M','J','P','G'), 30, cvSize(frame_width, frame_height), 1);
    
    // do the tracking
    printf("Start the tracking process, press ESC to quit.\n");
    int frame_idx = 1;
    while ((frame = cvQueryFrame(cap)) != NULL) {        // get frame from the video

        // stop the program if no more images
        if (frame.rows == 0 || frame.cols == 0)
            break;

        // update the tracking result
        bool isfound = tracker->update(frame, roi);
        frame_idx++;

	if (save_video)
        {
            pt1 = cvPoint(roi.x, roi.y);
            pt2 = cvPoint(roi.x + roi.width, roi.y + roi.height);
            cvRectangle(frame, pt1, pt2, CV_RGB(255, 0, 0), 5, 1, 0);
            cvWriteFrame(video_out, frame);
        }

        //quit on ESC button
        if (cvWaitKey(1) == 27)break;
    }

    // cout << "Elapsed sec: " << static_cast<double>(tick_counter) / cv::getTickFrequency() << endl;
    // cout << "FPS: " << ((double)(frame_idx)) / (static_cast<double>(tick_counter) / cv::getTickFrequency()) << endl;
}
