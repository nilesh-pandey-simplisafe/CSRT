/*
Implementation of core CSRT tracker


*/
#pragma once
#ifndef __TRACKERCSRT_HPP__
#define __TRACKERCSRT_HPP__

#include "opencv2/imgproc/types_c.h"
#include "header.hpp"

  struct  Params
  {
    bool use_hog=true;
    bool use_gray=true;
    bool use_channel_weights=true;
    bool use_segmentation=true;

    std::string window_function="hann"; //!<  Window function: "hann", "cheb", "kaiser"
    float template_size=200;
    float gsl_sigma=1.0f;
    float hog_orientations=9;
    float hog_clip=0.2f;
    float padding=3.0f;
    float filter_lr=0.02f;
    float weights_lr=0.02f;
    int num_hog_channels_used=18;
    int admm_iterations=4;
    int histogram_bins=16;
    float histogram_lr=0.04f;
    int background_ratio=2;
    int number_of_scales=33;
    float scale_sigma_factor=0.250f;
    float scale_model_max_area=512.0f;
    float scale_lr=0.025f;
    float scale_step=1.020f;

    float psr_threshold; //!< we lost the target, if the psr is lower than this.
    Params() = default;
  };

class TrackerCSRT_
{

 public:
  TrackerCSRT_(const Params &parameters = Params());     
  ~TrackerCSRT_();

   bool init( Mat image, const Rect2d& boundingBox );

   bool update( Mat image, Rect2d& boundingBox );


  static Ptr<TrackerCSRT_> create(); // what does static means here ?


  Params params;

  bool initImpl(const Mat& image, const Rect2d& boundingBox);
  void setInitialMask(const Mat mask);
  bool updateImpl(const Mat& image, Rect2d& boundingBox);
  void update_csr_filter(const Mat &image, const Mat &my_mask);
  void update_histograms(const Mat &image, const Rect &region);
  void extract_histograms(const Mat &image, cv::Rect region, Histogram &hf, Histogram &hb);
  std::vector<Mat> create_csr_filter(const std::vector<cv::Mat>
          img_features, const cv::Mat Y, const cv::Mat P);
  Mat calculate_response(const Mat &image, const std::vector<Mat> filter);
  Mat get_location_prior(const Rect roi, const Size2f target_size, const Size img_sz);
  Mat segment_region(const Mat &image, const Point2f &object_center,
          const Size2f &template_size, const Size &target_size, float scale_factor);
  Point2f estimate_new_position(const Mat &image);
  std::vector<Mat> get_features(const Mat &patch, const Size2i &feature_size);

  bool isInit;

  // Ptr<TrackerFeatureSet> featureSet;
  // Ptr<TrackerSampler> sampler;

private:
    bool check_mask_area(const Mat &mat, const double obj_area);
    float current_scale_factor;
    Mat window;
    Mat yf;
    Rect2f bounding_box;
    std::vector<Mat> csr_filter;
    std::vector<float> filter_weights;
    Size2f original_target_size;
    Size2i image_size;
    Size2f template_size;
    Size2i rescaled_template_size;
    float rescale_ratio;
    Point2f object_center;
    // DSST dsst;
    Histogram hist_foreground;
    Histogram hist_background;
    double p_b;
    Mat erode_element;
    Mat filter_mask;
    Mat preset_mask;
    Mat default_mask;
    float default_mask_area;
    int cell_size;
};

#endif