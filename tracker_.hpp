/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2013, OpenCV Foundation, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

// #include "opencv2/core.hpp"
#include "opencv2/imgproc/types_c.h"
#include "feature_.hpp"

/*
 * Partially based on:
 * ====================================================================================================================
 *   - [AAM] S. Salti, A. Cavallaro, L. Di Stefano, Adaptive Appearance Modeling for Video Tracking: Survey and Evaluation
 *  - [AMVOT] X. Li, W. Hu, C. Shen, Z. Zhang, A. Dick, A. van den Hengel, A Survey of Appearance Models in Visual Object Tracking
 *
 * This Tracking API has been designed with PlantUML. If you modify this API please change UML files under modules/tracking/doc/uml
 *
 */

using namespace cv;


//! @addtogroup tracking
//! @{

/************************************ TrackerFeature Base Classes ************************************/

/** @brief Abstract base class for TrackerFeature that represents the feature.
 */
class  TrackerFeature
{
 public:
  virtual ~TrackerFeature();

  /** @brief Compute the features in the images collection
    @param images The images
    @param response The output response
     */
  void compute( const std::vector<Mat>& images, Mat& response );

  /** @brief Create TrackerFeature by tracker feature type
    @param trackerFeatureType The TrackerFeature name

    The modes available now:

    -   "HAAR" -- Haar Feature-based

    The modes that will be available soon:

    -   "HOG" -- Histogram of Oriented Gradients features
    -   "LBP" -- Local Binary Pattern features
    -   "FEATURE2D" -- All types of Feature2D
     */
  static Ptr<TrackerFeature> create( const String& trackerFeatureType );

  /** @brief Identify most effective features
    @param response Collection of response for the specific TrackerFeature
    @param npoints Max number of features

    @note This method modifies the response parameter
     */
  virtual void selection( Mat& response, int npoints ) = 0;

  /** @brief Get the name of the specific TrackerFeature
     */
  String getClassName() const;

 protected:

  virtual bool computeImpl( const std::vector<Mat>& images, Mat& response ) = 0;

  String className;
};

/** @brief Class that manages the extraction and selection of features

@cite AAM Feature Extraction and Feature Set Refinement (Feature Processing and Feature Selection).
See table I and section III C @cite AMVOT Appearance modelling -\> Visual representation (Table II,
section 3.1 - 3.2)

TrackerFeatureSet is an aggregation of TrackerFeature

@sa
   TrackerFeature

 */
class  TrackerFeatureSet
{
 public:

  TrackerFeatureSet();

  ~TrackerFeatureSet();

  /** @brief Extract features from the images collection
    @param images The input images
     */
  void extraction( const std::vector<Mat>& images );

  /** @brief Identify most effective features for all feature types (optional)
     */
  void selection();

  /** @brief Remove outliers for all feature types (optional)
     */
  void removeOutliers();

  /** @brief Add TrackerFeature in the collection. Return true if TrackerFeature is added, false otherwise
    @param trackerFeatureType The TrackerFeature name

    The modes available now:

    -   "HAAR" -- Haar Feature-based

    The modes that will be available soon:

    -   "HOG" -- Histogram of Oriented Gradients features
    -   "LBP" -- Local Binary Pattern features
    -   "FEATURE2D" -- All types of Feature2D

    Example TrackerFeatureSet::addTrackerFeature : :
    @code
        //sample usage:

        Ptr<TrackerFeature> trackerFeature = new TrackerFeatureHAAR( HAARparameters );
        featureSet->addTrackerFeature( trackerFeature );

        //or add CSC sampler with default parameters
        //featureSet->addTrackerFeature( "HAAR" );
    @endcode
    @note If you use the second method, you must initialize the TrackerFeature
     */
  bool addTrackerFeature( String trackerFeatureType );

  /** @overload
    @param feature The TrackerFeature class
    */
  bool addTrackerFeature( Ptr<TrackerFeature>& feature );

  /** @brief Get the TrackerFeature collection (TrackerFeature name, TrackerFeature pointer)
     */
  const std::vector<std::pair<String, Ptr<TrackerFeature> > >& getTrackerFeature() const;

  /** @brief Get the responses

    @note Be sure to call extraction before getResponses Example TrackerFeatureSet::getResponses : :
     */
  const std::vector<Mat>& getResponses() const;

 private:

  void clearResponses();
  bool blockAddTrackerFeature;

  std::vector<std::pair<String, Ptr<TrackerFeature> > > features;  //list of features
  std::vector<Mat> responses;        //list of response after compute

};

/************************************ TrackerSampler Base Classes ************************************/

/** @brief Abstract base class for TrackerSamplerAlgorithm that represents the algorithm for the specific
sampler.
 */
class  TrackerSamplerAlgorithm
{
 public:
  /**
   * \brief Destructor
   */
  virtual ~TrackerSamplerAlgorithm();

  /** @brief Create TrackerSamplerAlgorithm by tracker sampler type.
    @param trackerSamplerType The trackerSamplerType name

    The modes available now:

    -   "CSC" -- Current State Center
    -   "CS" -- Current State
     */
  static Ptr<TrackerSamplerAlgorithm> create( const String& trackerSamplerType );

  /** @brief Computes the regions starting from a position in an image.

    Return true if samples are computed, false otherwise

    @param image The current frame
    @param boundingBox The bounding box from which regions can be calculated

    @param sample The computed samples @cite AAM Fig. 1 variable Sk
     */
  bool sampling( const Mat& image, Rect boundingBox, std::vector<Mat>& sample );

  /** @brief Get the name of the specific TrackerSamplerAlgorithm
    */
  String getClassName() const;

 protected:
  String className;

  virtual bool samplingImpl( const Mat& image, Rect boundingBox, std::vector<Mat>& sample ) = 0;
};

/**
 * \brief Class that manages the sampler in order to select regions for the update the model of the tracker
 * [AAM] Sampling e Labeling. See table I and section III B
 */

/** @brief Class that manages the sampler in order to select regions for the update the model of the tracker

@cite AAM Sampling e Labeling. See table I and section III B

TrackerSampler is an aggregation of TrackerSamplerAlgorithm
@sa
   TrackerSamplerAlgorithm
 */
class  TrackerSampler
{
 public:

  /**
   * \brief Constructor
   */
  TrackerSampler();

  /**
   * \brief Destructor
   */
  ~TrackerSampler();

  /** @brief Computes the regions starting from a position in an image
    @param image The current frame
    @param boundingBox The bounding box from which regions can be calculated
     */
  void sampling( const Mat& image, Rect boundingBox );

  /** @brief Return the collection of the TrackerSamplerAlgorithm
    */
  const std::vector<std::pair<String, Ptr<TrackerSamplerAlgorithm> > >& getSamplers() const;

  /** @brief Return the samples from all TrackerSamplerAlgorithm, @cite AAM Fig. 1 variable Sk
    */
  const std::vector<Mat>& getSamples() const;

  /** @brief Add TrackerSamplerAlgorithm in the collection. Return true if sampler is added, false otherwise
    @param trackerSamplerAlgorithmType The TrackerSamplerAlgorithm name

    The modes available now:
    -   "CSC" -- Current State Center
    -   "CS" -- Current State
    -   "PF" -- Particle Filtering

    Example TrackerSamplerAlgorithm::addTrackerSamplerAlgorithm : :
    @code
         TrackerSamplerCSC::Params CSCparameters;
         Ptr<TrackerSamplerAlgorithm> CSCSampler = new TrackerSamplerCSC( CSCparameters );

         if( !sampler->addTrackerSamplerAlgorithm( CSCSampler ) )
           return false;

         //or add CSC sampler with default parameters
         //sampler->addTrackerSamplerAlgorithm( "CSC" );
    @endcode
    @note If you use the second method, you must initialize the TrackerSamplerAlgorithm
     */
  bool addTrackerSamplerAlgorithm( String trackerSamplerAlgorithmType );

  /** @overload
    @param sampler The TrackerSamplerAlgorithm
    */
  bool addTrackerSamplerAlgorithm( Ptr<TrackerSamplerAlgorithm>& sampler );

 private:
  std::vector<std::pair<String, Ptr<TrackerSamplerAlgorithm> > > samplers;
  std::vector<Mat> samples;
  bool blockAddTrackerSampler;

  void clearSamples();
};

/************************************ TrackerModel Base Classes ************************************/

/** @brief Abstract base class for TrackerTargetState that represents a possible state of the target.

See @cite AAM \f$\hat{x}^{i}_{k}\f$ all the states candidates.

Inherits this class with your Target state, In own implementation you can add scale variation,
width, height, orientation, etc.
 */
class  TrackerTargetState
{
 public:
  virtual ~TrackerTargetState()
  {
  }
  ;
  /**
   * \brief Get the position
   * \return The position
   */
  Point2f getTargetPosition() const;

  /**
   * \brief Set the position
   * \param position The position
   */
  void setTargetPosition( const Point2f& position );
  /**
   * \brief Get the width of the target
   * \return The width of the target
   */
  int getTargetWidth() const;

  /**
   * \brief Set the width of the target
   * \param width The width of the target
   */
  void setTargetWidth( int width );
  /**
   * \brief Get the height of the target
   * \return The height of the target
   */
  int getTargetHeight() const;

  /**
   * \brief Set the height of the target
   * \param height The height of the target
   */
  void setTargetHeight( int height );

 protected:
  Point2f targetPosition;
  int targetWidth;
  int targetHeight;

};

/** @brief Represents the model of the target at frame \f$k\f$ (all states and scores)

See @cite AAM The set of the pair \f$\langle \hat{x}^{i}_{k}, C^{i}_{k} \rangle\f$
@sa TrackerTargetState
 */
typedef std::vector<std::pair<Ptr<TrackerTargetState>, float> > ConfidenceMap;

/** @brief Represents the estimate states for all frames

@cite AAM \f$x_{k}\f$ is the trajectory of the target up to time \f$k\f$

@sa TrackerTargetState
 */
typedef std::vector<Ptr<TrackerTargetState> > Trajectory;

/** @brief Abstract base class for TrackerStateEstimator that estimates the most likely target state.

See @cite AAM State estimator

See @cite AMVOT Statistical modeling (Fig. 3), Table III (generative) - IV (discriminative) - V (hybrid)
 */
class  TrackerStateEstimator
{
 public:
  virtual ~TrackerStateEstimator();

  /** @brief Estimate the most likely target state, return the estimated state
    @param confidenceMaps The overall appearance model as a list of :cConfidenceMap
     */
  Ptr<TrackerTargetState> estimate( const std::vector<ConfidenceMap>& confidenceMaps );

  /** @brief Update the ConfidenceMap with the scores
    @param confidenceMaps The overall appearance model as a list of :cConfidenceMap
     */
  void update( std::vector<ConfidenceMap>& confidenceMaps );

  /** @brief Create TrackerStateEstimator by tracker state estimator type
    @param trackeStateEstimatorType The TrackerStateEstimator name

    The modes available now:

    -   "BOOSTING" -- Boosting-based discriminative appearance models. See @cite AMVOT section 4.4

    The modes available soon:

    -   "SVM" -- SVM-based discriminative appearance models. See @cite AMVOT section 4.5
     */

  /** @brief Get the name of the specific TrackerStateEstimator
     */
  String getClassName() const;

 protected:

  virtual Ptr<TrackerTargetState> estimateImpl( const std::vector<ConfidenceMap>& confidenceMaps ) = 0;
  virtual void updateImpl( std::vector<ConfidenceMap>& confidenceMaps ) = 0;
  String className;
};


/** @brief Abstract class that represents the model of the target. It must be instantiated by specialized
tracker

See @cite AAM Ak

Inherits this with your TrackerModel
 */
class  TrackerModel
{
 public:

  /**
   * \brief Constructor
   */
  TrackerModel();

  /**
   * \brief Destructor
   */
  virtual ~TrackerModel();

 protected:
  std::vector<ConfidenceMap> confidenceMaps;
  ConfidenceMap currentConfidenceMap;
  Trajectory trajectory;
  int maxCMLength;

  virtual void modelEstimationImpl( const std::vector<Mat>& responses ) = 0;
  virtual void modelUpdateImpl() = 0;

};

/************************************ Tracker Base Class ************************************/

/** @brief Base abstract class for the long-term tracker:
 */
class Tracker : public Algorithm
{
 public:

  virtual ~Tracker() CV_OVERRIDE;

  /** @brief Initialize the tracker with a known bounding box that surrounds the target
    @param image The initial frame
    @param boundingBox The initial bounding box

    @return True if initialization was successful, false otherwise
     */
   bool init( InputArray image, const Rect2d& boundingBox );

  /** @brief Update the tracker, find the new most likely bounding box for the target
    @param image The current frame
    @param boundingBox The bounding box that represents the new target location, if true was returned, not
    modified otherwise

    @return True if the target was located, false otherwise. Note that the latter does not imply that the tracker has failed; the target may not be visible in the current frame.
     */
   bool update( InputArray image, CV_OUT Rect2d& boundingBox );

  virtual void read( const FileNode& fn ) CV_OVERRIDE = 0;
  virtual void write( FileStorage& fs ) const CV_OVERRIDE = 0;

 protected:

  virtual bool initImpl( const Mat& image, const Rect2d& boundingBox ) = 0;
  virtual bool updateImpl( const Mat& image, Rect2d& boundingBox ) = 0;

  bool isInit;

  Ptr<TrackerFeatureSet> featureSet;
  Ptr<TrackerSampler> sampler;
  Ptr<TrackerModel> model;
};


class  TrackerSamplerCSC : public TrackerSamplerAlgorithm
{
 public:
  enum
  {
    MODE_INIT_POS = 1,  //!< mode for init positive samples
    MODE_INIT_NEG = 2,  //!< mode for init negative samples
    MODE_TRACK_POS = 3,  //!< mode for update positive samples
    MODE_TRACK_NEG = 4,  //!< mode for update negative samples
    MODE_DETECT = 5   //!< mode for detect samples
  };

  struct  Params
  {
    Params();
    float initInRad;        //!< radius for gathering positive instances during init
    float trackInPosRad;    //!< radius for gathering positive instances during tracking
    float searchWinSize;  //!< size of search window
    int initMaxNegNum;      //!< # negative samples to use during init
    int trackMaxPosNum;     //!< # positive samples to use during training
    int trackMaxNegNum;     //!< # negative samples to use during training
  };

  /** @brief Constructor
    @param parameters TrackerSamplerCSC parameters TrackerSamplerCSC::Params
     */
  TrackerSamplerCSC( const TrackerSamplerCSC::Params &parameters = TrackerSamplerCSC::Params() );

  /** @brief Set the sampling mode of TrackerSamplerCSC
    @param samplingMode The sampling mode

    The modes are:

    -   "MODE_INIT_POS = 1" -- for the positive sampling in initialization step
    -   "MODE_INIT_NEG = 2" -- for the negative sampling in initialization step
    -   "MODE_TRACK_POS = 3" -- for the positive sampling in update step
    -   "MODE_TRACK_NEG = 4" -- for the negative sampling in update step
    -   "MODE_DETECT = 5" -- for the sampling in detection step
     */
  void setMode( int samplingMode );

  ~TrackerSamplerCSC();

 protected:

  bool samplingImpl( const Mat& image, Rect boundingBox, std::vector<Mat>& sample ) CV_OVERRIDE;

 private:

  Params params;
  int mode;
  RNG rng;

  std::vector<Mat> sampleImage( const Mat& img, int x, int y, int w, int h, float inrad, float outrad = 0, int maxnum = 1000000 );
};

/** @brief TrackerSampler based on CS (current state), used by algorithm TrackerBoosting
 */
class  TrackerSamplerCS : public TrackerSamplerAlgorithm
{
 public:
  enum
  {
    MODE_POSITIVE = 1,  //!< mode for positive samples
    MODE_NEGATIVE = 2,  //!< mode for negative samples
    MODE_CLASSIFY = 3  //!< mode for classify samples
  };

  struct  Params
  {
    Params();
    float overlap;  //!<overlapping for the search windows
    float searchFactor;  //!<search region parameter
  };
  /** @brief Constructor
    @param parameters TrackerSamplerCS parameters TrackerSamplerCS::Params
     */
  TrackerSamplerCS( const TrackerSamplerCS::Params &parameters = TrackerSamplerCS::Params() );

  /** @brief Set the sampling mode of TrackerSamplerCS
    @param samplingMode The sampling mode

    The modes are:

    -   "MODE_POSITIVE = 1" -- for the positive sampling
    -   "MODE_NEGATIVE = 2" -- for the negative sampling
    -   "MODE_CLASSIFY = 3" -- for the sampling in classification step
     */
  void setMode( int samplingMode );

  ~TrackerSamplerCS();

  bool samplingImpl( const Mat& image, Rect boundingBox, std::vector<Mat>& sample ) CV_OVERRIDE;
  Rect getROI() const;
 private:
  Rect getTrackingROI( float searchFactor );
  Rect RectMultiply( const Rect & rect, float f );
  std::vector<Mat> patchesRegularScan( const Mat& image, Rect trackingROI, Size patchSize );
  void setCheckedROI( Rect imageROI );

  Params params;
  int mode;
  Rect trackedPatch;
  Rect validROI;
  Rect ROI;

};


/************************************ Specific TrackerFeature Classes ************************************/

/**
 * \brief TrackerFeature based on Feature2D
 */
class  TrackerFeatureFeature2d : public TrackerFeature
{
 public:

  /**
   * \brief Constructor
   * \param detectorType string of FeatureDetector
   * \param descriptorType string of DescriptorExtractor
   */
  TrackerFeatureFeature2d( String detectorType, String descriptorType );

  ~TrackerFeatureFeature2d() CV_OVERRIDE;

  void selection( Mat& response, int npoints ) CV_OVERRIDE;

 protected:

  bool computeImpl( const std::vector<Mat>& images, Mat& response ) CV_OVERRIDE;

 private:

  std::vector<KeyPoint> keypoints;
};

/**
 * \brief TrackerFeature based on HOG
 */
class  TrackerFeatureHOG : public TrackerFeature
{
 public:

  TrackerFeatureHOG();

  ~TrackerFeatureHOG() CV_OVERRIDE;

  void selection( Mat& response, int npoints ) CV_OVERRIDE;

 protected:

  bool computeImpl( const std::vector<Mat>& images, Mat& response ) CV_OVERRIDE;

};


class MultiTracker : public Algorithm
{
public:

  /**
  * \brief Constructor.
  */
   MultiTracker();

  /**
  * \brief Destructor
  */
  ~MultiTracker() CV_OVERRIDE;

  /**
  * \brief Add a new object to be tracked.
  *
  * @param newTracker tracking algorithm to be used
  * @param image input image
  * @param boundingBox a rectangle represents ROI of the tracked object
  */
   bool add(Ptr<Tracker> newTracker, InputArray image, const Rect2d& boundingBox);

  /**
  * \brief Add a set of objects to be tracked.
  * @param newTrackers list of tracking algorithms to be used
  * @param image input image
  * @param boundingBox list of the tracked objects
  */
  bool add(std::vector<Ptr<Tracker> > newTrackers, InputArray image, std::vector<Rect2d> boundingBox);

  /**
  * \brief Update the current tracking status.
  * The result will be saved in the internal storage.
  * @param image input image
  */
  bool update(InputArray image);

  /**
  * \brief Update the current tracking status.
  * @param image input image
  * @param boundingBox the tracking result, represent a list of ROIs of the tracked objects.
  */
   bool update(InputArray image, CV_OUT std::vector<Rect2d> & boundingBox);

  /**
  * \brief Returns a reference to a storage for the tracked objects, each object corresponds to one tracker algorithm
  */
   const std::vector<Rect2d>& getObjects() const;

  /**
  * \brief Returns a pointer to a new instance of MultiTracker
  */
   static Ptr<MultiTracker> create();

protected:
  //!<  storage for the tracker algorithms.
  std::vector< Ptr<Tracker> > trackerList;

  //!<  storage for the tracked objects, each object corresponds to one tracker algorithm.
  std::vector<Rect2d> objects;
};

/************************************ Multi-Tracker Classes ---By Tyan Vladimir---************************************/

/** @brief Base abstract class for the long-term Multi Object Trackers:

@sa Tracker, MultiTrackerTLD
*/
class  MultiTracker_Alt
{
public:
  /** @brief Constructor for Multitracker
  */
  MultiTracker_Alt()
  {
    targetNum = 0;
  }

  /** @brief Add a new target to a tracking-list and initialize the tracker with a known bounding box that surrounded the target
  @param image The initial frame
  @param boundingBox The initial bounding box of target
  @param tracker_algorithm Multi-tracker algorithm

  @return True if new target initialization went succesfully, false otherwise
  */
  bool addTarget(InputArray image, const Rect2d& boundingBox, Ptr<Tracker> tracker_algorithm);

  /** @brief Update all trackers from the tracking-list, find a new most likely bounding boxes for the targets
  @param image The current frame

  @return True means that all targets were located and false means that tracker couldn't locate one of the targets in
  current frame. Note, that latter *does not* imply that tracker has failed, maybe target is indeed
  missing from the frame (say, out of sight)
  */
  bool update(InputArray image);

  /** @brief Current number of targets in tracking-list
  */
  int targetNum;

  /** @brief Trackers list for Multi-Object-Tracker
  */
  std::vector <Ptr<Tracker> > trackers;

  /** @brief Bounding Boxes list for Multi-Object-Tracker
  */
  std::vector <Rect2d> boundingBoxes;
  /** @brief List of randomly generated colors for bounding boxes display
  */
  std::vector<Scalar> colors;
};


/*********************************** CSRT ************************************/
/** @brief the CSRT tracker

The implementation is based on @cite Lukezic_IJCV2018 Discriminative Correlation Filter with Channel and Spatial Reliability
*/
class TrackerCSRT : public Tracker
{
public:
  struct  Params
  {
    /**
    * \brief Constructor
    */
    Params();

    /**
    * \brief Read parameters from a file
    */
    void read(const FileNode& /*fn*/);

    /**
    * \brief Write parameters to a file
    */
    void write(cv::FileStorage& fs) const;

    bool use_hog;
    bool use_color_names;
    bool use_gray;
    bool use_rgb;
    bool use_channel_weights;
    bool use_segmentation;

    std::string window_function; //!<  Window function: "hann", "cheb", "kaiser"
    float kaiser_alpha;
    float cheb_attenuation;

    float template_size;
    float gsl_sigma;
    float hog_orientations;
    float hog_clip;
    float padding;
    float filter_lr;
    float weights_lr;
    int num_hog_channels_used;
    int admm_iterations;
    int histogram_bins;
    float histogram_lr;
    int background_ratio;
    int number_of_scales;
    float scale_sigma_factor;
    float scale_model_max_area;
    float scale_lr;
    float scale_step;

    float psr_threshold; //!< we lost the target, if the psr is lower than this.
  };

  /** @brief Constructor
  @param parameters CSRT parameters TrackerCSRT::Params
  */
  static Ptr<TrackerCSRT> create(const TrackerCSRT::Params &parameters);

   static Ptr<TrackerCSRT> create();

  virtual void setInitialMask(const Mat mask) = 0;

  virtual ~TrackerCSRT() CV_OVERRIDE {}
};

//! @}
 /* namespace cvx */

#endif
