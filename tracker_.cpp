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

// #include "precomp.hpp"

#include "header.hpp"
using namespace cv;




//--------------------------------------------------------------------------------




/*
 *  Tracker
 */
TrackerFeature::~TrackerFeature()
{

}

void TrackerFeature::compute( const std::vector<Mat>& images, Mat& response )
{
  if( images.empty() )
    return;

  computeImpl( images, response );
}

Ptr<TrackerFeature> TrackerFeature::create( const String& trackerFeatureType )
{
  if( trackerFeatureType.find( "FEATURE2D" ) == 0 )
  {
    size_t firstSep = trackerFeatureType.find_first_of('.');
    size_t secondSep = trackerFeatureType.find_last_of('.');

    String detector = trackerFeatureType.substr( firstSep, secondSep - firstSep );
    String descriptor = trackerFeatureType.substr( secondSep, trackerFeatureType.length() - secondSep );

    return Ptr<TrackerFeatureFeature2d>( new TrackerFeatureFeature2d( detector, descriptor ) );
  }

  if( trackerFeatureType.find( "HOG" ) == 0 )
  {
    return Ptr<TrackerFeatureHOG>( new TrackerFeatureHOG() );
  }


  CV_Error( -1, "Tracker feature type not supported" );
  return Ptr<TrackerFeature>();
}

String TrackerFeature::getClassName() const
{
  return className;
}

/**
 * TrackerFeatureFeature2d
 */
TrackerFeatureFeature2d::TrackerFeatureFeature2d( String /*detectorType*/, String /*descriptorType*/)
{
  className = "FEATURE2D";
}

TrackerFeatureFeature2d::~TrackerFeatureFeature2d()
{

}

bool TrackerFeatureFeature2d::computeImpl( const std::vector<Mat>& /*images*/, Mat& /*response*/)
{
  return false;
}

void TrackerFeatureFeature2d::selection( Mat& /*response*/, int /*npoints*/)
{

}

/**
 * TrackerFeatureHOG
 */
TrackerFeatureHOG::TrackerFeatureHOG()
{
  className = "HOG";
}

TrackerFeatureHOG::~TrackerFeatureHOG()
{

}

bool TrackerFeatureHOG::computeImpl( const std::vector<Mat>& /*images*/, Mat& /*response*/)
{
  return false;
}

void TrackerFeatureHOG::selection( Mat& /*response*/, int /*npoints*/)
{

}

/**
 * TrackerFeatureHAAR
 */

/**
 * Parameters
 */




/**
 * TrackerFeatureLBP
 */


 /* namespace cv */

TrackerFeatureSet::TrackerFeatureSet()
{
  blockAddTrackerFeature = false;
}

/*
 * Destructor
 */
TrackerFeatureSet::~TrackerFeatureSet()
{

}

void TrackerFeatureSet::extraction( const std::vector<Mat>& images )
{

  clearResponses();
  responses.resize( features.size() );

  for ( size_t i = 0; i < features.size(); i++ )
  {
    Mat response;
    features[i].second->compute( images, response );
    responses[i] = response;
  }

  if( !blockAddTrackerFeature )
  {
    blockAddTrackerFeature = true;
  }
}

void TrackerFeatureSet::selection()
{

}

void TrackerFeatureSet::removeOutliers()
{

}

bool TrackerFeatureSet::addTrackerFeature( String trackerFeatureType )
{
  if( blockAddTrackerFeature )
  {
    return false;
  }
  Ptr<TrackerFeature> feature = TrackerFeature::create( trackerFeatureType );

  if( feature == 0 )
  {
    return false;
  }

  features.push_back( std::make_pair( trackerFeatureType, feature ) );

  return true;
}

bool TrackerFeatureSet::addTrackerFeature( Ptr<TrackerFeature>& feature )
{
  if( blockAddTrackerFeature )
  {
    return false;
  }

  String trackerFeatureType = feature->getClassName();
  features.push_back( std::make_pair( trackerFeatureType, feature ) );

  return true;
}

const std::vector<std::pair<String, Ptr<TrackerFeature> > >& TrackerFeatureSet::getTrackerFeature() const
{
  return features;
}

const std::vector<Mat>& TrackerFeatureSet::getResponses() const
{
  return responses;
}

void TrackerFeatureSet::clearResponses()
{
  responses.clear();
}

 /* namespace cv */


Tracker::~Tracker()
{
}

bool Tracker::init( InputArray image, const Rect2d& boundingBox )
{

  if( isInit )
  {
    return false;
  }

  if( image.empty() )
    return false;

  sampler = Ptr<TrackerSampler>( new TrackerSampler() );
  featureSet = Ptr<TrackerFeatureSet>( new TrackerFeatureSet() );
  model = Ptr<TrackerModel>();

  bool initTracker = initImpl( image.getMat(), boundingBox );

  //check if the model component is initialized
  if( model == 0 )
  {
    CV_Error( -1, "The model is not initialized" );
  }

  if( initTracker )
  {
    isInit = true;
  }

  return initTracker;
}

bool Tracker::update( InputArray image, Rect2d& boundingBox )
{

  if( !isInit )
  {
    return false;
  }

  if( image.empty() )
    return false;

  return updateImpl( image.getMat(), boundingBox );
}
