#include "arucoDetector.hpp"
#include <iostream>
#include <sensor_msgs/msg/detail/image__struct.hpp>


/**
 * @brief Initializes the ArucoDetector object with default parameters.
 *
 * This constructor initializes the ArucoDetector object with default parameters for detecting Aruco markers.
 * The minimum marker perimeter rate is set to 0.1, and the predefined Aruco dictionary is used.
 *
 * @see cv::aruco::DetectorParameters::create
 * @see cv::aruco::getPredefinedDictionary
 */
ArucoDetector::ArucoDetector() 
{
  detectorParams_ = cv::aruco::DetectorParameters::create();
  detectorParams_->minMarkerPerimeterRate = 0.1;
  dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
}


void ArucoDetector::detect(const sensor_msgs::msg::Image::ConstSharedPtr &img)
{
  try {
    mCvPtr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    return;
  }
  currentFrame_ = mCvPtr_->image;
  cv::aruco::detectMarkers(currentFrame_, dict_, markerCorners_, markerIds_,
                           detectorParams_);
}


const sensor_msgs::msg::Image::ConstSharedPtr
ArucoDetector::getFrameAsImgMsg() 
{
  return mCvPtr_->toImageMsg();
}

