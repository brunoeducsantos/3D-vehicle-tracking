#include <numeric>
#include "matching2D.hpp"

using namespace std;
using namespace cv;
// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        //int normType = cv::NORM_HAMMING;
        int normType= cv::NORM_L1;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
          matcher= DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        // implement k-nearest-neighbor matching
        double t = (double)cv::getTickCount();
        std::vector< std::vector<DMatch> > knn_matches;
        matcher->knnMatch( descSource, descRef, knn_matches,2 );

        // filter matches using descriptor distance ratio test
        for (size_t i = 0; i < knn_matches.size(); i++){
            if (knn_matches[i][0].distance < 0.8f * knn_matches[i][1].distance){
                matches.push_back(knn_matches[i][0]);
            }
        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    if(descriptorType.compare("ORB") == 0){
        extractor= cv::ORB::create();
    }
    if(descriptorType.compare("FREAK") == 0){
        extractor= cv::xfeatures2d::FREAK::create();
    }
    if(descriptorType.compare("AKAZE") == 0){
        extractor= cv::AKAZE::create();
    }
    if(descriptorType.compare("SIFT") == 0){
        int  nfeatures=0;
        int nOctaveLayers = 3;
        double contrastThreshold=0.04; 
        double edgeThreshold=10; 
        double sigma=1.6;
        extractor = cv::xfeatures2d::SIFT::create(nfeatures, nOctaveLayers, contrastThreshold,  edgeThreshold, sigma);
    }
    if(descriptorType.compare("BRIEF") == 0){
        extractor= cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    int nb_detected_points=100;
    double t = (double)cv::getTickCount();
    if(detectorType.compare("FAST")== 0){
        cv::Ptr<cv::FastFeatureDetector> detector= cv::FastFeatureDetector::create(nb_detected_points, true);
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "FAST detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }
    
    if(detectorType.compare("BRISK")== 0){
        //Check parameters
        cv::Ptr<cv::BRISK> detector= cv::BRISK::create();
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "BRISK detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }
    if(detectorType.compare("ORB")== 0){
        cv::Ptr<cv::ORB> detector= cv::ORB::create(3000);
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "ORB detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }

    if(detectorType.compare("SIFT")== 0){
        cv::Ptr<cv::Feature2D> detector= SIFT::create();
        detector->detect(img, keypoints); 
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "SIFT detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
  
    }
    if(detectorType.compare("AKAZE")== 0){
        cv::Ptr<cv::AKAZE> detector= cv::AKAZE::create();
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "AKAZE detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    }
      
    
     // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}


void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis){
      double t = (double)cv::getTickCount();
    // Detector parameters
        int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
        int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
        int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
        double k = 0.04;       // Harris parameter (see equation for details)
        cv::Mat dst, dst_norm, dst_norm_scaled;
        dst = cv::Mat::zeros(img.size(), CV_32FC1);
        cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
        //normalize from 0 to 255 image dst
        cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

        cv::convertScaleAbs(dst_norm, dst_norm_scaled);
        // locate local maxima in the Harris response matrix 
        // Response matrix is a score for each local neighboor corner : det(M) - k*tr(M)^2
        // where k is the empirical harris parameter
        
        double maxOverlap = 0.0;
        for(int i=0; i<dst_norm.rows; i++){
            for(int j=0; j<dst_norm.cols; j++){
                int response = (int)dst_norm.at<float>(i,j);
                if(response> minResponse){
                     cv::KeyPoint newKeyPoint;
                     newKeyPoint.pt = cv::Point2f(j, i);
                     newKeyPoint.size = 2 * apertureSize;
                     newKeyPoint.response = response;
                     bool bOverlap = false;
                   
                    for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                      double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                       if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }

                    }
                }
                    if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }

            }
        }
    }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "HARRIS detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
            // visualize results
        if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Detector Corner Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
