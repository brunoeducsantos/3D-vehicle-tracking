
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include<set>
#include "camFusion.hpp"
#include "dataStructures.h"
#include <math.h>
using namespace std;
using namespace cv;

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor); 

        cv::imwrite("../images/cluster.png",topviewImg); 
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{   
    //compute mean distance between matche keypoints
    float mean_dist=0.;
    for(auto & match : kptMatches){
        auto prevFrameKpt= kptsPrev[match.queryIdx].pt;
        auto currFrameKpt= kptsCurr[match.trainIdx].pt;
        auto diff_pt=  prevFrameKpt-currFrameKpt;
        if(boundingBox.roi.contains(currFrameKpt)){
            mean_dist+= cv::norm(prevFrameKpt - currFrameKpt);
        }
    }
    mean_dist/=kptMatches.size();
    int iter=0;
    std::vector<cv::DMatch> matchCC;
    //find keypoints matching inside boundingBox 
    for(auto & match : kptMatches)
    {       
            auto prevFrameKpt= kptsPrev[match.queryIdx].pt;
            auto currFrameKpt= kptsCurr[match.trainIdx].pt;
            auto diff_pt=  prevFrameKpt-currFrameKpt;
            float dist= cv::norm(prevFrameKpt - currFrameKpt);
            //check if are outliers and inside ROI
            if (boundingBox.roi.contains(currFrameKpt) && (fabs(mean_dist-dist)<20.)){
                matchCC.push_back(kptMatches[iter]);
            }
            iter++;
    }

    boundingBox.kptMatches = matchCC;

}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
  // compute distance ratios between all matched keypoints
    std::vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

 
    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence
    float dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
      // auxiliary variables
    double dT = 1./frameRate; // time between two measurements in seconds

    // find closest distance to Lidar points 
    double minXPrev = 1e9, minXCurr = 1e9;
    int ind=0;
    for(auto it=lidarPointsPrev.begin(); it!=lidarPointsPrev.end(); ++it) {
        if(!isOutlier(lidarPointsPrev, 40 , 0.2, ind)){
            minXPrev = minXPrev>it->x ? it->x : minXPrev;
        }
        ind++;
    }
    ind=0;
    for(auto it=lidarPointsCurr.begin(); it!=lidarPointsCurr.end(); ++it) {
        if(!isOutlier(lidarPointsCurr, 40 , 0.2, ind)){
            minXCurr = minXCurr>it->x ? it->x : minXCurr;
        }
        ind++;
    }
    
    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev-minXCurr);
    
}

bool isOutlier(std::vector<LidarPoint> &lidarPoints, int n_min, float dist, int index){
    int n_curr=0;
    for(int i=0;i<lidarPoints.size();i++){
        if(i==index) continue;
        float curr_dist = sqrt(pow(lidarPoints[i].x-lidarPoints[index].x,2) + pow(lidarPoints[i].y-lidarPoints[index].y,2) +pow(lidarPoints[i].z-lidarPoints[index].z,2));
        if(curr_dist <dist){
            n_curr++;
        }
        if(n_curr> n_min) return false;
    }
    return true;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame,bool bVis)
{       

        vector<std::pair<int, int>> bestMatches;
        bestMatches.reserve(prevFrame.boundingBoxes.size()*currFrame.boundingBoxes.size());
        //loop over all matches betwen previous and current frame
        for(auto & match : matches){
            auto prevFrameKpt= prevFrame.keypoints[match.queryIdx].pt;
            auto currFrameKpt= currFrame.keypoints[match.trainIdx].pt;
            //Get box ids inside bounding box
            for(auto &box_prev: prevFrame.boundingBoxes){
                for(auto &box_curr: currFrame.boundingBoxes){
                    //Find if matched points belong to a pair of box ID
                    if (box_prev.roi.contains(prevFrameKpt) && box_curr.roi.contains(currFrameKpt))
                    {
                        std::pair<int,int> box_m = std::make_pair(box_prev.boxID,box_curr.boxID);
                        bestMatches.push_back(box_m);
                    }
                }
            }
        }
        //Count the number of points per bbox match
        std::map<std::pair<int, int>, int> countsMatch;
        for( const auto & p :bestMatches){
            ++countsMatch[p];
        }

        //Find the highest number of points per bounding box in prev and current frame

        int minBBXcount = 10;
        //use set to store visited ids
        std::set<int> visited_ids;
        int max[countsMatch.size()];
        int match_box[countsMatch.size()]= {-1};
        for (const auto& p : countsMatch) {
            std::pair<int,int> bbIDs = p.first;
            if(visited_ids.find(bbIDs.second) != visited_ids.end()){
                if((p.second > max[bbIDs.second])&& (p.second > minBBXcount)) {
                    max[bbIDs.second]= p.second;
                    match_box[bbIDs.second]= bbIDs.first;
                }
            }
            else{
                max[bbIDs.second] = p.second;
                if(p.second>minBBXcount) match_box[bbIDs.second]= bbIDs.first;
                visited_ids.insert(bbIDs.second);
            }
        }

        for(int i=0; i< countsMatch.size();i++){
            if(match_box[i]>0){
                 bbBestMatches.insert({match_box[i],i});
            }
        }

 if (bVis) {
    cv::Mat currImg = currFrame.cameraImg.clone();
    cv::Mat prevImg = prevFrame.cameraImg.clone();
    int match_id = 0;
    for (auto bmatch:bbBestMatches) {

        std::string label = std::to_string(match_id);

        // Draw rectangle displaying the bounding box
        int top, left, width, height;
        top = currFrame.boundingBoxes[bmatch.second].roi.y;
        left = currFrame.boundingBoxes[bmatch.second].roi.x;
        width = currFrame.boundingBoxes[bmatch.second].roi.width;
        height = currFrame.boundingBoxes[bmatch.second].roi.height;
        cv::rectangle(currImg, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);
        cv::putText(currImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 255, 0), 2);

        top = prevFrame.boundingBoxes[bmatch.first].roi.y;
        left = prevFrame.boundingBoxes[bmatch.first].roi.x;
        width = prevFrame.boundingBoxes[bmatch.first].roi.width;
        height = prevFrame.boundingBoxes[bmatch.first].roi.height;
        cv::rectangle(prevImg, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 0, 255), 2);
        cv::putText(prevImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 255), 2);

        ++match_id;
        }

        string windowName = "Object classification";
        cv::Mat concat_img;
        cv::vconcat(currImg, prevImg, concat_img);
        cv::namedWindow(windowName, 2);
        cv::imshow(windowName, concat_img);
        cv::waitKey(0); // wait for key to be pressed
  }
}
