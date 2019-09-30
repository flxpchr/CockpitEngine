#include "imageProcessing/cuda/markerprocessor.h"
#include "QDebug"
#include <vector>
#include <opencv2/cudafilters.hpp>

using namespace cv;
using namespace std;
using namespace gpu;

MarkerProcessor::MarkerProcessor():
    m_minimumArea(50),
    m_circleCenter(-1, -1),
    m_circleRadius(-1),
    m_sphereSearchArea(0, 0, 0, 0),
    m_morpho(cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_8UC1, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)))){
    cv::cuda::setDevice(0);
}

bool MarkerProcessor::locateTool(cv::Point2f &_toolCoordinates, int _flags) {

    //if(!m_sphereSearchArea.area())

    // CPU bluring + Cuda Morpho //20ms
    // bluring is faster in CPU :p
    cv::cvtColor(m_frame["bgr"], m_frame["gray"], CV_BGR2GRAY);    
    //cv::medianBlur(m_frame["gray"], m_frame["blurred"], 3);
    //m_frameCuda["grayBlurred"].upload(m_frame["blurred"]);
    m_frameCuda["gray"].upload(m_frame["gray"]);

    cv::cuda::threshold(m_frameCuda["gray"],m_frameCuda["bin"], 240, 255, CV_THRESH_BINARY);
    //cv::cuda::threshold(m_frameCuda["grayBlurred"],m_frameCuda["bin"], 240, 255, CV_THRESH_BINARY);
    m_morpho->apply(m_frameCuda["bin"], m_frameCuda["bin"]);
    m_frameCuda["bin"].download(m_frame["bin"]);

    vector<Point> contour;
    bool found = findSphericalMarkerContour(m_frame["bin"], contour, m_minimumArea, m_circleRadius, m_circleCenter);


    if(found)
    {
        _toolCoordinates = m_circleCenter / m_scalingFactor;

        m_sphereSearchArea = Rect(m_circleCenter.x - m_circleRadius, m_circleCenter.y - m_circleRadius, 2 * m_circleRadius, 2 * m_circleRadius);
    }
    else
        m_sphereSearchArea = Rect();

    return found;
}

bool MarkerProcessor::findSphericalMarkerContour(cv::Mat &_src, vector<Point> &_contour, int _areaLowerThreshold, float &radius, Point2f &center)
{
    // boolean variable initialization
    bool ret = false;

    // Each contour is an array of points
    cv::Mat sourceImage = _src;
    vector<vector<Point>> retrievedContours;
    vector<Point> selectedContour;
    float score = 0, tempScore = 0;

    // "FindContours" finds contours in a binary image  //ONLY CPU Available
    cv::findContours(sourceImage, retrievedContours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Scan all the contours
    for(auto contour : retrievedContours){

        float area = cv::contourArea(contour);
        if(area > _areaLowerThreshold && contour.size() >= 5){

            // Fit the contour to an ellipse
            RotatedRect minEllipse = fitEllipse(contour);
            // Compute the major and minor axis
            float majorEllipseAxis = max(minEllipse.size.height, minEllipse.size.width);
            float minorEllipseAxis = min(minEllipse.size.height, minEllipse.size.width);
            // Eccentricity [0, 1]
            float inertiaRatio = minorEllipseAxis / majorEllipseAxis;

            // Enclose points into a circle
            minEnclosingCircle(contour, center, radius);
            float roundnessIndex = contourArea(contour) / (radius * radius * CV_PI);

            // Total score for the contour
            score = roundnessIndex * inertiaRatio;

            if(score >= tempScore){
                tempScore = score;
                ret = true;
                selectedContour = contour;
            }
        }
    }

    // If the current contour respects all the constraints, it is saved
    if(ret)
    {
        minEnclosingCircle(selectedContour, center, radius);
        _contour = selectedContour;
    }

    return ret;
}
