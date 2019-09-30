#ifndef MARKERPROCESSORCUDA_H
#define MARKERPROCESSORCUDA_H

#include "imageprocessor.h"

namespace gpu{

class MarkerProcessor : public ImageProcessor
{
public:
    MarkerProcessor();
    ~MarkerProcessor(){}

    bool locateTool(cv::Point2f &_toolCoordinates, int _flags = 0);

private:
    float getLargestContour(const std::vector<std::vector<cv::Point> > &contours, std::vector<cv::Point> &contour);
    bool findSphericalMarkerContour(cv::Mat &_src, std::vector<cv::Point> &_contour, int _areaLowerThreshold, float &radius, cv::Point2f &center);

    int m_minimumArea;
    float m_circleRadius;
    cv::Point2f m_circleCenter;
    cv::Rect m_sphereSearchArea;
    cv::Ptr<cv::cuda::Filter> m_morpho;
};

}
#endif // MARKERPROCESSORCUDA_H
