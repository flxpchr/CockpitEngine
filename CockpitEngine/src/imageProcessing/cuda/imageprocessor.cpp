#include "imageProcessing/cuda/imageprocessor.h"

using namespace gpu;
using namespace cv;
using namespace std;

minX minXoperator;
minY minYoperator;

ImageProcessor::ImageProcessor() :
    m_scalingFactor(1),
    m_OtsuMethod(false),
    m_OtsuThreshold(0),
    m_binarizationThreshold(0),
    m_marginHorizontal(0),
    m_marginVertical(0) {
    cv::cuda::setDevice(0);
}

void ImageProcessor::setImage(cv::Mat &_src) {
    m_frame["bgr"] = _src;
    m_frameCuda["bgr"] = cuda::GpuMat(_src);
}

void ImageProcessor::getSupplementaryImage(std::string key, cv::Mat &_image){
    _image = m_frame[key];
}

void ImageProcessor::getSupplementaryCudaImage(std::string key, cv::cuda::GpuMat &_image){
    _image = m_frameCuda[key];
}

void ImageProcessor::getSupplementaryImages(std::map<std::string, cv::Mat > & _images)
{
    for(frame_it it = m_frame.begin(); it != m_frame.end(); it++){
        if(_images.find(it->first)!=_images.end()){
            _images[it->first] = it->second.clone();
        }
    }
    for(frameCuda_it it = m_frameCuda.begin(); it != m_frameCuda.end(); it++){
        if(_images.find(it->first)!=_images.end()){
            it->second.download(_images[it->first]);
        }
    }
}

void ImageProcessor::extractChroma(cuda::GpuMat &src, cuda::GpuMat &dst)
{
    cuda::GpuMat ciel;
    std::vector<cuda::GpuMat> channels(3);

    src.convertTo(ciel, CV_64FC3);
    // Division of a multi-channel array into several single-channel arrays
    cv::cuda::split(ciel, channels);

    cv::Mat a2, b2, c;
    // Pow raises every array element to a power
    cv::pow(channels[1], 2, a2);
    cv::pow(channels[2], 2, b2);
    cv::sqrt(a2+b2, c);

    double min, max;
    // Normalization
    minMaxLoc(c, &min, &max);
    c = (c-min)/(max-min) *255;

    // Return 1 channel in uint8 (0..255)
    c.convertTo(dst, CV_8U);
}

void ImageProcessor::computeHistogram(cuda::GpuMat &src /*CV_8UC1 */, cuda::GpuMat &dst
                                      /* one row, 256 columns, and the CV_32SC1*/)
{
    cuda::calcHist( src, dst );
}

void ImageProcessor::drawHistogram(cuda::GpuMat &src, cuda::GpuMat &dst)
{
    int histSize = 256;
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound(static_cast<float>(hist_w)/static_cast<float>(histSize));
    cv::Mat srcCPU =  cv::Mat();
    src.download(srcCPU);
    cv::Mat dstCPU = cv::Mat( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
    cv::normalize(src, src, 0, dstCPU.rows, cv::NORM_MINMAX, -1, cv::Mat() );
    for( int i = 1; i < histSize; i++ )
    {
        cv::line( dstCPU, cv::Point( bin_w*(i), hist_h),
                  cv::Point( bin_w*(i), hist_h - cvRound(srcCPU.at<float>(i)) ),
                  cv::Scalar( 255, 255, 255), 1);
    }
    dst.upload(dstCPU);
}

void ImageProcessor::detectEdges(cuda::GpuMat &_src, cuda::GpuMat &_dst, float _dir)
{
    cv::Mat sobelx, sobely, sobel;

    // Calculates the first x- or y- image derivative using Scharr operator.
    // dx derivative
    cv::Scharr(_src, sobelx, CV_16S, 1, 0);
    // dy derivative
    cv::Scharr(_src, sobely, CV_16S, 0, 1);

    // Computation of the weighted sum of two arrays
    cv::addWeighted(sobelx, _dir, sobely, 1-_dir, 0, sobel);
    // Scales, calculates absolute values, and converts the result to 8-bit
    cv::convertScaleAbs(sobel, _dst);
}

void ImageProcessor::deinterlace(cuda::GpuMat &src, cuda::GpuMat &dst)
{
    cv::Mat temp;
    for(int i = 0; i < src.rows; i+=2)
        // push_back adds elements to the bottom of the matrix
        temp.push_back(src.row(i));
    cv::resize(temp,dst,src.size(),0,0,cv::INTER_AREA);
}

bool ImageProcessor::findToolContour(cv::Mat &_src, std::vector<cv::Point> &_contour, float _inertiaUpperThreshold, float _areaLowerThreshold, float radius, Point2f center)
{
    // boolean variable initialization
    bool ret = false;

    // Each contour is an array of points
    std::vector<std::vector<cv::Point> > contours;

    // "clone" creates a copy of the matrix, called "temp"
    cv::Mat temp = _src;//.clone();

    // "FindContours" finds contours in a binary image
    cv::findContours(temp, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    std::vector<cv::Point> selectedContour;
    // Scan all the contours
    for(auto contour : contours){
        float area = cv::contourArea(contour);

        // If the area is bigger than a certain threshold and the size is bigger than 5 (why 5?)
        if(area > _areaLowerThreshold && contour.size() >= 5){
            // Fit an ellipse around a set of 2D points.
            cv::RotatedRect minEllipse = cv::fitEllipse(contour);

            // Calculate the ratio width/height
            float inertiaRatio = minEllipse.size.width/minEllipse.size.height;

            if(inertiaRatio > _inertiaUpperThreshold){
                ret = true;
                selectedContour = contour;
            }
        }
    }

    // If the current contour respects all the constraints, it is saved
    if(ret){
        minEnclosingCircle(selectedContour, center, radius);
        _contour = selectedContour;
    }

    return ret;
}

std::vector<cv::Point> ImageProcessor::findContourExtremePoints(std::vector<cv::Point> &_contour)
{
    // Variable definition
    std::vector<cv::Point> extremePts;

    extremePts.push_back(*std::min_element(_contour.begin(), _contour.end(), minYoperator));
    extremePts.push_back(*std::max_element(_contour.begin(), _contour.end(), minYoperator));
    extremePts.push_back(*std::max_element(_contour.begin(), _contour.end(), minXoperator));
    extremePts.push_back(*std::min_element(_contour.begin(), _contour.end(), minXoperator));

    return extremePts;
}

int ImageProcessor::countBorderPoints(std::vector<cv::Point> _pts, cv::Rect _frame)
{
    int borderPts = 0;
    for(size_t i = 0; i < _pts.size(); i++)
    {
        if(_pts.at(i).x <= _frame.x || _pts.at(i).x >= _frame.x + _frame.width ||
                _pts.at(i).y <= _frame.y || _pts.at(i).y >= _frame.y + _frame.height)
        {
            borderPts++;
        }
    }
    return borderPts;
}

// BINARIZATION THRESHOLD
void ImageProcessor::setBinarizationThreshold(int _val) {

    // Protected data in ImageProcessor Class
    m_binarizationThreshold = _val;
}

void ImageProcessor::setOtsuMethod() {

    // This function toggles the flag state to use the Otsu thresholdin method
    m_OtsuMethod = !m_OtsuMethod;
}

string ImageProcessor::getOtsuThreshold(){

    string OtsuTreshold;
    OtsuTreshold = std::to_string(m_OtsuThreshold);
    return OtsuTreshold;
}

bool ImageProcessor::locateTool(cv::Point2f &_toolCoordinates, cv::Point2f _lastPosition, const cv::Point2f _velocity, int _flags)
{
    return locateTool(_toolCoordinates, _flags);
}

float ImageProcessor::segmentation(cuda::GpuMat &src, cuda::GpuMat &dst, bool otsu, int defaultThreshold){
    Mat bin, dist;
    float thres = -1;

    // TODO Add a median blur before thresholding ??

    // if the Otsu flag is true, the method for automatic threshold calculation is applied
    if(otsu)
        thres = cv::cuda::threshold(src, bin, 0, 255, CV_THRESH_BINARY_INV + CV_THRESH_OTSU);
    // else applies a fixed value threshold
    else
        cv::cuda::threshold(src, bin, defaultThreshold, 255, CV_THRESH_BINARY);

    // Calculation of the distance to the closest zero pixel for each pixel of the source image
    distanceTransform(bin, dist, CV_DIST_L2, 3);

    // Normalization of the distance-based image in the range 0-255
    cv::normalize(dist, dist, 0, 255, NORM_MINMAX);

    Mat dist_8U;
    // convertTo() -> converts an array to another data type with optional scaling.
    dist.convertTo(dist_8U, CV_8UC1);

    // Thresholding on the "dist_8U" obtained from the distance function MA 03/05/2017
    // float percentage = 0.7f;
    // threshold(dist_8U, dst, 255*percentage, 255, CV_THRESH_BINARY);
    cuda::threshold(dist_8U, dst, 0, 255, CV_THRESH_BINARY);
    // thres = threshold(dist_8U, dst, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    return thres;
}

