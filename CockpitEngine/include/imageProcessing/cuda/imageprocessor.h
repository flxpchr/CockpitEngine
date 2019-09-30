// Check whether the given token has been #defined earlier in the file or in an included file
// If not, it includes the code between it and the closing #else or, if no #else is present, #endif statement
    #ifndef IMAGEPROCESSORCUDA_H

//Definition of the header file "imageprocessor.h", after checking that it does not exist.
    #define IMAGEPROCESSORCUDA_H
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <vector>
#include <map>
#include <string>
#include <QString>

// Declaration of a global string-type
    extern struct minX
        {
            bool operator() (cv::Point p1, cv::Point p2)
                {
                    return p1.x < p2.x;
                }
        }   minXoperator;

// Declaration of a global string-type object
    extern struct minY
        {
            bool operator() (cv::Point p1, cv::Point p2)
                {
                    return p1.y < p2.y;
                }
        }   minYoperator;


/*!
 * \brief THE IMAGE PROCESSOR INTERFACE
 * Definition of the ImageProcessor Class
 * Visual tool tracking engine should implement this interface
 */

namespace gpu{

// Definition of the "ImageProcessor" Class
class ImageProcessor
{
public:
    ImageProcessor();               //Constructor
    virtual ~ImageProcessor(){}     //Destructor

    /*!
     * \brief Sets the image to be processed
     * \param src
     */
    virtual void setImage(cv::Mat &src);

    /*!
     * \brief Locates the tool within the image
     * \param _toolCoordinates output tool coordinates
     * \param _flags optional flags
     * \return true if the tool was found within the image
     *
     * Image has to be set using setImage or a proper constructor beforehand
     */
    virtual bool locateTool(cv::Point2f &_toolCoordinates, int _flags = 0) = 0;

    /*!
     * \brief Locates the tool within the image, but takes into account its former velocity and position
     * \param _toolCoordinates output tool coordinates
     * \param _lastPosition last known tool position, (-1, -1) if unknown
     * \param _velocity last known tool velocity, (0, 0) if unknown
     * \param _flags optional flags
     * \return true if the tool was found within the image
     */
    virtual bool locateTool(cv::Point2f &_toolCoordinates, cv::Point2f _lastPosition, const cv::Point2f _velocity, int _flags);

    /*!
     * \brief Returns requested supplementary, intermediate images that were used during image processing
     * \param _images images mapping
     *
     * Correct mapping keys have to be inserted before using this function, as only images corresponding to preset keys will be returned
     * Example: retrieve only binary and CIEL image
     *   map<string, Mat> supplementary;
     *   supplementary["ciel"] = Mat();
     *   supplementary["bin"] = Mat();
     */
    virtual void getSupplementaryImages(std::map<std::string, cv::Mat> &_images);
    virtual void getSupplementaryImage(std::string key, cv::Mat &_image);
    virtual void getSupplementaryCudaImage(std::string key, cv::cuda::GpuMat &_image);

    virtual bool findToolContour(cv::Mat &_src, std::vector<cv::Point> &_contour, float _inertiaUpperThreshold, float _areaLowerThreshold, float radius, cv::Point2f center);

    virtual std::vector<cv::Point> findContourExtremePoints(std::vector<cv::Point> &_contour);

    virtual int countBorderPoints(std::vector<cv::Point> _pts, cv::Rect _frame);

    static float segmentation(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst, bool otsu, int defaultThreshold);

    void setBinarizationThreshold(int _val);

    void setOtsuMethod();

    std::string getOtsuThreshold();

    void setScalingFactor(const QString &arg1) {m_scalingFactor = arg1.toFloat();}

    void setX_searchArea(const QString &arg1) {m_marginHorizontal = arg1.toInt();}
    void setY_searchArea(const QString &arg1) {m_marginVertical = arg1.toInt();}

    virtual void extractChroma(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst);

    virtual void computeHistogram(cv::cuda::GpuMat& src, cv::cuda::GpuMat& dst);

    virtual void drawHistogram(cv::cuda::GpuMat &src, cv::cuda::GpuMat &dst);

    virtual void detectEdges(cv::cuda::GpuMat &_src, cv::cuda::GpuMat &_dst, float _dir = 0.5);

    /* Collection of static functions that may be helpful even outside the Tooltip localization context */
    static void deinterlace(cv::cuda::GpuMat &src, cv::cuda::GpuMat &dst);

protected:
    typedef std::map<std::string, cv::Mat>::iterator frame_it;
    typedef std::map<std::string, cv::cuda::GpuMat>::iterator frameCuda_it;

    // Current frame and its label
        std::map<std::string, cv::Mat> m_frame;
        std::map<std::string, cv::cuda::GpuMat> m_frameCuda;

    // Scaling factor to reduce the frame size
        float m_scalingFactor;

    // Binarization Variables
        int m_binarizationThreshold;
        int m_OtsuThreshold;
        bool m_OtsuMethod;

    // Area for tool search
        int m_marginVertical, m_marginHorizontal;
        cv::Rect m_toolSearchArea;

private:

};

}
#endif // IMAGEPROCESSORCUDA_H
