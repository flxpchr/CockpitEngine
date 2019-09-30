#include "widgets/opengloverlay/cqtopencvviewergl.h"


CQtOpenCVViewerGl::CQtOpenCVViewerGl(QWidget *parent) :
QOpenGLWidget(parent)
{
    mBgColor = QColor::fromRgb(150, 150, 150);
}

void CQtOpenCVViewerGl::initializeGL()
{
    makeCurrent();
    initializeOpenGLFunctions();
    float r = static_cast<float>(mBgColor.darker().red())/255.0f;
    float g = static_cast<float>(mBgColor.darker().green())/255.0f;
    float b = static_cast<float>(mBgColor.darker().blue())/255.0f;
    glClearColor(r,g,b,1.0f);
}

void CQtOpenCVViewerGl::resizeGL(int width, int height)
{
    makeCurrent();
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(0, width, -height, 0, 0, 1);

    glMatrixMode(GL_MODELVIEW);

    recalculatePosition();

    emit imageSizeChanged(mRenderWidth, mRenderHeight);

    updateScene();
}

void CQtOpenCVViewerGl::updateScene()
{
    if (this->isVisible()) update();
}

void CQtOpenCVViewerGl::paintGL()
{
    makeCurrent();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    renderImage();
}

void CQtOpenCVViewerGl::renderImage()
{

	drawMutex.lock();
    makeCurrent();

    glClear(GL_COLOR_BUFFER_BIT);

    if (!mRenderQtImg.isNull())
    {
        glLoadIdentity();

        glPushMatrix();
        {
            if (mResizedImg.width() <= 0)
            {
                if (mRenderWidth == mRenderQtImg.width() && mRenderHeight == mRenderQtImg.height())
                    mResizedImg = mRenderQtImg;
                else
                    mResizedImg = mRenderQtImg.scaled(QSize(mRenderWidth, mRenderHeight),
                                                      Qt::IgnoreAspectRatio,
                                                      Qt::SmoothTransformation);
            }

            // ---> Centering image in draw area

            glRasterPos2i(mRenderPosX, mRenderPosY);

            glPixelZoom(1, -1);
			
            glDrawPixels(mResizedImg.width(), mResizedImg.height(), GL_RGBA, GL_UNSIGNED_BYTE, mResizedImg.bits());
			
        }
        glPopMatrix();

        // end
        glFlush();
    }

	drawMutex.unlock();
}

void CQtOpenCVViewerGl::recalculatePosition()
{
    mImgRatio = static_cast<float>(mOrigImage.cols)/static_cast<float>(mOrigImage.rows);

    mRenderWidth = this->size().width();
    mRenderHeight = floor(mRenderWidth / mImgRatio);

    if (mRenderHeight > this->size().height())
    {
        mRenderHeight = this->size().height();
        mRenderWidth = floor(mRenderHeight * mImgRatio);
    }

    mRenderPosX = floor((this->size().width() - mRenderWidth) / 2);
    mRenderPosY = -floor((this->size().height() - mRenderHeight) / 2);

    mResizedImg = QImage();
}

bool CQtOpenCVViewerGl::showImage(const cv::Mat& image)
{
	drawMutex.lock();
    if (image.channels() == 3)
        cv::cvtColor(image, mOrigImage, CV_BGR2RGBA);
    else if (image.channels() == 1)
        cv::cvtColor(image, mOrigImage, CV_GRAY2RGBA);
    else if (image.channels() == 4)
        mOrigImage = image;
    else return false;
   ;
    mRenderQtImg = QImage( const_cast<const uchar*>(mOrigImage.data),
                          mOrigImage.cols, mOrigImage.rows,
                          static_cast<int>(mOrigImage.step1()), QImage::Format_RGB32);
    recalculatePosition();

    updateScene();
	drawMutex.unlock();
    return true;
}
