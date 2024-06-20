#ifndef COLORDETECTOR_H
#define COLORDETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace ecn
{

struct Rad
{
  double angle;
  explicit Rad(double angle) : angle{angle} {}
};

struct Deg
{
  double angle;
  explicit Deg(double angle) : angle{angle*M_PI/180} {}
};


struct CamParam
{
    double ipx, ipy, u0, v0, px, py;
    CamParam() {}
    // init from classical parameters
    CamParam(double _px, double _py, double _u0, double _v0):
        u0(_u0), v0(_v0), px(_px), py(_py), ipx(1./_px), ipy(1./_py)
    {}

    CamParam(int width, int height, Rad field_of_view) :
      CamParam(width, height, field_of_view.angle)
    {}

    CamParam(int width, int height, Deg field_of_view) :
      CamParam(width, height, field_of_view.angle)
    {}

private:

    // init from resolution and horizontal view angle
    CamParam(int width, int height, double field_of_view)
    {
        u0 = width/2.;
        v0 = height/2.;
        ipx = ipy = tan(field_of_view/2)/u0;
        px = py = 1./ipx;
    }


};

class ColorDetector
{
public:
    ColorDetector()
    {
        // default values
        setContourDisplay(255,255,255);
        setSaturationValue(130,95);

        setCamera(640,480,Deg(60));
    }

    ColorDetector(int r, int g, int b)
    {
        setCamera(640,480,Deg(60));
        setContourDisplay(255,255,255);
        setSaturationValue(130,95);
        detectColor(r, g, b);
    }

    // tuning
    void setSaturationValue(int sat, int value)
    {
        if(show_segment_)
        {
             cv::setTrackbarPos("Saturation", "Color detector - range", sat);
             cv::setTrackbarPos("Value", "Color detector - range", value);
        }
        sat_ = sat; val_ = value;
    }
    void detectColor(int r, int g, int b);
    void showSegmentation();
    inline void showOutput() {show_output_ = true;}
    inline void fitCircle() {fit_circle_ = true;}
    inline void setContourDisplay(int r, int g, int b)
    {
        ccolor = cv::Scalar(b,g,r);
    }
    void setCamera(double px, double py, double u0, double v0)
    {
        cam = CamParam(px, py, u0, v0);
        // default values for x,y
        x_ = cam.u0/2;
        y_ = cam.v0/2;
    }
    void setCamera(int width, int height, Rad field_of_view)
    {
        cam = CamParam(width, height, field_of_view);
    }
    void setCamera(int width, int height, Deg field_of_view)
    {
        cam = CamParam(width, height, field_of_view);
    }

    inline double xLim() const {return cam.u0*cam.ipx;}
    inline double yLim() const {return cam.v0*cam.ipy;}

    // processing functions
    std::vector<cv::Point> findMainContour(const cv::Mat &_im);

    bool process(const cv::Mat &_im, cv::Mat &_im_processed, bool write_output = true);
    bool process(const cv::Mat &_im);

    // get resulting info
    inline double x() const {return (x_-cam.u0)*cam.ipx;}
    inline double y() const {return (y_-cam.v0)*cam.ipy;}
    inline double area() const {return area_*cam.ipx*cam.ipy;}
    inline double x_p() const {return x_;}
    inline double y_p() const {return y_;}
    inline double area_p() const {return area_;}
    CamParam cam;

protected:    
    double x_=0, y_=0, area_=0;

    std::vector<int> hue_;
    int sat_, val_;
    cv::Scalar ccolor;
    cv::Mat img_, seg1_, seg2_;
    bool show_segment_ = false, show_output_ = false, fit_circle_ = false;

};
}

#endif // COLORDETECTOR_H
