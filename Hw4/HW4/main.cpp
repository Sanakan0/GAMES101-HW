#include <chrono>
#include <iostream>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    
    if (control_points.size()==1) return control_points[0];
    std::vector<cv::Point2f>newpts;
    for (int i=0;i<control_points.size()-1;++i){
        auto& p0 = control_points[i];
        auto& p1 = control_points[i+1];
        newpts.push_back(t*p0+(1-t)*p1);
    }
    return recursive_bezier(newpts, t);

}     
int mvx[9]={-1,0,1,-1,0,1,-1,0,1};
int mvy[9]={-1,-1,-1,0,0,0,1,1,1};
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (float t=0.0f;t<=1;t+=0.001){
        auto pt = recursive_bezier(control_points, t);
        for (int i=0;i<9;++i){
            int cx=pt.x+mvx[i];
            int cy=pt.y+mvy[i];
            if (0<=cx&&cx<700&&0<=cy&&cy<=700){
                cv::Point2f cnter(cx+0.5,cy+0.5);
                auto dist=cv::norm(cnter-pt);
                auto alpha = 1 - dist*sqrt(2)/3;
                window.at<cv::Vec3b>(cy,cx)[1]=std::max(window.at<cv::Vec3b>(cy,cx)[1],(uchar)(255*alpha)) ;
            }
        }
        
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
