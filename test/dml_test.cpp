#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

float bla = 0.02;
float max_range = 5;

// ----------------------------------------------------------------------------------------------------

int findEdgeHorizontal(int y, int x_start, int x_end, const rgbd::View view, cv::Mat& canvas)
{
    if (x_end - x_start < 10) // TODO: magic number
        return -1;

    geo::Vector3 p1_3d, p2_3d;
    if (!view.getPoint3D(x_start, y, p1_3d) || !view.getPoint3D(x_end, y, p2_3d))
        return -1;

    geo::Vec2 p1(p1_3d.x, -p1_3d.z);
    geo::Vec2 p2(p2_3d.x, -p2_3d.z);
    geo::Vec2 n = (p2 - p1).normalized();

    int x_edge = -1;
    float max_dist_sq = 0;
    for(int x2 = x_start; x2 <= x_end; ++x2)
    {
        geo::Vector3 p_3d;
        if (!view.getPoint3D(x2, y, p_3d))
            continue;

        geo::Vec2 p(p_3d.x, -p_3d.z);

        // Calculate distance of p to line (p1, p2)
        geo::Vec2 p1_p = p - p1;

        float dist_sq = (p1_p - (p1_p.dot(n) * n)).length2();

        if (dist_sq > max_dist_sq)
        {
            max_dist_sq = dist_sq;
            x_edge = x2;
        }
    }

    if (x_edge < 0)
        return -1;

    float d = view.getDepth(x_edge, y);
    if (d > max_range)
        return -1;

    float th = d * bla;
    if (max_dist_sq < (th * th))
        return -1;

    // Visualize
    canvas.at<cv::Vec3b>(y, x_edge) = cv::Vec3b(0, 0, 255);

    // Recursively find edges
    findEdgeHorizontal(y, x_start, x_edge, view, canvas);

    return x_edge;
}

// ----------------------------------------------------------------------------------------------------


int findEdgeVertical(int x, int y_start, int y_end, const rgbd::View view, cv::Mat& canvas)
{
    if (y_end - y_start < 10) // TODO: magic number
        return -1;

    geo::Vector3 p1_3d, p2_3d;
    if (!view.getPoint3D(x, y_start, p1_3d) || !view.getPoint3D(x, y_end, p2_3d))
        return -1;

    geo::Vec2 p1(p1_3d.y, -p1_3d.z);
    geo::Vec2 p2(p2_3d.y, -p2_3d.z);
    geo::Vec2 n = (p2 - p1).normalized();

    int y_edge = -1;
    float max_dist_sq = 0;
    for(int y2 = y_start; y2 <= y_end; ++y2)
    {
        geo::Vector3 p_3d;
        if (!view.getPoint3D(x, y2, p_3d))
            continue;

        geo::Vec2 p(p_3d.y, -p_3d.z);

        // Calculate distance of p to line (p1, p2)
        geo::Vec2 p1_p = p - p1;

        float dist_sq = (p1_p - (p1_p.dot(n) * n)).length2();

        if (dist_sq > max_dist_sq)
        {
            max_dist_sq = dist_sq;
            y_edge = y2;
        }
    }

    if (y_edge < 0)
        return -1;

    float d = view.getDepth(x, y_edge);
    if (d > max_range)
        return -1;

    float th = d * bla;
    if (max_dist_sq < (th * th))
        return -1;

    // Visualize
    canvas.at<cv::Vec3b>(y_edge, x) = cv::Vec3b(0, 255, 0);

    // Recursively find edges
    findEdgeVertical(x, y_start, y_edge, view, canvas);

    return y_edge;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dml_test");
    ros::NodeHandle nh;

    rgbd::Client client;
    client.intialize("/amigo/top_kinect/rgbd");

    ros::Rate r(30);
    while (ros::ok())
    {
        r.sleep();

        rgbd::Image image;
        if (!client.nextImage(image))
            continue;

        cv::Mat depth = image.getDepthImage();
        if (!depth.data)
            continue;

        rgbd::View view(image, depth.cols);

        cv::Mat canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(50, 50, 50));
        for(int i = 0; i < depth.cols * depth.rows; ++i)
        {
            float d = depth.at<float>(i);
            if (d == d)
            {
                int c = d / 8 * 255;
                canvas.at<cv::Vec3b>(i) = cv::Vec3b(c, c, c);
            }
            else
                canvas.at<cv::Vec3b>(i) = cv::Vec3b(100, 0, 0);
        }

        tue::Timer timer;
        timer.start();

        // - - - - - - - - - - - - HORIZONTAL - - - - - - - - - - - -

        for(int y = 0; y < depth.rows; ++y)
        {
            int x = 0;
            int x_start;

            // Find first valid value
            for(; x < depth.cols; ++x)
            {
                float d = depth.at<float>(y, x);
                if (d == d && d > 0)
                {
                    x_start = x;
                    break;
                }
            }

            for(; x < depth.cols; ++x)
            {
                int x_end = x;

                float d = depth.at<float>(y, x);
                if (d != d || d == 0)
                    continue;

                if (x_end - x_start >= 30) // TODO: magic number
                {
                    int x_edge = findEdgeHorizontal(y, x_start, x_end, view, canvas);

                    if (x_edge >= 0)
                    {
                        x = x_edge;
                        x_start = (x_start + x_end) / 2;
                    }
                    else
                        x_start = (x_start + x_end) / 2;
                }
            }
        }

        // - - - - - - - - - - - - VERTICAL - - - - - - - - - - - -

        for(int x = 0; x < depth.cols; ++x)
        {
            int y = 0;
            int y_start;

            // Find first valid value
            for(; y < depth.rows; ++y)
            {
                float d = depth.at<float>(y, x);
                if (d == d && d > 0)
                {
                    y_start = y;
                    break;
                }
            }

            for(; y < depth.rows; ++y)
            {
                int y_end = y;

                float d = depth.at<float>(y, x);
                if (d != d || d == 0)
                    continue;

                if (y_end - y_start >= 30) // TODO: magic number
                {
                    int y_edge = findEdgeVertical(x, y_start, y_end, view, canvas);

                    if (y_edge >= 0)
                    {
                        y = y_edge;
                        y_start = (y_start + y_end) / 2;
                    }
                    else
                        y_start = (y_start + y_end) / 2;
                }
            }
        }

        std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

        cv::imshow("depth", canvas);

        cv::waitKey(3);
    }

    ros::spin();

    return 0;
}
