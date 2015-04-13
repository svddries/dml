#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

float bla = 0.02;
float max_range = 5;

int findEdgeVertical(int x, int y_start, int y_end, const rgbd::View view, cv::Mat& canvas, std::string indent = "")
{
    if (x == 320)
        std::cout << indent << y_start << " - " << y_end << std::endl;

//    if (indent.size() > 0)
//        cv::line(canvas, cv::Point(x, y_start), cv::Point(x, y_end), cv::Scalar(255, 0, 0));
//    else
//        cv::line(canvas, cv::Point(x, y_start + 1), cv::Point(x, y_end - 1), cv::Scalar(0, 0, 255));

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

    float th = view.getDepth(x, y_edge) * bla;
    if (max_dist_sq < (th * th))
        return -1;

    // Visualize
    if (indent.size() > 0)
        canvas.at<cv::Vec3b>(y_edge, x) = cv::Vec3b(0, 0, 255);
    else
        canvas.at<cv::Vec3b>(y_edge, x) = cv::Vec3b(0, 255, 0);



    // Recursively find edges
    findEdgeVertical(x, y_start, y_edge, view, canvas, indent + "    ");
//    y_edge = findEdgeVertical(x, y_edge, y_end, view, canvas);

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


//        for(int x = 0; x < depth.cols; ++x)
//        {
//            for(int y = 1; y < depth.rows + 1; ++y)
//            {
//                float& d1 = depth.at<float>(y - 1, x);
//                float& d2 = depth.at<float>(y, x);
//                float& d3 = depth.at<float>(y + 1, x);

//                if (d1 != d1 || d1 == 0 || d2 != d2 || d2 == 0 || d3 != d3 || d3 == 0)
//                    continue;

//                float diff1 = d2 - d1;
//                float diff2 = d2 - d3;

//                if (diff1 * diff2 > 0) // && std::abs(diff1) < 0.1 && std::abs(diff2) < 0.1)
//                    d2 = d1;
//            }
//        }




        // - - - - - - - - - - - - HORIZONTAL - - - - - - - - - - - -

//        for(int y = 0; y < depth.rows; ++y)
//        {
//            step = 50;
//            for(int x = 0; x < depth.cols - step; x += step / 2)
//            {
//                float d = depth.at<float>(y, x);
//                if (d != d || d == 0)
//                    continue;

//                step = 50 / d;
//                int x_end = std::min(depth.cols, x + step);

//                geo::Vector3 p1_3d, p2_3d;
//                if (!view.getPoint3D(x, y, p1_3d) || !view.getPoint3D(x_end, y, p2_3d))
//                    continue;

//                geo::Vec2 p1(p1_3d.x, -p1_3d.z);
//                geo::Vec2 p2(p2_3d.x, -p2_3d.z);
//                geo::Vec2 n = (p2 - p1).normalized();

//                int x2_best = -1;
//                float max_dist_sq = 0;

//                for(int x2 = x; x2 <= x_end; ++x2)
//                {
//                    geo::Vector3 p_3d;
//                    if (!view.getPoint3D(x2, y, p_3d))
//                        continue;

//                    geo::Vec2 p(p_3d.x, -p_3d.z);

//                    // Calculate distance of p to line (p1, p2)
//                    geo::Vec2 p1_p = p - p1;

//                    float dist_sq = (p1_p - (p1_p.dot(n) * n)).length2();

//                    if (dist_sq > max_dist_sq)
//                    {
//                        max_dist_sq = dist_sq;
//                        x2_best = x2;
//                    }
//                }

//                if (x2_best >= 0)
//                {
//                    float th = depth.at<float>(y, x2_best) * bla;
//                    if (max_dist_sq > (th * th))
//                        canvas.at<cv::Vec3b>(y, x2_best) = cv::Vec3b(0, 0, 255);

//                    x = x2_best + 1;
//                }
//            }
//        }

        // - - - - - - - - - - - - VERTICAL - - - - - - - - - - - -

        for(int x = 0; x < depth.cols; ++x)
//        for(int x = 320; x <= 320; ++x)
        {
            int y = 0;
            int y_start = 0;
            float d_min, d_max, d_last;

            // Find first valid value
            for(; y < depth.rows; ++y)
            {
                float d = depth.at<float>(y, x);
                if (d == d && d > 0 && d < max_range)
                {
                    y_start = y;
                    d_min = d;
                    d_max = d;
                    d_last = d;
                    break;
                }
            }

            for(; y < depth.rows; ++y)
            {
                int y_end = y;

                float d = depth.at<float>(y, x);
                if (d != d || d == 0 || d > max_range)
                    continue;

                int y_edge = -1;
                bool check_edge = false;
                if (std::abs(d - d_last) > 0.1 * d)   // TODO: magic number
                {
                    y_edge = y;
                }
                else
                {
                    d_min = std::min(d, d_min);
                    d_max = std::max(d, d_max);

//                    if (d_max - d_min > d * bla && y_end - y_start >= 30) // TODO: magic number
                    if (y_end - y_start >= 30) // TODO: magic number
                        check_edge = true;
                }

                if (check_edge)
                {
                    y_edge = findEdgeVertical(x, y_start, y_end, view, canvas);

                    if (y_edge == -1)
                    {
                        y_start = (y_start + y_end) / 2;
                        d_min = depth.at<float>(y_start, x);
                        d_max = d_min;
                    }
                }

                if (y_edge >= 0 || y_end - y >= 50) // TODO: magic number
                {
                    if (y_edge >= 0)
                    {
                        canvas.at<cv::Vec3b>(y_edge, x) = cv::Vec3b(0, 255, 0);
                        y = y_edge;
                    }

                    y_start = (y_start + y_end) / 2;
                    d_min = depth.at<float>(y_start, x);
                    d_max = d_min;
                }

                d_last = depth.at<float>(y, x);
            }
        }

        cv::imshow("depth", canvas);

        cv::waitKey(3);
    }

    ros::spin();

    return 0;
}
