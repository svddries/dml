#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dml_test");
    ros::NodeHandle nh;

    rgbd::Client client;
    client.intialize("rgbd");

    float bla = 0.02;

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

        int step = 100;

        // - - - - - - - - - - - - VERTICAL - - - - - - - - - - - -

        for(int y = 0; y < depth.rows; ++y)
        {
            for(int x = 0; x < depth.cols - step; x += step / 2)
            {
                geo::Vector3 p1_3d, p2_3d;
                if (!view.getPoint3D(x, y, p1_3d) || !view.getPoint3D(x + step, y, p2_3d))
                    continue;

                geo::Vec2 p1(p1_3d.x, -p1_3d.z);
                geo::Vec2 p2(p2_3d.x, -p2_3d.z);
                geo::Vec2 n = (p2 - p1).normalized();

                int x2_best = -1;
                float max_dist_sq = 0;

                for(int x2 = x; x2 <= x + step; ++x2)
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
                        x2_best = x2;
                    }
                }

                if (x2_best >= 0)
                {
                    float th = depth.at<float>(y, x2_best) * bla;
                    if (max_dist_sq > (th * th))
                        canvas.at<cv::Vec3b>(y, x2_best) = cv::Vec3b(0, 0, 255);
                }
            }
        }

        // - - - - - - - - - - - - HORIZONTAL - - - - - - - - - - - -

        for(int x = 0; x < depth.cols; ++x)
        {
            for(int y = 0; y < depth.rows - step; y += step / 2)
            {
                geo::Vector3 p1_3d, p2_3d;
                if (!view.getPoint3D(x, y, p1_3d) || !view.getPoint3D(x, y + step, p2_3d))
                    continue;

                geo::Vec2 p1(p1_3d.y, -p1_3d.z);
                geo::Vec2 p2(p2_3d.y, -p2_3d.z);
                geo::Vec2 n = (p2 - p1).normalized();

                int y2_best = -1;
                float max_dist_sq = 0;

                for(int y2 = y; y2 <= y + step; ++y2)
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
                        y2_best = y2;
                    }
                }

                if (y2_best >= 0)
                {
                    float th = depth.at<float>(y2_best, x) * bla;

                    if (max_dist_sq > (th * th))
                        canvas.at<cv::Vec3b>(y2_best, x) = cv::Vec3b(0, 255, 0);
                }
            }
        }

        cv::imshow("depth", canvas);

        cv::waitKey(3);
    }

    ros::spin();

    return 0;
}
