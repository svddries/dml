#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

#include <geolib/ros/tf_conversions.h>

#include <queue>

#include <dml/mesh_tools.h>
#include <geolib/Box.h>

#include "dml/line_estimator.h"

#include "timer.h"

// ----------------------------------------------------------------------------------------------------

void findObstacles(const rgbd::Image& image, const geo::Pose3D& sensor_pose)
{
    Timer timer;

    cv::Mat depth = image.getDepthImage();

    int width = depth.cols;
    int height = depth.rows;

    rgbd::View view(image, width);
    const geo::DepthCamera& rasterizer = view.getRasterizer();

    cv::Mat obstacle_map(500, 500, CV_32FC1, 0.0);
    cv::Mat bla = depth.clone();

    cv::Mat buffer(height, 1, CV_64FC4, 0.0);
    std::vector<geo::Vector3> p_floors(height);

    int x_step = 1;
    int edge_window_size = 30;

    for(int x = 0; x < width; x += x_step)
    {
        buffer.at<cv::Vec4d>(0, 0) = cv::Vec4d(0, 0, 0, 0);

        for(int y = 1; y < height; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d)
            {
                buffer.at<cv::Vec4d>(y, 0) = buffer.at<cv::Vec4d>(y - 1, 0);
                continue;
            }

            geo::Vector3 p_sensor = rasterizer.project2Dto3D(x, y) * d;
            p_floors[y] = sensor_pose * p_sensor;
            const geo::Vector3& p_floor = p_floors[y];

            if (p_floor.z > 0.2)
            {
                cv::Point2i p(p_floor.x * 50 + obstacle_map.rows / 2,
                              obstacle_map.cols / 2 - p_floor.y * 50);
                if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                    obstacle_map.at<float>(p) = 0.5;
            }

            //            x_sum_ += x;
            //            y_sum_ += y;
            //            xy_sum_ += x * y;
            //            x2_sum_ += x * x;

            buffer.at<cv::Vec4d>(y, 0) = buffer.at<cv::Vec4d>(y - 1, 0)
                        + cv::Vec4d(p_floor.y, p_floor.z, p_floor.y * p_floor.z, p_floor.y * p_floor.y);
        }

        for(int y = edge_window_size; y < height - edge_window_size; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d)
                continue;

            const geo::Vector3& p_floor = p_floors[y];

            if (p_floor.z < 0.2)
            {
//                const cv::Vec4d& bm = buffer.at<cv::Vec4d>(y, 0);

//                double c1, s1;
//                cv::Vec4d b1 = bm - buffer.at<cv::Vec4d>(y - edge_window_size, 0);
//                s1 = (edge_window_size * b1[2] - b1[0] * b1[1]) / (edge_window_size * b1[3] - b1[0] * b1[0]);
                //            c1 = b1[1] / edge_window_size - s1 * (b1[0] / edge_window_size);

                double c2, s2;
                cv::Vec4d b2 = buffer.at<cv::Vec4d>(y + edge_window_size / 2, 0)
                        - buffer.at<cv::Vec4d>(y - edge_window_size / 2, 0);
                s2 = (edge_window_size * b2[2] - b2[0] * b2[1]) / (edge_window_size * b2[3] - b2[0] * b2[0]);
                //            c2 = b2[1] / edge_window_size - s1 * (b2[0] / edge_window_size);

//                if (std::abs(s1) > 1 && std::abs(s2) < 1)
                if (std::abs(s2) > 1)
                {
                    bla.at<float>(y, x) = 10;
                    cv::Point2i p(p_floor.x * 50 + obstacle_map.rows / 2,
                                  obstacle_map.cols / 2 - p_floor.y * 50);
                    if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                        obstacle_map.at<float>(p) = 1;
                }
            }
        }
    }

    timer.stop();
    std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

    cv::imshow("bla", bla / 10);
    cv::imshow("obstacle map", obstacle_map);

//        cv::imshow("cross_section", cross_section);
    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dml_test_poly");
    ros::NodeHandle nh;

    rgbd::Client client;
    client.intialize("/amigo/top_kinect/rgbd");

    tf::TransformListener tf_listener;

    std::queue<rgbd::ImageConstPtr> image_buffer;

    ros::Rate r(30);
    while (ros::ok())
    {
        r.sleep();

        // - - - - - - - - - - - - - - - - - -
        // Fetch kinect image and place in image buffer

        rgbd::ImageConstPtr rgbd_image = client.nextImage();
        if (rgbd_image && rgbd_image->getDepthImage().data)
            image_buffer.push(rgbd_image);

        if (image_buffer.empty())
            continue;

        rgbd_image = image_buffer.front();

        // - - - - - - - - - - - - - - - - - -
        // Determine absolute kinect pose based on TF

        geo::Pose3D sensor_pose;

        try
        {
            tf::StampedTransform t_sensor_pose;
            tf_listener.lookupTransform("/amigo/base_link", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
            geo::convert(t_sensor_pose, sensor_pose);
            image_buffer.pop();
        }
        catch(tf::ExtrapolationException& ex)
        {
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error (i.e., the image is too old or
                // to new, respectively). If it is too old, discard it.

                tf::StampedTransform latest_sensor_pose;
                tf_listener.lookupTransform("/amigo/base_link", rgbd_image->getFrameId(), ros::Time(0), latest_sensor_pose);
                // If image time stamp is older than latest transform, throw it out
                if ( latest_sensor_pose.stamp_ > ros::Time(rgbd_image->getTimestamp()) )
                {
                    image_buffer.pop();
                    ROS_WARN_STREAM("[ED KINECT PLUGIN] Image too old to look-up tf: image timestamp = " << std::fixed
                                    << ros::Time(rgbd_image->getTimestamp()));
                }

                continue;
            }
            catch(tf::TransformException& exc)
            {
                ROS_WARN("[ED KINECT PLUGIN] Could not get latest sensor pose (probably because tf is still initializing): %s", ex.what());
                continue;
            }
        }
        catch(tf::TransformException& ex)
        {
            ROS_WARN("[ED KINECT PLUGIN] Could not get sensor pose: %s", ex.what());
            continue;
        }

        // Convert from ROS coordinate frame to geolib coordinate frame
        sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

        tf::Matrix3x3 m;
        geo::convert(sensor_pose.R, m);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        sensor_pose.R.setRPY(roll, pitch, 0);

//        std::cout << roll << " " << pitch << " " << yaw << std::endl;

        // - - - - - - - - - - - - - - - - - -

        findObstacles(*rgbd_image, sensor_pose);
    }

    return 0;
}
