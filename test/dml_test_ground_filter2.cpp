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

class EdgeDetector
{

public:

    EdgeDetector() {}

    EdgeDetector(unsigned int w)
    {
        setWindowSize(w);
    }

    void setWindowSize(unsigned int w)
    {
        e_front.setWindowSize(w);
        e_back.setWindowSize(w);
        i_buffer_.setSize(w);
    }

    void addPoint(double x, double y, int i)
    {
        if (e_front.filled())
        {
            double x_pivot, y_pivot;
            e_front.latest(&x_pivot, &y_pivot);
            e_back.addPoint(x_pivot, y_pivot);
        }        

        i_buffer_.add(i);
        e_front.addPoint(x, y);
    }

    bool hasEdge(int* i_edge) const
    {
        if (!e_front.filled() || !e_back.filled())
            return false;

        double c1, s1, c2, s2;
        if (!e_front.calculateLineEstimate(&c1, &s1) || !e_back.calculateLineEstimate(&c2, &s2))
            return false;

        if (std::abs(s1) > 0.7 && std::abs(s2) < 0.7)
        {
            *i_edge = i_buffer_.oldest();
            return true;
        }

        return false;

//        geo::Vec2f v1(1.0, s1);
//        geo::Vec2f v2(1.0, s2);

//        if (v1.normalized().dot(v2.normalized()) > 0.8)
//            return false;

//        *x = x_pivot_;
//        *y = y_pivot_;

//        return true;
    }

    double x_pivot_, y_pivot_;

    dml::LineEstimator e_front;
    dml::LineEstimator e_back;
    dml::RingBuffer<int> i_buffer_;
};

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

        cv::Mat depth = rgbd_image->getDepthImage();

        rgbd::View view(*rgbd_image, depth.cols);
        const geo::DepthCamera& rasterizer = view.getRasterizer();

        cv::Mat bla = depth.clone();
        cv::Mat obstacle_map(500, 500, CV_32FC1, 0.0);

        unsigned int edge_window_size = 30;

        int width = depth.cols;
        int height = depth.rows;

        Timer timer;

        for(int x = 0; x < width; x += 1)
        {

            EdgeDetector edge_det;
            edge_det.setWindowSize(edge_window_size);

            for(int y = height - 1; y >= 0; --y)
            {
                float d = depth.at<float>(y, x);
                if (d == 0 || d != d)
                    continue;

                geo::Vector3 p_sensor = rasterizer.project2Dto3D(x, y) * d;
                geo::Vector3 p_floor = sensor_pose * p_sensor;

                int y_edge;
                edge_det.addPoint(p_floor.y, p_floor.z, y);

                if (p_floor.z > 0.2)
                {
                    cv::Point2i p(p_floor.x * 50 + obstacle_map.rows / 2,
                                  obstacle_map.cols / 2 - p_floor.y * 50);
                    if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                        obstacle_map.at<float>(p) = 0.5;
                }
                else if (edge_det.hasEdge(&y_edge))
                {
                    bla.at<float>(y_edge, x) = 10;
                    //                        cv::Point2i p_edge(x_edge * 100, y_edge * 100);
                    //                        cv::circle(cross_section, p_edge, 10, cv::Scalar(1));

                    float d_edge = depth.at<float>(y_edge, x);
                    geo::Vector3 p_edge_sensor = rasterizer.project2Dto3D(x, y_edge) * d_edge;
                    geo::Vector3 p_edge_floor = sensor_pose * p_edge_sensor;

                    cv::Point2i p(p_edge_floor.x * 50 + obstacle_map.rows / 2,
                                  obstacle_map.cols / 2 - p_edge_floor.y * 50);
                    if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                        obstacle_map.at<float>(p) = 1;
                }
            }
        }

        timer.stop();
        std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

        // - - - - - - - - - - - - - - - - - -

        // Visualize
//        cv::imshow("depth", depth2 / 10);
//        cv::imshow("ranges", canvas);
//        cv::imshow("reprojection", reprojection / 10);
        cv::imshow("bla", bla / 10);
        cv::imshow("obstacle map", obstacle_map);

//        cv::imshow("cross_section", cross_section);
        cv::waitKey(3);
    }

    return 0;
}
