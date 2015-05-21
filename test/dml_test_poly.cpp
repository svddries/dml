#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

#include <geolib/ros/tf_conversions.h>

#include <queue>

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

        // - - - - - - - - - - - - - - - - - -

        std::cout << sensor_pose << std::endl;

        cv::Mat depth = rgbd_image->getDepthImage();

        //        for(int x = 0; x < depth.cols; ++x)
        //            for(int y = 0; y < depth.rows; ++y)
        //                depth.at<float>(y, x) = 1.0 / ((float)y / depth.rows + 1.0);

        rgbd::View view(*rgbd_image, depth.cols);

        cv::Mat canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));

        for(int x = 0; x < depth.cols; ++x)
        {

            float min_depth = 10;
            for(int y = 0; y < depth.rows - 240; ++y)
            {
                float d = depth.at<float>(y, x);
                if (d > 0 && d == d)
                {
                    min_depth = std::min(d, min_depth);
                }
            }

            geo::Vec2 p = geo::Vec2(view.getRasterizer().project2Dto3DX(x), 1) * min_depth;
            geo::Vec2 p_canvas(p.x * 50 + canvas.cols / 2, canvas.rows - p.y * 50);

            if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
            {
                canvas.at<cv::Vec3b>(p_canvas.y, p_canvas.x) = cv::Vec3b(255, 255, 255);
            }
        }

        // Visualize
        cv::imshow("depth", depth / 10);
        cv::imshow("ranges", canvas);
        cv::waitKey(3);
    }

    return 0;
}
