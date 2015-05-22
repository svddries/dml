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

        tf::Matrix3x3 m;
        geo::convert(sensor_pose.R, m);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        sensor_pose.R.setRPY(roll, 0, 0);

        std::cout << roll << " " << pitch << " " << yaw << std::endl;

        // - - - - - - - - - - - - - - - - - -

        std::cout << sensor_pose << std::endl;

        cv::Mat depth = rgbd_image->getDepthImage();
        cv::Mat depth2 = depth.clone();

        rgbd::View view(*rgbd_image, depth.cols);
        const geo::DepthCamera& rasterizer = view.getRasterizer();

        cv::Mat canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat reprojection(400, depth.cols, CV_32FC1, 0.0);

        std::vector<float> ranges(depth.cols, 10);

        for(int x = 0; x < depth.cols; ++x)
        {
            for(int y = 0; y < depth.rows; ++y)
            {
                float d = depth.at<float>(y, x);
                if (d == 0 || d != d)
                    continue;

                geo::Vector3 p_sensor = rasterizer.project2Dto3D(x, y) * d;
                geo::Vector3 p_floor = sensor_pose * p_sensor;

                if (p_floor.z < 0.2) // simple floor filter
                    continue;

                geo::Vec2 p_canvas(p_floor.x * 100 + canvas.cols / 2, canvas.rows - p_floor.y * 100);
                if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
                {
                    int c = 10 + p_floor.z / 1.5 * 245;
//                    std::cout << p_floor.z << " " << c << std::endl;
                    cv::Vec3b& b = canvas.at<cv::Vec3b>(p_canvas.y, p_canvas.x);
                    if (b[0] < c)
                        b = cv::Vec3b(c, c, c);
                }

                int i = (rasterizer.getFocalLengthX() * p_floor.x + rasterizer.getOpticalTranslationX()) / p_floor.y + rasterizer.getOpticalCenterX();
                if (i >= 0 && i < ranges.size())
                {
                    ranges[i] = std::min<float>(ranges[i], p_floor.y);

                    cv::Point2i p_reprojection(i, (1.0 - p_floor.z / 2.0) * reprojection.rows);
                    if (p_reprojection.y >= 0 && p_reprojection.y < reprojection.rows)
                    {
                        float& d = reprojection.at<float>(p_reprojection);
                        if (d == 0 || p_floor.y < d)
                            d = p_floor.y;
                    }
                }

                if (i % 20 == 0)
                    depth2.at<float>(y, x) = depth.at<float>(y, x) + 1;
            }
        }

        for(unsigned int i = 0; i < ranges.size(); ++i)
        {
            geo::Vec2 p = geo::Vec2(view.getRasterizer().project2Dto3DX(i), 1) * ranges[i];
            cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);

            if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
            {
                cv::circle(canvas, p_canvas, 1, cv::Scalar(0, 255, 0));
//                canvas.at<cv::Vec3b>(p_canvas.y, p_canvas.x) = cv::Vec3b(0, 255, 0);
            }
        }

        // Visualize
        cv::imshow("depth", depth2 / 10);
        cv::imshow("ranges", canvas);
        cv::imshow("reprojection", reprojection / 10);
        cv::waitKey(3);
    }

    return 0;
}
