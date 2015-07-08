#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

#include <geolib/ros/tf_conversions.h>

#include <queue>

// ----------------------------------------------------------------------------------------------------

class BeamCalculator
{

public:

    BeamCalculator(double w, unsigned int num_beams) : half_num_beams_(num_beams / 2)
    {
        fx_ = 2 * num_beams / w;

        rays_.resize(num_beams);
        for(unsigned int i = 0; i < num_beams; ++i)
            rays_[i] = geo::Vec2(((double)(i) - half_num_beams_) / fx_, 1);
    }

    inline int CalculateBeam(double x, double depth)
    {
        return (fx_ * x) / depth + half_num_beams_;
    }

    inline geo::Vec2 CalculatePoint(int i, double depth)
    {
        return rays_[i] * depth;
    }

    void RenderModel(const std::vector<geo::Vec2>& model, const geo::Transform2& pose, std::vector<double>& ranges)
    {
        std::vector<geo::Vec2> t_vertices(model.size());
        for(unsigned int i = 0; i < model.size(); ++i)
            t_vertices[i] = pose * model[i];

        int nbeams = num_beams();

        for(unsigned int i = 0; i < model.size(); ++i)
        {
            unsigned int j = (i + 1) % model.size();
            const geo::Vec2& p1 = t_vertices[i];
            const geo::Vec2& p2 = t_vertices[j];

            std::cout << p1 << " - " << p2 << std::endl;

            int i1 = CalculateBeam(p1.x, p1.y) + 1;
            int i2 = CalculateBeam(p2.x, p2.y);

            if (i2 < i1 || i2 < 0 || i1 >= nbeams)
                continue;

            i1 = std::max(0, i1);
            i2 = std::min(i2, nbeams - 1);

            geo::Vec2 s = p2 - p1;

            for(int i_beam = i1; i_beam <= i2; ++i_beam)
            {
                const geo::Vec2& r = rays_[i_beam];

                // calculate depth of intersection between line (p1, p2) and r
                double d = (p1.x * s.y - p1.y * s.x) / (r.x * s.y - r.y * s.x);

                double& depth_old = ranges[i_beam];
                if (d > 0 && (d < depth_old || depth_old == 0))
                    depth_old = d;
            }
        }
    }

    inline unsigned int num_beams() const { return rays_.size(); }

    inline const std::vector<geo::Vec2>& rays() const { return rays_; }

private:

    double fx_;
    unsigned int half_num_beams_;
    std::vector<geo::Vec2> rays_;
};

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // MODEL
    std::vector<geo::Vec2> model;
    model.push_back(geo::Vec2(-0.6, -0.4));
    model.push_back(geo::Vec2( 0.6, -0.4));
    model.push_back(geo::Vec2( 0.6,  0.4));
    model.push_back(geo::Vec2(-0.6,  0.4));


    ros::init(argc, argv, "dml_test_poly");
    ros::NodeHandle nh;

    rgbd::Client client;
    client.intialize("/amigo/top_kinect/rgbd");

    tf::TransformListener tf_listener;

    std::queue<rgbd::ImageConstPtr> image_buffer;

    BeamCalculator beam_calculator(2, 100);

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
        cv::Mat depth2 = depth.clone();

        rgbd::View view(*rgbd_image, depth.cols);
        const geo::DepthCamera& rasterizer = view.getRasterizer();

        cv::Mat canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat reprojection(400, beam_calculator.num_beams(), CV_32FC1, 0.0);

        std::vector<float> ranges(beam_calculator.num_beams(), 0);

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

                int i = beam_calculator.CalculateBeam(p_floor.x, p_floor.y);
                if (i >= 0 && i < ranges.size())
                {
                    float& r = ranges[i];
                    if (r == 0 || p_floor.y < r)
                        r = p_floor.y;

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
            geo::Vec2 p = beam_calculator.CalculatePoint(i, ranges[i]);
            cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);

            if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
                cv::circle(canvas, p_canvas, 1, cv::Scalar(0, 255, 0));
        }

        std::vector<double> model_ranges(ranges.size(), 0);

        double alpha = 1;
        double cos_alpha = cos(alpha);
        double sin_alpha = sin(alpha);
        geo::Mat2 rot(cos_alpha, -sin_alpha, sin_alpha, cos_alpha);

        beam_calculator.RenderModel(model, geo::Transform2(rot, geo::Vec2(0, 10)), model_ranges);

        int i_beam = ranges.size() / 2;
        double ds = ranges[i_beam];
        double dr = model_ranges[i_beam];

        if (ds > 0 && dr > 0)
        {
            model_ranges.resize(ranges.size(), 0);
            beam_calculator.RenderModel(model, geo::Transform2(rot, geo::Vec2(0, 10 + ds - dr)), model_ranges);

            for(unsigned int i = 0; i < model_ranges.size(); ++i)
            {
                std::cout << i << ": " << model_ranges[i] << std::endl;

                geo::Vec2 p = beam_calculator.CalculatePoint(i, model_ranges[i]);
                cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);

                if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
                    cv::circle(canvas, p_canvas, 1, cv::Scalar(0, 0, 255));
            }
        }

//        std::vector<geo::Vec2> points(ranges.size());
//        for(unsigned int i = 0; i < ranges.size(); ++i)
//            points[i] = geo::Vec2(view.getRasterizer().project2Dto3DX(i), 1) * ranges[i];

        // - - - - - - - - - - - - - - - - - -

        // Visualize
        cv::imshow("depth", depth2 / 10);
        cv::imshow("ranges", canvas);
        cv::imshow("reprojection", reprojection / 10);
        cv::waitKey(3);
    }

    return 0;
}