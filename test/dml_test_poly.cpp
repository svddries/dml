#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>

#include <geolib/ros/tf_conversions.h>

#include <queue>

typedef std::vector<unsigned int> ScanSegment;

// ----------------------------------------------------------------------------------------------------

void cluster(const std::vector<float>& sensor_ranges, std::vector<ScanSegment>& segments)
{
    std::size_t num_beams = sensor_ranges.size();

    float segment_depth_threshold_ = 0.2;
    int max_gap_size_ = 3;
    int min_segment_size_pixels_ = 10;

    // Find first valid value
    ScanSegment current_segment;
    for(unsigned int i = 0; i < num_beams; ++i)
    {
        if (sensor_ranges[i] > 0)
        {
            current_segment.push_back(i);
            break;
        }
    }

    if (current_segment.empty())
    {
        std::cout << "No residual point cloud!" << std::endl;
        return;
    }

    int gap_size = 0;

    for(unsigned int i = current_segment.front(); i < num_beams; ++i)
    {
        float rs = sensor_ranges[i];

        if (rs == 0 || std::abs(rs - sensor_ranges[current_segment.back()]) > segment_depth_threshold_)
        {
            // Found a gap
            ++gap_size;

            if (gap_size >= max_gap_size_)
            {
                i = current_segment.back() + 1;

                if (current_segment.size() >= min_segment_size_pixels_)
                    segments.push_back(current_segment);

                current_segment.clear();

                // Find next good value
                while(sensor_ranges[i] == 0 && i < num_beams)
                    ++i;

                current_segment.push_back(i);
            }
        }
        else
        {
            gap_size = 0;
            current_segment.push_back(i);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

struct Line
{
    Line(unsigned int i_start_, unsigned int i_end_) : i_start(i_start_), i_end(i_end_) {}
    unsigned int i_start;
    unsigned int i_end;
};

// ----------------------------------------------------------------------------------------------------

void extractLines(const std::vector<geo::Vec2>& points, double max_point_dist_sq,
                  unsigned int i1, unsigned int i2, std::vector<Line>& lines)
{
    const geo::Vec2& p1 = points[i1];
    const geo::Vec2& p2 = points[i2];

    geo::Vec2 l = (p2 - p1).normalized();

    int last_point_side = 0;

    int i_pivot_point = -1;
    double max_found_dist_sq = 0;

    // Calculate distances of all points to the line
    for(unsigned int i = i1; i <= i2; ++i)
    {
        const geo::Vec2& p = points[i];
        geo::Vec2 p1_p = p - p1;

        // Calculate distance of p to line (p1, p2)
        float dist_sq = (p1_p - (p1_p.dot(l) * l)).length2();

        if (dist_sq < max_point_dist_sq)
        {
            last_point_side = 0;
            continue;
        }

        int point_side = ((l.x * p1_p.y - l.y * p1_p.x) < 0) ? -1 : 1;

        if (point_side == last_point_side && dist_sq > max_found_dist_sq)
        {
            i_pivot_point = i;
            max_found_dist_sq = dist_sq;
        }

        last_point_side = point_side;
    }

    if (i_pivot_point < 0)
    {
        // No breaking point found, so add the line!
        if (i2 - i1 > 10)
            lines.push_back(Line(i1, i2));
    }
    else
    {
        extractLines(points, max_point_dist_sq, i1, i_pivot_point, lines);
        extractLines(points, max_point_dist_sq, i_pivot_point, i2, lines);
    }
}

// ----------------------------------------------------------------------------------------------------

void extractLines(const std::vector<geo::Vec2>& points, int init_set_size, double max_point_dist_sq, std::vector<Line>& lines)
{
    for(unsigned int i = 0; i < points.size() - init_set_size; i += init_set_size)
    {
        extractLines(points, max_point_dist_sq, i, i + init_set_size, lines);
    }

    // Todo: merge collinear line segments
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

        sensor_pose.R.setRPY(roll, 0, 0);

        std::cout << roll << " " << pitch << " " << yaw << std::endl;

        // - - - - - - - - - - - - - - - - - -

        cv::Mat depth = rgbd_image->getDepthImage();
        cv::Mat depth2 = depth.clone();

        rgbd::View view(*rgbd_image, depth.cols);
        const geo::DepthCamera& rasterizer = view.getRasterizer();

        cv::Mat canvas(depth.rows, depth.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat reprojection(400, depth.cols, CV_32FC1, 0.0);

        std::vector<float> ranges(depth.cols, 0);

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
            geo::Vec2 p = geo::Vec2(view.getRasterizer().project2Dto3DX(i), 1) * ranges[i];
            cv::Point p_canvas(p.x * 100 + canvas.cols / 2, canvas.rows - p.y * 100);

            if (p_canvas.x >= 0 && p_canvas.y >= 0 && p_canvas.x < canvas.cols && p_canvas.y < canvas.rows)
            {
//                cv::circle(canvas, p_canvas, 1, cv::Scalar(0, 255, 0));
//                canvas.at<cv::Vec3b>(p_canvas.y, p_canvas.x) = cv::Vec3b(0, 255, 0);
            }
        }

        std::vector<geo::Vec2> points(ranges.size());
        for(unsigned int i = 0; i < ranges.size(); ++i)
            points[i] = geo::Vec2(view.getRasterizer().project2Dto3DX(i), 1) * ranges[i];

        std::vector<Line> lines;
        extractLines(points, 50, 0.02 * 0.02, lines);

        for(std::vector<Line>::const_iterator it = lines.begin(); it != lines.end(); ++it)
        {
            const Line& line = *it;

            unsigned int i1 = line.i_start;
            unsigned int i2 = line.i_end;

            geo::Vec2 p1 = geo::Vec2(view.getRasterizer().project2Dto3DX(i1), 1) * ranges[i1];
            geo::Vec2 p2 = geo::Vec2(view.getRasterizer().project2Dto3DX(i2), 1) * ranges[i2];

            cv::Point p1_canvas(p1.x * 100 + canvas.cols / 2, canvas.rows - p1.y * 100);
            cv::Point p2_canvas(p2.x * 100 + canvas.cols / 2, canvas.rows - p2.y * 100);

            cv::line(canvas, p1_canvas, p2_canvas, cv::Scalar(0, 0, 255), 1);
            cv::circle(canvas, p1_canvas, 2, cv::Scalar(0, 0, 255), 2);
            cv::circle(canvas, p2_canvas, 2, cv::Scalar(0, 0, 255), 2);

        }

        // - - - - - - - - - - - - - - - - - -

        // Visualize
        cv::imshow("depth", depth2 / 10);
        cv::imshow("ranges", canvas);
        cv::imshow("reprojection", reprojection / 10);
        cv::waitKey(3);
    }

    return 0;
}
