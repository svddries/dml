#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

#include <tue/profiling/timer.h>

int factor = 1;
float bla = 0.02;
float max_range = 4;
int min_segment_size = 10 / factor; // in pixels
int max_segment_size = 30 / factor; // in pixels
bool visualize = true;

// ----------------------------------------------------------------------------------------------------

struct EdgeFeature
{
    int x, y;
    geo::Vector3 point;
    int dx, dy;
};

// ----------------------------------------------------------------------------------------------------

struct EdgeMap
{
    EdgeMap(const rgbd::View& view_) : view(view_) {}

    void addEdgePoint(int x, int y, int dx, int dy)
    {
        geo::Vector3 p;
        if (!view.getPoint3D(x, y, p))
            return;

        features.push_back(EdgeFeature());
        EdgeFeature& f = features.back();
        f.x = x;
        f.y = y;
        f.dx = dx;
        f.dy = dy;
        f.point = p;
    }

    const rgbd::View& view;
    std::vector<EdgeFeature> features;
};

// ----------------------------------------------------------------------------------------------------

int findEdgeHorizontal(int y, int x_start, int x_end, const rgbd::View view, EdgeMap& edge_map)
{
    if (x_end - x_start < min_segment_size)
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

    // Add edge point
    edge_map.addEdgePoint(x_edge, y, 1, 0);

    // Recursively find edges
    findEdgeHorizontal(y, x_start, x_edge, view, edge_map);

    return x_edge;
}

// ----------------------------------------------------------------------------------------------------


int findEdgeVertical(int x, int y_start, int y_end, const rgbd::View view, EdgeMap& edge_map)
{
    if (y_end - y_start < min_segment_size)
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

    // Add edge point
    edge_map.addEdgePoint(x, y_edge, 0, 1);

    // Recursively find edges
    findEdgeVertical(x, y_start, y_edge, view, edge_map);

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

        cv::Mat depth_original = image.getDepthImage();
        if (!depth_original.data)
            continue;

        // - - - - - - - - - - - - - - - - - -
        // Downsample depth image

        cv::Mat depth;
        if (factor == 1)
        {
            depth = depth_original;
        }
        else
        {
            depth = cv::Mat(depth_original.rows / factor, depth_original.cols / factor, CV_32FC1, 0.0);

            for(int y = 0; y < depth.rows; ++y)
            {
                for(int x = 0; x < depth.cols; ++x)
                {
                    depth.at<float>(y, x) = depth_original.at<float>(y * factor, x * factor);
                }
            }
        }

        // - - - - - - - - - - - - - - - - - -

        rgbd::View view(image, depth.cols);

        tue::Timer timer;
        timer.start();

        EdgeMap edge_map(view);

        // - - - - - - - - - - - - HORIZONTAL - - - - - - - - - - - -

        for(int y = 0; y < depth.rows; ++y)
        {
            int x = 0;
            int x_start;
            float d_last;
            int x_last;

            // Find first valid value
            for(; x < depth.cols; ++x)
            {
                float d = depth.at<float>(y, x);
                if (d == d && d > 0)
                {
                    x_start = x;
                    x_last = x;
                    d_last = d;
                    break;
                }
            }

            for(; x < depth.cols; ++x)
            {
                int x_end = x;

                float d = depth.at<float>(y, x);
                if (d != d || d == 0)
                    continue;

                bool check_edge = false;
                int new_x_start;

                if (std::abs(d - d_last) > d * 0.1)
                {
                    if (d < d_last && d < max_range)
                        edge_map.addEdgePoint(x, y, 1, 0);
                    else if (d_last < max_range)
                        edge_map.addEdgePoint(x_last, y, 1, 0);

                    new_x_start = x;
                    x_end--;
                    check_edge = true;
                }
                else if (x_end - x_start >= max_segment_size)
                {
                    check_edge = true;
                    new_x_start = (x_start + x_end) / 2;
                }

                if (check_edge)
                {
                    int x_edge = findEdgeHorizontal(y, x_start, x_end, view, edge_map);

                    if (x_edge >= 0)
                        x = x_edge;

                    x_start = new_x_start;
                }

                d_last = depth.at<float>(y, x);
                x_last = x;
            }
        }

        // - - - - - - - - - - - - VERTICAL - - - - - - - - - - - -

        for(int x = 0; x < depth.cols; ++x)
        {
            int y = 0;
            int y_start;
            float d_last;
            int y_last;

            // Find first valid value
            for(; y < depth.rows; ++y)
            {
                float d = depth.at<float>(y, x);
                if (d == d && d > 0)
                {
                    y_start = y;
                    y_last = y;
                    d_last = d;
                    break;
                }
            }

            for(; y < depth.rows; ++y)
            {
                int y_end = y;

                float d = depth.at<float>(y, x);
                if (d != d || d == 0)
                    continue;

                bool check_edge = false;
                int new_y_start;

                if (std::abs(d - d_last) > d * 0.1)
                {
                    if (d < d_last && d < max_range)
                        edge_map.addEdgePoint(x, y, 0, 1);
                    else if (d_last < max_range)
                        edge_map.addEdgePoint(x, y_last, 0, -1);

                    new_y_start = y;
                    y_end--;
                    check_edge = true;
                }
                else if (y_end - y_start >= max_segment_size)
                {
                    check_edge = true;
                    new_y_start = (y_start + y_end) / 2;
                }

                if (check_edge)
                {
                    int y_edge = findEdgeVertical(x, y_start, y_end, view, edge_map);

                    if (y_edge >= 0)
                        y = y_edge;

                    y_start = new_y_start;
                }

                d_last = depth.at<float>(y, x);
                y_last = y;
            }
        }

        std::cout << std::endl;
        std::cout << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;
        std::cout << "Number of edge points = " << edge_map.features.size() << std::endl;

        if (visualize)
        {
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

            for(std::vector<EdgeFeature>::const_iterator it = edge_map.features.begin(); it != edge_map.features.end(); ++it)
            {
                const EdgeFeature& f = *it;

                if (f.dx == -1)
                    canvas.at<cv::Vec3b>(f.y, f.x) = cv::Vec3b(255, 0, 0);
                else if (f.dx == 1)
                    canvas.at<cv::Vec3b>(f.y, f.x) = cv::Vec3b(255, 255, 0);
                else if (f.dy == -1)
                    canvas.at<cv::Vec3b>(f.y, f.x) = cv::Vec3b(0, 255, 0);
                else if (f.dy == 1)
                    canvas.at<cv::Vec3b>(f.y, f.x) = cv::Vec3b(0, 255, 255);
            }

            cv::imshow("edges", canvas);

            cv::waitKey(3);
        }

    }

    ros::spin();

    return 0;
}
