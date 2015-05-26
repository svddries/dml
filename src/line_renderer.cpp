#include "dml/line_renderer.h"

#include <geolib/Mesh.h>

// ----------------------------------------------------------------------------------------------------

LineRenderer::LineRenderer()
{
}

// ----------------------------------------------------------------------------------------------------

LineRenderer::~LineRenderer()
{
}

// ----------------------------------------------------------------------------------------------------

void LineRenderer::render(const geo::Mesh& mesh, const geo::Pose3D& pose, float z_min, float z_max, std::vector<Line>& lines)
{

    const std::vector<geo::TriangleI>& triangles = mesh.getTriangleIs();
    const std::vector<geo::Vector3>& points = mesh.getPoints();

    // transform Z-coordinates of all vertices
    std::vector<double> zs_t(points.size());
    geo::Vector3 Rz = pose.R.getRow(2);
    double z_offset = pose.t.z;
    for(unsigned int i = 0; i < points.size(); ++i)
        zs_t[i] = Rz.dot(points[i]) + z_offset;

    geo::Vector3 Rx = pose.getBasis().getRow(0);
    geo::Vector3 Ry = pose.getBasis().getRow(1);

    // Iterate over all triangles
    for(std::vector<geo::TriangleI>::const_iterator it_tri = triangles.begin(); it_tri != triangles.end(); ++it_tri)
    {
        double z1 = zs_t[it_tri->i1_];
        double z2 = zs_t[it_tri->i2_];
        double z3 = zs_t[it_tri->i3_];

        bool p1_under = z1 < z_min; bool p1_above = z1 > z_max;
        bool p2_under = z2 < z_min; bool p2_above = z2 > z_max;
        bool p3_under = z3 < z_min; bool p3_above = z3 > z_max;

        int n_under = p1_under + p2_under + p3_under;
        int n_above = p1_above + p2_above + p3_above;

        // Skip if all points below z_min or above z_max
        if (n_under == 3 || n_above == 3)
            continue;

        if (n_under == 0 && n_above == 0)
        {
            // All points are between z_min and z_max, so just draw the triangle

        }

        // ...

        if (p1_under || p2_under || p3_under || p1_above || p2_above || p3_above)
        {
            // Not all points are between z_min and z_max
        }
        else
        {

        }
    }
}

