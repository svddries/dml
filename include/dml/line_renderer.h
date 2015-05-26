#ifndef DML_LINE_RENDERER_H_
#define DML_LINE_RENDERER_H_

#include <geolib/datatypes.h>

namespace geo
{
class Mesh;
}

struct Line
{
    geo::Vec2 start;
    geo::Vec2 end;
};

class LineRenderer
{

public:

    LineRenderer();

    ~LineRenderer();

    void render(const geo::Mesh& mesh, const geo::Pose3D& pose, float z_min, float z_max, std::vector<Line>& lines);

private:

    float z_min_;
    float z_max_;

};

#endif
