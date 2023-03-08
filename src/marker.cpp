#include <coral/marker.h>
#include <coral/transforms.h>
#include <coral/osg_make_ref.h>
#include <coral/resource_helpers.h>
#include <osg/Shape>

using namespace coral;

osg::Group* Marker::world;

namespace
{
// some math to approximate a path with piecewise linear
using PointIt = std::vector<osg::Vec3d>::const_iterator;

/// a 3D line described by a point x0 and a direction u
struct Line
{  
  struct Segment
  {
    osg::Matrix pose;
    double length;
    osg::Vec3 last;
  };

  osg::Vec3d x0;
  osg::Vec3d u;
  PointIt start, end;

  /// returns where the point p is projected on this line as alpha such as proj = x0 + alpha.u
  inline auto abscissa(const osg::Vec3d &p) const
  {
    return (p-x0)*u;
  }
  /// returns squared distance between p and this 3D line
  inline auto sqDistance(const osg::Vec3d &p) const
  {
    return ((p-x0)-u*abscissa(p)).length2();
  }

  /// returns the segment (pose + length) from this line as close as possible to [start,end[
  Segment build() const
  {
    std::vector<double> proj;
    proj.reserve(end-start);
    std::transform(start, end, std::back_inserter(proj),
                   [&](const auto &p){return abscissa(p);});
    const auto [am,aM] = std::minmax_element(proj.begin(), proj.end()); {}

    Segment cyl;
    const auto p1{x0 + u*(*am)};
    cyl.last = x0 + u*(*aM);

    const auto z{osg::Vec3d{0,0,1}};

    cyl.length = (p1-cyl.last).length();
    cyl.pose.setTrans((p1+cyl.last)/2);

    const auto axis{z^u};
    const auto angle{atan2(axis.length(), z*u)};
    cyl.pose.setRotate(osg::Quat(angle, axis));
    return cyl;
  }

  /// find the center and direction of a 3D line closest to points between [start, end[
  void fit(PointIt start, PointIt end, std::optional<osg::Vec3> prev)
  {
    this->start = start;
    this->end = end;

    x0 = prev.value_or(std::accumulate(start, end, osg::Vec3d())/(end-start));
    u = *(end-1) - *start;
    for(auto p = start; p != end; ++p)
      u += (*p-x0)*((*p-x0)*u);
    u.normalize();
  }
};



std::vector<Line::Segment> piecewiseLinear(const std::vector<osg::Vec3d> &path, double threshold)
{
  if(path.size() < 2)
    return {};

  auto end{path.begin()+1};

  std::vector<Line::Segment> segments;
  std::vector<double> distances;
  distances.reserve(path.size());
  Line line;
  std::optional<osg::Vec3> prev;


  while(true)
  {
    auto start{end-1};
    end++;
    if(end >= path.end())
      break;

    while(true)
    {
      line.fit(start, end, prev);
      distances.clear();
      std::transform(start, end, std::back_inserter(distances),
                     [&line](const auto &point)
      {return line.sqDistance(point);});
      const auto err{sqrt(*std::max_element(distances.begin(), distances.end()))};

      if(err > threshold || end == path.end())
        break;
      end++;
    }
    prev = segments.emplace_back(line.build()).last;
  }
  return segments;
}
}

Marker::Marker(const Visual &visual, bool moving) : Visual(visual)
{
  configure(false, moving ? osg::Object::DYNAMIC : osg::Object::STATIC);
  world->addChild(pose);
}

Goal::Goal(const osg::Vec4 &rgba) : Marker({osg::make_ref<osg::Cylinder>(osg::Vec3{0,0,length/2}, radius, length),
                                           osg::make_ref<osg::Cone>(osg::Vec3{0,0,length}, radius*1.5, head)},
{},Visual::makeStateSet(rgba),true)
{

}

void Goal::update(const Buffer &buffer, const geometry_msgs::msg::PoseStamped &msg)
{
  auto Mw{osgMatFrom(msg.pose.position, msg.pose.orientation)};
  if(msg.header.frame_id != "world")
  {
    const auto tr{buffer.lookup(msg.header.frame_id)};
    if(!tr.has_value())
    {
      hide();
      return;
    }
    Mw = osgMatFrom(tr->translation, tr->rotation) * Mw;
  }
  pose->setMatrix(Mw);
}

void Path::reset(size_t dim)
{
  segments.clear();
  segments.reserve(dim);
}

void Path::update(const Buffer &buffer, const nav_msgs::msg::Path &path)
{
  // check if the same path is already registered
  const auto &poses{path.poses};

  if(poses.size() < 2)
  {
    reset();
    return;
  }
  bool update_segments{true};
  const auto world_path{path.header.frame_id == "world"};

  std::optional<osg::Matrix> M;
  if(!world_path)
  {
    const auto pose{buffer.lookup(path.header.frame_id)};
    if(!pose.has_value())
      return;
    M = osgMatFrom(pose->translation, pose->rotation);
  }

  const auto toWorld = [&M](const geometry_msgs::msg::Point &point) -> osg::Vec3d
  {
    auto p{osgVecFrom(point)};
    if(!M.has_value())
      return p;
    return M.value() * p;
  };

  if(points.size() != poses.size())
  {
    points.clear();
    points.reserve(poses.size());
    std::transform(poses.begin(), poses.end(), std::back_inserter(points),
                   [&](const geometry_msgs::msg::PoseStamped &pose)
    {
      return toWorld(pose.pose.position);
    });
  }
  else
  {
    update_segments = false;
    {
      auto point{points.begin()};
      auto pose{poses.begin()};
      for(; point != points.end(); point++, pose++)
      {
        if(const auto abs_point{toWorld(pose->pose.position)};
           abs_point != *point)
        {
          update_segments = true;
          *point = abs_point;
        }
      }
    }
  }

  if(!update_segments)
    return;

  // compute segments from new points
  const auto segments{piecewiseLinear(points, radius)};
  reset(segments.size());
  for(const auto &segment: segments)
    this->segments.emplace_back(Visual::Shapes{osg::make_ref<osg::Cylinder>(osg::Vec3{}, radius, segment.length)}, segment.pose, color);
}
