#include <coral/marker.h>
#include <coral/transforms.h>
#include <coral/osg_make_ref.h>
#include <coral/resource_helpers.h>
#include <coral/scene_lock.h>
#include <osg/ShapeDrawable>

#include <coral/srv/marker.hpp>

using namespace coral;

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

void Marker::spawnThrough(rclcpp::Node* node, osg::Group* world, Buffer* buffer)
{
  Marker::world = world;
  Marker::node = node;
  Marker::buffer = buffer;
  using MarkerPtr = std::unique_ptr<Marker>;
  static std::vector<MarkerPtr> markers;

  static auto service = node->create_service<srv::Marker>("/coral/marker",[&]
                                                          (const srv::Marker::Request::SharedPtr request,
                                                          [[maybe_unused]] srv::Marker::Response::SharedPtr response)
  {
    if(request->topic.empty())
      return;

    const auto twin{std::find_if(markers.begin(), markers.end(), [&](const MarkerPtr &marker)
      {return marker->topic() == request->topic;})};

    if(twin != markers.end())
    {
      if(request->message.empty())
        markers.erase(twin);
      return;
    }

    const auto msg_ends_with = [request](const std::string &ending)
    {
        if (ending.size() > request->message.size()) return false;
        return std::equal(ending.rbegin(), ending.rend(), request->message.rbegin());
    };

    if(msg_ends_with("Path"))
      markers.push_back(std::make_unique<markers::Path>(request->topic, request->rgb));
    else if(msg_ends_with("Pose"))
      markers.push_back(std::make_unique<markers::Pose>(request->topic, request->rgb, false));
    else if(msg_ends_with("PoseStamped"))
      markers.push_back(std::make_unique<markers::Pose>(request->topic, request->rgb, true));
  });

  static auto refreshMarkers = node->create_wall_timer(std::chrono::milliseconds(100),[&]()
  {
    for(auto &marker: markers)
      marker->refresh();
  });
}

markers::Pose::Pose(const std::string &topic, const std::array<double, 3> &rgb, bool pose_stamped)
{
  base = Visual::fromShapes({osg::make_ref<osg::Cylinder>(osg::Vec3{0,0,length/2}, radius, length),
                             osg::make_ref<osg::Cone>(osg::Vec3{0,0,length}, radius*1.5, head)},
                            makeStateSet(rgb), osg::Matrix::identity()).frame();
  attachToWorld(true);

  if(pose_stamped)
  {
    sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(topic, 1, [&](geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        frame_id = msg->header.frame_id;
        pending = msg->pose;
  });
  }
  else
  {
    frame_id = WORLD_NAME;
    sub = node->create_subscription<geometry_msgs::msg::Pose>(topic, 1, [&](geometry_msgs::msg::Pose::SharedPtr msg)
    {
        pending = *msg;
  });
  }
}

void markers::Pose::refresh()
{
  if(!pending.has_value())
    return;

  auto Mw{osgMatFrom(pending->position, pending->orientation)};
  if(frame_id != WORLD_NAME)
  {
    const auto tr{buffer->lookup(frame_id)};
    if(!tr.has_value())
    {
      hide();
      return;
    }
    Mw = tr.value() * Mw;
  }
  // z to x
  const osg::Matrix xMz(osg::Quat{0, .5, 0, .5});
  [[maybe_unused]] const auto lock{coral_lock()};
  setMatrix(xMz * Mw);
}


markers::Path::Path(const std::string &topic, const RGB &rgb)
{
  base = Visual::fromShapes({}, makeStateSet(rgb)).frame();
  attachToWorld(false);

  sub = node->create_subscription<nav_msgs::msg::Path>(topic, 1, [&](nav_msgs::msg::Path::SharedPtr msg)
  {
      frame_id = msg->header.frame_id;
      pending = msg->poses;
});
}

void markers::Path::refresh()
{
  if(!pending.has_value())
    return;

  // check if the same path is already registered

  if(pending->size() < 2)
  {
    points.clear();
    base->removeChildren(0, base->getNumChildren());
    return;
  }

  bool update_segments{true};
  std::optional<osg::Matrix> M;
  if(frame_id != WORLD_NAME)
  {
    M = buffer->lookup(frame_id);
    if(!M.has_value())
      return;
  }

  const auto toWorld = [&M](const geometry_msgs::msg::Point &point) -> osg::Vec3d
  {
    auto p{osgVecFrom(point)};
    if(!M.has_value())
      return p;
    return M.value() * p;
  };

  if(points.size() != pending->size())
  {
    points.clear();
    points.reserve(pending->size());
    std::transform(pending->begin(), pending->end(), std::back_inserter(points),
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
      auto pose{pending->begin()};
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
  [[maybe_unused]] const auto lock{coral_lock()};

  const auto prev_size{base->getNumChildren()};
  const auto new_size{segments.size()};

  if(new_size < prev_size)
    base->removeChildren(new_size, prev_size-new_size);

  const auto createCylinder = [](const Line::Segment &segment)
  {
    auto cyl = osg::make_ref<osg::Cylinder>(segment.pose.getTrans(), radius, segment.length);
    cyl->setRotation(segment.pose.getRotate());
    return osg::make_ref<osg::ShapeDrawable>(cyl);
  };

  for(size_t i = 0; i < prev_size; ++i)
    base->setChild(i, createCylinder(segments[i]).get());
  for(size_t i = prev_size; i < new_size; ++i)
    base->addChild(createCylinder(segments[i]));
}
