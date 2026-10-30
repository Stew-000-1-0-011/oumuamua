#include "task_commander/pickup.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace task_commander {
namespace {
constexpr double infinity=std::numeric_limits<double>::infinity();
bool finite(const Pose2& p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw); }
bool valid_box(const Box& b) {
    return std::isfinite(b.xmin)&&std::isfinite(b.ymin)&&std::isfinite(b.xmax)&&std::isfinite(b.ymax)
        &&b.xmin<b.xmax&&b.ymin<b.ymax;
}
bool inside(Point2 p,const Box& b) { return p[0]>=b.xmin&&p[0]<=b.xmax&&p[1]>=b.ymin&&p[1]<=b.ymax; }
Polygon rectangle(const Box& b) {return {{b.xmin,b.ymin},{b.xmax,b.ymin},{b.xmax,b.ymax},{b.xmin,b.ymax}};}
double cross(Point2 a,Point2 b,Point2 c) {return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);}
bool intersects(Point2 a,Point2 b,Point2 c,Point2 d) {
    if(std::max(a[0],b[0])<std::min(c[0],d[0])||std::max(c[0],d[0])<std::min(a[0],b[0])
       ||std::max(a[1],b[1])<std::min(c[1],d[1])||std::max(c[1],d[1])<std::min(a[1],b[1]))return false;
    return cross(a,b,c)*cross(a,b,d)<=0&&cross(c,d,a)*cross(c,d,b)<=0;
}
bool contains(const Polygon& poly,Point2 p) {
    bool in=false;
    for(std::size_t i=0;i<poly.size();++i) {
        const auto a=poly[i],b=poly[(i+1)%poly.size()];
        if((a[1]>p[1])!=(b[1]>p[1]) && p[0]<a[0]+(p[1]-a[1])*(b[0]-a[0])/(b[1]-a[1]))in=!in;
    }
    return in;
}
double segment_distance(Point2 p,Point2 a,Point2 b) {
    const double dx=b[0]-a[0],dy=b[1]-a[1],len=dx*dx+dy*dy;
    const double t=len>0?std::clamp(((p[0]-a[0])*dx+(p[1]-a[1])*dy)/len,0.,1.):0.;
    return std::hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);
}
Polygon transformed(const Polygon& poly,const Pose2& pose) {
    Polygon out;out.reserve(poly.size());const double c=std::cos(pose.yaw),s=std::sin(pose.yaw);
    for(auto p:poly)out.push_back({pose.x+c*p[0]-s*p[1],pose.y+s*p[0]+c*p[1]});
    return out;
}
double field_clearance(const Polygon& poly,const Field& field) {
    double best=infinity;
    for(auto p:poly)best=std::min({best,p[0]-field.bounds.xmin,field.bounds.xmax-p[0],p[1]-field.bounds.ymin,field.bounds.ymax-p[1]});
    if(best<=0)return best;
    for(const auto& box:field.obstacles)best=std::min(best,polygon_box_clearance(poly,box));
    return best;
}
Polygon clip(const Polygon& polygon,int axis,double boundary,bool keep_less) {
    Polygon out;
    for(std::size_t i=0;i<polygon.size();++i) {
        auto a=polygon[i],b=polygon[(i+1)%polygon.size()];
        const bool ina=keep_less?a[axis]<=boundary:a[axis]>=boundary;
        const bool inb=keep_less?b[axis]<=boundary:b[axis]>=boundary;
        if(ina)out.push_back(a);
        if(ina!=inb) {
            const double t=(boundary-a[axis])/(b[axis]-a[axis]);
            out.push_back({a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])});
        }
    }
    return out;
}
bool valid_polygon(const Polygon& poly) {
    if(poly.size()<3)return false;
    double area=0;
    for(std::size_t i=0;i<poly.size();++i) {
        auto a=poly[i],b=poly[(i+1)%poly.size()];
        if(!std::isfinite(a[0])||!std::isfinite(a[1]))return false;
        area+=a[0]*b[1]-b[0]*a[1];
    }
    return std::abs(area)>1e-10;
}
Field padded(Field field,const PickupSettings& s) {
    field.bounds.xmin+=s.bounds_padding;field.bounds.ymin+=s.bounds_padding;
    field.bounds.xmax-=s.bounds_padding;field.bounds.ymax-=s.bounds_padding;
    // Fixed obstacles (walls and shelf) are expanded conservatively; notes
    // are appended afterwards, with their independent driving margin.
    for(auto& b:field.obstacles) { b.xmin-=s.wall_padding;b.ymin-=s.wall_padding;b.xmax+=s.wall_padding;b.ymax+=s.wall_padding; }
    for(auto& b:field.tall_obstacles) { b.xmin-=s.wall_padding;b.ymin-=s.wall_padding;b.xmax+=s.wall_padding;b.ymax+=s.wall_padding; }
    return field;
}
std::vector<Pose2> with_start(Pose2 start,const std::vector<Pose2>& points) {
    std::vector<Pose2> out{start};out.insert(out.end(),points.begin(),points.end());return out;
}
// Match polyline_path: omit exact repeated poses, reject in-place turns.
bool usable(const std::vector<Pose2>& path) {
    for(std::size_t i=1;i<path.size();++i)
        if(std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y)<=1e-6
           &&std::abs(std::remainder(path[i].yaw-path[i-1].yaw,2*std::numbers::pi))>1e-6)return false;
    return true;
}
}

auto PickupLayout::box(std::size_t id) const -> Box {const double h=size/2;return {x.at(id)-h,y-h,x.at(id)+h,y+h};}
auto validate_layout(const PickupLayout& layout,const Field& field) -> PickupStatus {
    if(!valid_box(field.bounds)||layout.x.empty()||!std::isfinite(layout.y)||!std::isfinite(layout.size)||layout.size<=0)
        return std::unexpected("notes_x must be nonempty; note geometry and bounds must be finite and valid");
    for(std::size_t i=0;i<layout.x.size();++i) {
        if(!std::isfinite(layout.x[i]))return std::unexpected("notes_x must contain finite centres");
        const auto box=layout.box(i);
        if(!inside({box.xmin,box.ymin},field.bounds)||!inside({box.xmax,box.ymax},field.bounds))return std::unexpected("note lies outside field");
        for(std::size_t j=0;j<i;++j)if(std::abs(layout.x[i]-layout.x[j])<layout.size+1e-9)return std::unexpected("notes overlap or touch");
        for(const auto& wall:field.obstacles)if(polygon_box_clearance(rectangle(box),wall)<=0)return std::unexpected("note intersects a fixed obstacle");
    }
    return {};
}

auto polygon_box_clearance(const Polygon& poly,const Box& box) -> double {
    if(poly.empty())return infinity;
    const auto rect=rectangle(box);
    for(auto p:poly)if(inside(p,box))return 0;
    for(auto p:rect)if(contains(poly,p))return 0;
    double best=infinity;
    for(std::size_t i=0;i<poly.size();++i) {
        auto a=poly[i],b=poly[(i+1)%poly.size()];
        best=std::min(best,std::hypot(std::max({box.xmin-a[0],0.,a[0]-box.xmax}),std::max({box.ymin-a[1],0.,a[1]-box.ymax})));
        for(std::size_t j=0;j<4;++j) {
            if(intersects(a,b,rect[j],rect[(j+1)%4]))return 0;
            best=std::min(best,segment_distance(rect[j],a,b));
        }
    }
    return best;
}

auto without_contact(const Polygon& poly,const Box& contact) -> std::vector<Polygon> {
    // The complement of a rectangle is the union of four half-planes. Keep
    // all their polygon intersections, including boundary edges. This never
    // exempts nonselected lower geometry or another pad.
    std::vector<Polygon> out;
    for(auto part:{clip(poly,0,contact.xmin,true),clip(poly,0,contact.xmax,false),
                   clip(poly,1,contact.ymin,true),clip(poly,1,contact.ymax,false)})
        if(valid_polygon(part))out.push_back(std::move(part));
    return out;
}

auto shape_sweep_clearance(const Field& field,const Polygon& poly,const std::vector<Pose2>& path) -> double {
    if(path.empty()||poly.empty())return -infinity;
    for(const auto& p:path)if(!finite(p))return -infinity;
    double radius=0;for(auto p:poly)radius=std::max(radius,std::hypot(p[0],p[1]));
    double worst=field_clearance(transformed(poly,path.front()),field);
    for(std::size_t i=1;i<path.size();++i) {
        const auto a=path[i-1],b=path[i];
        const double angle=std::remainder(b.yaw-a.yaw,2*std::numbers::pi);
        const double motion=std::hypot(b.x-a.x,b.y-a.y)+radius*std::abs(angle);
        if(!std::isfinite(motion)||motion>100)return -infinity;
        const auto n=std::max(1,static_cast<int>(std::ceil(motion/.004)));
        double previous=field_clearance(transformed(poly,a),field);
        for(int k=1;k<=n;++k) {
            const double u=static_cast<double>(k)/n;
            Pose2 p{a.x+u*(b.x-a.x),a.y+u*(b.y-a.y),a.yaw+u*angle};
            const double current=field_clearance(transformed(poly,p),field);
            worst=std::min(worst,std::min(previous,current)-motion/(2*n));previous=current;
        }
    }
    return worst;
}

auto pickup_stage_clearance(const PickupLayout& layout,const Field& fixed,const PickupGeometry& g,const PickupSettings& s,
                            const PickupRequest& req,const std::vector<Pose2>& path,bool lowered,bool captured) -> double {
    Field others=padded(fixed,s),all=others;
    for(std::size_t i=0;i<layout.x.size();++i) {
        const bool removed=!req.removed.empty()&&req.removed.at(i);
        if(removed||(captured&&i==req.block))continue;
        all.obstacles.push_back(layout.box(i));
        if(i!=req.block)others.obstacles.push_back(layout.box(i));
    }
    double gap=shape_sweep_clearance(all,g.raised,path);
    // Raising only clears low walls/notes. Arms and any held box still need
    // their full planar envelope against the tall shelf and outer boundary.
    gap=std::min(gap,shape_sweep_clearance({others.bounds,others.tall_obstacles},g.lowered,path));
    if(lowered) {
        gap=std::min(gap,shape_sweep_clearance(others,g.lowered,path));
        Field target{others.bounds,{layout.box(req.block)}};
        for(const auto& part:without_contact(g.lowered,g.contacts.at(req.gripper)))
            gap=std::min(gap,shape_sweep_clearance(target,part,path));
    }
    return gap;
}

auto plan_pickup(const PickupLayout& layout,const Field& fixed,const PickupGeometry& g,const PickupSettings& s,const PickupRequest& req)
    -> std::expected<PickupPlan,std::string> {
    if(auto valid=validate_layout(layout,padded(fixed,s));!valid)return std::unexpected(valid.error());
    if(req.block>=layout.x.size()||req.gripper>=g.contacts.size()||req.unavailable.size()!=layout.x.size()
       ||(!req.removed.empty()&&req.removed.size()!=layout.x.size()))return std::unexpected("invalid pickup IDs or state size");
    if(req.unavailable[req.block])return std::unexpected("box is already held or uncertain; reconcile and reset first");
    if(!finite(req.start)||!std::isfinite(req.tool[0])||!std::isfinite(req.tool[1])||std::hypot(req.tool[0],req.tool[1])<1e-6)
        return std::unexpected("invalid current pose or suction centre offset");
    if(!valid_polygon(g.raised)||!valid_polygon(g.lowered)||!valid_box(g.contacts[req.gripper]))return std::unexpected("invalid pickup footprint/contact region");
    for(double v:{s.yaw,s.transit_y,s.standoff,s.margin,s.wall_padding,s.bounds_padding,s.yaw_resolution})
        if(!std::isfinite(v))return std::unexpected("pickup settings must be finite");
    if(s.standoff<=0||s.margin<=0||s.wall_padding<0||s.bounds_padding<0||s.yaw_offsets.empty())return std::unexpected("invalid pickup distances or yaw candidates");
    for(double offset:s.yaw_offsets)if(!std::isfinite(offset)||std::abs(offset)>2*std::numbers::pi+1e-6)
        return std::unexpected("pickup yaw candidates must be finite and within one revolution");
    if(s.yaw_resolution<0 || (s.yaw_resolution>0&&s.yaw_resolution<.001))return std::unexpected("pickup yaw resolution must be 0 or >=0.001 rad");
    auto angles=s.yaw_offsets;
    if(s.yaw_resolution>0) {
        const auto [low,high]=std::minmax_element(s.yaw_offsets.begin(),s.yaw_offsets.end());
        if(*high-*low>2*std::numbers::pi+1e-6)return std::unexpected("pickup yaw range exceeds one revolution");
        const int n=static_cast<int>(std::ceil(std::max(std::abs(*low),std::abs(*high))/s.yaw_resolution));
        for(int i=1;i<=n;++i)for(double sign:{1.,-1.}) {
            const double offset=sign*i*s.yaw_resolution;
            if(offset>=*low&&offset<=*high)angles.push_back(offset);
        }
    }
    for(double offset:angles) {
        auto target=pose_for_tool(layout.x[req.block],layout.y,s.yaw+offset,req.tool[0],req.tool[1]);
        if(pickup_stage_clearance(layout,fixed,g,s,req,{target},true,false)<s.margin)continue;
        // All starts remain outside the row (+y), but diagonal approaches can
        // avoid sweeping the opposite raised arm's conservative lower outline
        // through a neighbouring note. Never weaken the collision constraints.
        for(double lateral:{0.,1.,-1.,2.,-2.}) {
            Pose2 pre{target.x+lateral*s.standoff,target.y+s.standoff,target.yaw};
            const std::vector<Pose2> attack{pre,target},retreat{target,pre};
            double gap=pickup_stage_clearance(layout,fixed,g,s,req,attack,true,false);
            gap=std::min(gap,pickup_stage_clearance(layout,fixed,g,s,req,retreat,false,true));
            if(gap<s.margin)continue;
            const std::vector<std::vector<Pose2>> candidates{
                {pre},
                {{req.start.x,s.transit_y,req.start.yaw},{pre.x,s.transit_y,pre.yaw},pre},
                {{req.start.x,s.transit_y,pre.yaw},{pre.x,s.transit_y,pre.yaw},pre}
            };
            for(auto transit:candidates) {
                const auto sweep=with_start(req.start,transit);
                if(!usable(sweep))continue;
                const double complete_gap=std::min(gap,pickup_stage_clearance(layout,fixed,g,s,req,sweep,false,false));
                if(complete_gap>=s.margin)return PickupPlan{std::move(transit),{target},{pre},complete_gap};
            }
        }
    }
    return std::unexpected("no pickup path clears walls, remaining boxes and nonselected contact parts; check tool calibration, box order or choose another gripper");
}
} // namespace task_commander
