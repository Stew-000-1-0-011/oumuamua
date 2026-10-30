// Offline probe: numeric fixtures supplied by pickup_preview.py, no ROS/YAML dependency.
#include "task_commander/pickup.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>

using namespace task_commander;
int main() {
    Field field;std::size_t n;
    std::cin>>field.bounds.xmin>>field.bounds.ymin>>field.bounds.xmax>>field.bounds.ymax>>n;
    for(std::size_t i=0;i<n;++i){Box b;std::cin>>b.xmin>>b.ymin>>b.xmax>>b.ymax;field.obstacles.push_back(b);}
    std::cin>>n;
    for(std::size_t i=0;i<n;++i){Box b;std::cin>>b.xmin>>b.ymin>>b.xmax>>b.ymax;field.tall_obstacles.push_back(b);}
    PickupLayout layout;std::cin>>n;layout.x.resize(n);for(auto& x:layout.x)std::cin>>x;std::cin>>layout.y>>layout.size;
    PickupGeometry geometry;
    for(auto* polygon:{&geometry.raised,&geometry.lowered}) {
        std::cin>>n;for(std::size_t i=0;i<n;++i){Point2 p;std::cin>>p[0]>>p[1];polygon->push_back(p);}
    }
    std::cin>>n;for(std::size_t i=0;i<n;++i){Box b;std::cin>>b.xmin>>b.ymin>>b.xmax>>b.ymax;geometry.contacts.push_back(b);}
    PickupSettings settings;
    double resolution;
    std::cin>>settings.yaw>>settings.transit_y>>settings.standoff>>settings.margin>>settings.wall_padding>>settings.bounds_padding>>resolution>>n;
    settings.yaw_resolution=resolution;
    settings.yaw_offsets.resize(n);for(auto& a:settings.yaw_offsets)std::cin>>a;
    Pose2 start;std::cin>>start.x>>start.y>>start.yaw;
    std::cin>>n;std::vector<Point2> tools(n);for(auto& p:tools)std::cin>>p[0]>>p[1];
    if(!std::cin)return 2;
    std::cout<<std::setprecision(12);
    // Each case is independent. "isolated" models other boxes already delivered,
    // not an automatic multi-box collection sequence.
    for(int scenario:{0,1,2})for(std::size_t block=0;block<layout.x.size();++block)for(std::size_t grip=0;grip<tools.size();++grip) {
        PickupRequest req{start,block,grip,tools[grip],std::vector<bool>(layout.x.size(),false),std::vector<bool>(layout.x.size(),false)};
        for(std::size_t i=0;i<layout.x.size();++i)req.removed[i]=scenario==1||(scenario==2&&i>block);
        req.removed[block]=false;
        auto plan=plan_pickup(layout,field,geometry,settings,req);
        std::cout<<"case,"<<scenario<<','<<block<<','<<grip<<','<<bool(plan)<<','<<(plan?plan->clearance:0)<<'\n';
        if(plan) {
            const auto tool=offset_pose(plan->approach.back(),tools[grip][0],tools[grip][1]);
            if(std::hypot(tool.x-layout.x[block],tool.y-layout.y)>1e-8 || plan->clearance<settings.margin)return 3;
            std::cout<<"point,"<<scenario<<','<<block<<','<<grip<<",start,"<<start.x<<','<<start.y<<','<<start.yaw<<'\n';
            for(auto part:{std::pair{"transit",&plan->transit},std::pair{"approach",&plan->approach},std::pair{"retreat",&plan->retreat}})
                for(auto p:*part.second)std::cout<<"point,"<<scenario<<','<<block<<','<<grip<<','<<part.first<<','<<p.x<<','<<p.y<<','<<p.yaw<<'\n';
        }
    }
    if(layout.x.size()==5&&tools.size()==2) {
        PickupRequest first{start,4,0,tools[0],std::vector<bool>(5,false),std::vector<bool>(5,false)};
        auto a=plan_pickup(layout,field,geometry,settings,first);
        if(!a)return 4;
        PickupRequest second{a->retreat.back(),3,1,tools[1],std::vector<bool>(5,false),std::vector<bool>(5,false)};
        second.unavailable[4]=true;second.removed[4]=true;
        if(!plan_pickup(layout,field,geometry,settings,second))return 5;
    }
}
