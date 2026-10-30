#include "task_commander/pickup.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>

using namespace task_commander;
namespace {
int failures = 0;
void check(bool ok, const char* message) { if (!ok) { std::cerr << message << '\n'; ++failures; } }
bool near(double a, double b) { return std::abs(a-b) < 1e-8; }
Polygon square(double half) { return {{-half,-half},{half,-half},{half,half},{-half,half}}; }

void test_layout() {
    Field field{{-3,-1,3,2}, {}};
    PickupLayout layout{{1,-1,0,2,-2}, .1, .15};
    check(validate_layout(layout, field).has_value(), "five unordered box IDs accepted");
    check(near(layout.box(0).xmin,.925) && near(layout.box(1).xmin,-1.075), "coordinate order is stable ID order");
    for (auto xs : {std::vector<double>{}, std::vector<double>{0,0}, std::vector<double>{0,.1},
                    std::vector<double>{3}, std::vector<double>{std::numeric_limits<double>::quiet_NaN()}}) {
        check(!validate_layout({xs,.1,.15},field), "invalid layout rejected");
    }
    check(!validate_layout({{0},-.99,.15},field), "box crossing outside field rejected");
}

void test_polygon_distance() {
    auto poly=square(1);
    check(near(polygon_box_clearance(poly,{-.1,-.1,.1,.1}),0), "enclosed rectangle collision");
    check(near(polygon_box_clearance({{-1,-.1},{1,-.1},{1,.1},{-1,.1}}, {-.1,-1,.1,1}),0), "crossed edges collision");
    check(near(polygon_box_clearance(poly,{2,2,3,3}),std::sqrt(2.)), "edge distance exact");
    Polygon concave{{0,0},{2,0},{2,1},{1,1},{1,2},{0,2}};
    check(near(polygon_box_clearance(concave,{1.2,1.2,1.4,1.4}),.2), "concavity preserved");
    Field f{{-5,-5,5,5},{{-.02,.8,.02,.9}}};
    Polygon arm{{-1,-.01},{1,-.01},{1,.01},{-1,.01}};
    check(shape_sweep_clearance(f,arm,{{0,0,0},{.001,0,std::numbers::pi}})<0, "rotation between safe endpoints detected");
    check(shape_sweep_clearance(f,square(.01),{{0,0,0},{0,1,0}})<0, "translation between endpoints detected");
}

PickupGeometry test_geometry() {
    // The lowered forward pad is at x=.4 in base_link.
    return {square(.08),{{-.08,-.08},{.5,-.08},{.5,.08},{-.08,.08}},{{.3,-.1,.5,.1}}};
}

void test_contact_region() {
    auto g=test_geometry();
    const auto rest=without_contact(g.lowered,g.contacts[0]);
    double gap=10;
    for(const auto& p:rest) gap=std::min(gap,polygon_box_clearance(p,{.36,-.04,.44,.04}));
    check(gap>.02, "selected pad can overlap its target");
    gap=10;
    for(const auto& p:rest) gap=std::min(gap,polygon_box_clearance(p,{.2,-.04,.27,.04}));
    check(gap==0, "non-pad part of lower body remains checked");
    gap=10;
    for(const auto& p:rest) gap=std::min(gap,polygon_box_clearance(p,{-.02,-.02,.02,.02}));
    check(gap==0, "chassis is never exempted");
}

void test_planner() {
    PickupLayout layout{{0},0,.1};
    Field field{{-2,-1,2,2},{}};
    PickupSettings settings;
    settings.yaw=-std::numbers::pi/2;settings.yaw_offsets={0};settings.standoff=.2;settings.transit_y=1;
    auto geometry=test_geometry();
    PickupRequest req{{0,1,-std::numbers::pi/2},0,0,{.4,0},{false}};
    auto plan=plan_pickup(layout,field,geometry,settings,req);
    check(plan.has_value(), "clear pickup can be planned");
    if(plan) {
        const auto target=plan->approach.back();
        const auto tool=offset_pose(target,.4,0);
        check(near(tool.x,0)&&near(tool.y,0), "configured suction centre aligns exactly");
        check(near(plan->transit.back().y,target.y+.2), "standoff goes outside row in +y");
        check(near(plan->retreat.back().y,plan->transit.back().y), "retreat returns to standoff");
    }
    req.unavailable[0]=true;
    check(!plan_pickup(layout,field,geometry,settings,req), "already unavailable target rejected");
    req.unavailable[0]=false;
    req.start.x=std::numeric_limits<double>::quiet_NaN();
    check(!plan_pickup(layout,field,geometry,settings,req), "nonfinite pose rejected");
    req.start={0,1,-std::numbers::pi/2};
    field.obstacles.push_back({-1,.3,1,.5});
    check(!plan_pickup(layout,field,geometry,settings,req), "wall across pickup destination rejected");
}

void test_neighbour_and_tool_side() {
    PickupSettings settings; settings.yaw=-std::numbers::pi/2; settings.yaw_offsets={0};settings.standoff=.2;
    PickupLayout layout{{-.4,.4},0,.1}; Field field{{-1,-1,1,2},{}};
    auto geometry=test_geometry();
    geometry.lowered={{-.08,-.28},{.5,-.28},{.5,.28},{-.08,.28}};
    geometry.contacts={{.15,.1,.55,.3},{.15,-.3,.55,-.1}};
    PickupRequest left{{0,1,settings.yaw},0,0,{.4,.2},{false,false}};
    PickupRequest right=left;right.gripper=1;right.tool[1]=-.2;
    auto a=plan_pickup(layout,field,geometry,settings,left);
    auto b=plan_pickup(layout,field,geometry,settings,right);
    // Tool offsets determine different chassis poses, even for the same box ID.
    check(a.has_value()&&b.has_value(), "both configured tool sides have a valid path");
    if(a&&b)check(near(a->approach.back().x-b->approach.back().x,-.4), "left/right tool position is not conflated");
    Field blocked=field;blocked.obstacles={{-.6,.35,-.2,.45}};
    check(!plan_pickup(layout,blocked,geometry,settings,left), "obstacle at endpoint rejected");
    layout.x={-.9};left.block=0;left.unavailable={false};
    check(!plan_pickup(layout,field,geometry,settings,left), "unreachable wall-end combination rejected");
}

void test_uncertain_box_remains_on_floor() {
    PickupLayout layout{{.4,0},0,.1};Field field{{-2,-1,2,2},{}};
    auto geometry=test_geometry();PickupSettings settings;
    PickupRequest req{{0,0,0},0,0,{.4,0},{false,true},{false,false}};
    check(pickup_stage_clearance(layout,field,geometry,settings,req,{{0,0,0}},true,false)<.02,
          "unavailable but uncertain box stays a floor obstacle");
    req.removed[1]=true;
    check(pickup_stage_clearance(layout,field,geometry,settings,req,{{0,0,0}},true,false)>.02,
          "confirmed delivered box can leave floor obstacles");
}
}

int main() {
    test_layout();test_polygon_distance();test_contact_region();test_planner();test_neighbour_and_tool_side();test_uncertain_box_remains_on_floor();
    if(!failures)std::cout<<"pickup geometry/planner tests passed\n";
    return failures?1:0;
}
