#pragma once

#include "task_commander/geometry.hpp"
#include <array>
#include <expected>
#include <string>
#include <vector>

namespace task_commander {
using Point2 = std::array<double,2>;
using Polygon = std::vector<Point2>;
using PickupStatus = std::expected<void,std::string>;

struct PickupLayout {
    std::vector<double> x;
    double y{};
    double size{};
    auto box(std::size_t id) const -> Box;
};
auto validate_layout(const PickupLayout&,const Field&) -> PickupStatus;

struct PickupGeometry {
    Polygon raised;
    Polygon lowered;
    std::vector<Box> contacts; // Selected-pad region in base_link, not a tool offset.
};
struct PickupSettings {
    double yaw{};
    std::vector<double> yaw_offsets{0};
    double transit_y{1.0};
    double standoff{.15};
    double margin{.02};
    double wall_padding{.0025};
    double bounds_padding{.005};
    double yaw_resolution{}; // 0 disables refinement inside the configured yaw range
};
struct PickupRequest {
    Pose2 start;
    std::size_t block{};
    std::size_t gripper{};
    Point2 tool{};
    std::vector<bool> unavailable;
    // Only confirmed lifted boxes disappear from floor obstacles. An empty
    // list conservatively leaves every box on the floor.
    std::vector<bool> removed{};
};
struct PickupPlan {
    std::vector<Pose2> transit; // excludes current pose; ends at standoff
    std::vector<Pose2> approach;
    std::vector<Pose2> retreat;
    double clearance{};
};

auto polygon_box_clearance(const Polygon&,const Box&) -> double;
auto without_contact(const Polygon&,const Box&) -> std::vector<Polygon>;
auto shape_sweep_clearance(const Field&,const Polygon&,const std::vector<Pose2>&) -> double;
auto plan_pickup(const PickupLayout&,const Field&,const PickupGeometry&,const PickupSettings&,const PickupRequest&)
    -> std::expected<PickupPlan,std::string>;
// Recheck a stage from current odometry immediately before executing it.
auto pickup_stage_clearance(const PickupLayout&,const Field&,const PickupGeometry&,const PickupSettings&,
                            const PickupRequest&,const std::vector<Pose2>&,bool lowered,bool captured) -> double;
} // namespace task_commander
