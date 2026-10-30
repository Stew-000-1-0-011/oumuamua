#pragma once

#include "task_commander/pickup.hpp"
#include <cstdint>
#include <functional>

namespace task_commander {
enum class PickupStage {RaiseAll,Transit,Clear,Approach,Lower,Suction,GripWait,Carry,Retreat};
enum class PickupLoad {Uncertain,Held};
auto run_pickup(const std::function<PickupStatus(PickupStage)>& perform,
                const std::function<void(PickupLoad)>& record,
                const std::function<PickupStatus()>& check_abort) -> PickupStatus;

struct LiftObservation {
    std::uint64_t sequence{};
    std::int64_t stamp_ns{};
    double received_at{}; // monotonic seconds
    double height{};
    bool arrived{};
    bool moving{};
    bool feedback{};
};
auto pickup_lift_confirmed(const LiftObservation&,std::uint64_t after_sequence,std::int64_t command_stamp,
                           double target_height,double now,bool require_feedback,double tolerance=.005) -> bool;
} // namespace task_commander
