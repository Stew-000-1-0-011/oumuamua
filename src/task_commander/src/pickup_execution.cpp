#include "task_commander/pickup_execution.hpp"
#include <cmath>

namespace task_commander {
auto run_pickup(const std::function<PickupStatus(PickupStage)>& perform,
                const std::function<void(PickupLoad)>& record,
                const std::function<PickupStatus()>& check_abort) -> PickupStatus {
    for(const auto stage:{PickupStage::RaiseAll,PickupStage::Transit,PickupStage::Clear,PickupStage::Approach,
                         PickupStage::Lower,PickupStage::Suction,PickupStage::GripWait,PickupStage::Carry,PickupStage::Retreat}) {
        if(auto ok=check_abort();!ok)return ok;
        // A request may reach the hardware even if its response is lost.
        if(stage==PickupStage::Suction)record(PickupLoad::Uncertain);
        if(auto ok=perform(stage);!ok)return ok;
        if(stage==PickupStage::Carry)record(PickupLoad::Held);
    }
    return {};
}
auto pickup_lift_confirmed(const LiftObservation& o,std::uint64_t after,std::int64_t stamp,
                           double target,double now,bool feedback,double tolerance) -> bool {
    return o.sequence>after&&o.stamp_ns>=stamp&&std::isfinite(now)&&std::isfinite(o.received_at)
        &&now>=o.received_at&&now-o.received_at<=.5&&o.arrived&&!o.moving
        &&(!feedback||o.feedback)&&std::isfinite(o.height)&&std::isfinite(target)
        &&std::isfinite(tolerance)&&tolerance>0&&std::abs(o.height-target)<=tolerance;
}
} // namespace task_commander
