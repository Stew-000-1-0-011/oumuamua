#include "task_commander/pickup_execution.hpp"
#include <iostream>
#include <vector>

using namespace task_commander;
namespace {
int failures=0;
void check(bool ok,const char* message){if(!ok){std::cerr<<message<<'\n';++failures;}}
void failures_at_each_stage(){
    for(int fail=0;fail<9;++fail){
        std::vector<PickupStage> sent;std::vector<PickupLoad> records;
        auto result=run_pickup([&](PickupStage stage)->PickupStatus{
            sent.push_back(stage);if(static_cast<int>(stage)==fail)return std::unexpected("simulated actuator failure");return{};
        },[&](PickupLoad load){records.push_back(load);},[]{return PickupStatus{};});
        check(!result,"failed actuator must fail pickup");
        check(sent.size()==static_cast<std::size_t>(fail+1),"no subsequent actuator command after a failure");
        if(fail<5)check(records.empty(),"failure before suction leaves target available");
        else if(fail<8)check(records==std::vector<PickupLoad>{PickupLoad::Uncertain},"suction uncertainty/carry failure interlocks box and gripper");
        else check(records==std::vector<PickupLoad>{PickupLoad::Uncertain,PickupLoad::Held},"retreat failure keeps acquired box held");
    }
}
void cancel_at_each_boundary(){
    for(int cancel=0;cancel<9;++cancel){
        int calls=0;std::vector<PickupLoad> records;
        auto result=run_pickup([&](PickupStage)->PickupStatus{++calls;return{};},
            [&](PickupLoad load){records.push_back(load);},[&]()->PickupStatus{
                if(calls==cancel)return std::unexpected("canceled");
                return{};
            });
        check(!result&&calls==cancel,"cancellation prevents next stage");
        if(cancel<=5)check(records.empty(),"pre-suction cancellation does not reserve a box");
        else if(cancel<=7)check(records==std::vector<PickupLoad>{PickupLoad::Uncertain},"post-suction cancellation preserves unknown load");
        else check(records.back()==PickupLoad::Held,"post-carry cancellation preserves confirmed held load");
    }
}
void normal_sequence(){
    std::vector<PickupStage> sent;std::vector<PickupLoad> records;
    auto r=run_pickup([&](PickupStage s)->PickupStatus{sent.push_back(s);return{};},[&](PickupLoad l){records.push_back(l);},[]{return PickupStatus{};});
    check(r.has_value()&&sent.size()==9,"all nine stages complete");
    check(sent==std::vector<PickupStage>{PickupStage::RaiseAll,PickupStage::Transit,PickupStage::Clear,PickupStage::Approach,PickupStage::Lower,PickupStage::Suction,PickupStage::GripWait,PickupStage::Carry,PickupStage::Retreat},"all-upper travel precedes lower approach, acquisition precedes retreat");
    check(records==std::vector<PickupLoad>{PickupLoad::Uncertain,PickupLoad::Held},"reserve before suction and confirm after carry");
}
void fresh_lift_state(){
    LiftObservation o{11,101,10.,.18,true,false,true};
    check(pickup_lift_confirmed(o,10,100,.18,10.1,true),"fresh matching feedback accepted");
    auto bad=o;bad.sequence=10;check(!pickup_lift_confirmed(bad,10,100,.18,10.1,true),"old arrival sequence rejected");
    bad=o;bad.stamp_ns=99;check(!pickup_lift_confirmed(bad,10,100,.18,10.1,true),"queued pre-command feedback rejected");
    check(!pickup_lift_confirmed(o,10,100,.18,11.,true),"stale reception rejected");
    bad=o;bad.feedback=false;check(!pickup_lift_confirmed(bad,10,100,.18,10.1,true),"hardware requires real lift feedback");
    check(pickup_lift_confirmed(bad,10,100,.18,10.1,false),"explicit simulation can use command feedback");
    bad=o;bad.moving=true;check(!pickup_lift_confirmed(bad,10,100,.18,10.1,true),"moving state rejected");
    bad=o;bad.height=.09;check(!pickup_lift_confirmed(bad,10,100,.18,10.1,true),"wrong height rejected");
}
}
int main(){failures_at_each_stage();cancel_at_each_boundary();normal_sequence();fresh_lift_state();if(!failures)std::cout<<"pickup execution tests passed\n";return failures?1:0;}
