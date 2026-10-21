#pragma once

#include <string>
#include <vector>

#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace perseus_bt_nodes
{

    /**
     * @brief Condition: SUCCESS iff the goal queue is empty.
     *
     * Used by the sequential waypoint trees to tell "every waypoint was reached
     * and popped" (queue empty -> SUCCESS) apart from "a waypoint failed after
     * all recoveries" (that waypoint is never popped, so the queue is non-empty
     * -> FAILURE). Throws if the [goals] input is missing, like the other
     * perseus goal-queue nodes, so a wrongly wired tree fails loudly.
     */
    class IsGoalQueueEmpty : public BT::ConditionNode
    {
    public:
        IsGoalQueueEmpty(const std::string& name, const BT::NodeConfiguration& config)
            : BT::ConditionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals")};
        }

        BT::NodeStatus tick() override
        {
            std::vector<geometry_msgs::msg::PoseStamped> goals;
            if (!getInput("goals", goals))
            {
                throw BT::RuntimeError("IsGoalQueueEmpty: missing input [goals]");
            }
            return goals.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
    };

}  // namespace perseus_bt_nodes
