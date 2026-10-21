// Tick-level tests for the perseus goal-queue BT nodes, loaded through the
// plugin entry point (BT_RegisterNodesFromPlugin) exactly as bt_navigator does.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace
{
    using Goals = std::vector<geometry_msgs::msg::PoseStamped>;

    geometry_msgs::msg::PoseStamped make_pose(double x)
    {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = x;
        p.pose.orientation.w = 1.0;
        return p;
    }

    Goals make_goals(std::initializer_list<double> xs)
    {
        Goals goals;
        for (double x : xs)
        {
            goals.push_back(make_pose(x));
        }
        return goals;
    }

    std::string single_node_tree(const std::string& node_xml)
    {
        return "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">" + node_xml +
               "</BehaviorTree></root>";
    }

    class GoalNodesTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            factory_.registerFromPlugin(PERSEUS_BT_NODES_LIB);
            blackboard_ = BT::Blackboard::create();
        }

        BT::NodeStatus tick_once(const std::string& node_xml)
        {
            auto tree = factory_.createTreeFromText(single_node_tree(node_xml), blackboard_);
            return tree.tickOnce();
        }

        BT::BehaviorTreeFactory factory_;
        BT::Blackboard::Ptr blackboard_;
    };
}  // namespace

TEST_F(GoalNodesTest, PluginRegistersAllNodes)
{
    const auto& manifests = factory_.manifests();
    for (const char* id : {"GetGoalFromGoals", "PopFirstGoal", "IncrementIndex", "IsGoalQueueEmpty"})
    {
        EXPECT_EQ(manifests.count(id), 1u) << id;
    }
}

TEST_F(GoalNodesTest, GetGoalFromGoalsReturnsIndexedGoal)
{
    blackboard_->set("goals", make_goals({1.0, 2.0, 3.0}));
    EXPECT_EQ(tick_once(R"(<GetGoalFromGoals goals="{goals}" index="1" goal="{goal}"/>)"),
              BT::NodeStatus::SUCCESS);
    EXPECT_DOUBLE_EQ(blackboard_->get<geometry_msgs::msg::PoseStamped>("goal").pose.position.x, 2.0);
}

TEST_F(GoalNodesTest, GetGoalFromGoalsFailsOnEmptyQueue)
{
    blackboard_->set("goals", Goals{});
    EXPECT_EQ(tick_once(R"(<GetGoalFromGoals goals="{goals}" index="0" goal="{goal}"/>)"),
              BT::NodeStatus::FAILURE);
}

TEST_F(GoalNodesTest, GetGoalFromGoalsFailsOnOutOfRangeIndex)
{
    blackboard_->set("goals", make_goals({1.0}));
    EXPECT_EQ(tick_once(R"(<GetGoalFromGoals goals="{goals}" index="1" goal="{goal}"/>)"),
              BT::NodeStatus::FAILURE);
    EXPECT_EQ(tick_once(R"(<GetGoalFromGoals goals="{goals}" index="-1" goal="{goal}"/>)"),
              BT::NodeStatus::FAILURE);
}

TEST_F(GoalNodesTest, GetGoalFromGoalsThrowsWhenGoalsMissing)
{
    EXPECT_ANY_THROW(tick_once(R"(<GetGoalFromGoals goals="{goals}" index="0" goal="{goal}"/>)"));
}

TEST_F(GoalNodesTest, PopFirstGoalRemovesOnlyTheFront)
{
    blackboard_->set("goals", make_goals({1.0, 2.0, 3.0}));
    EXPECT_EQ(tick_once(R"(<PopFirstGoal input_goals="{goals}" output_goals="{goals}"/>)"),
              BT::NodeStatus::SUCCESS);
    const auto goals = blackboard_->get<Goals>("goals");
    ASSERT_EQ(goals.size(), 2u);
    EXPECT_DOUBLE_EQ(goals[0].pose.position.x, 2.0);
    EXPECT_DOUBLE_EQ(goals[1].pose.position.x, 3.0);
}

TEST_F(GoalNodesTest, PopFirstGoalOnEmptyQueueIsNoOp)
{
    blackboard_->set("goals", Goals{});
    EXPECT_EQ(tick_once(R"(<PopFirstGoal input_goals="{goals}" output_goals="{goals}"/>)"),
              BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(blackboard_->get<Goals>("goals").empty());
}

TEST_F(GoalNodesTest, IsGoalQueueEmptyDistinguishesEmptyFromPending)
{
    blackboard_->set("goals", Goals{});
    EXPECT_EQ(tick_once(R"(<IsGoalQueueEmpty goals="{goals}"/>)"), BT::NodeStatus::SUCCESS);

    blackboard_->set("goals", make_goals({1.0}));
    EXPECT_EQ(tick_once(R"(<IsGoalQueueEmpty goals="{goals}"/>)"), BT::NodeStatus::FAILURE);
}

TEST_F(GoalNodesTest, IsGoalQueueEmptyThrowsWhenGoalsMissing)
{
    EXPECT_ANY_THROW(tick_once(R"(<IsGoalQueueEmpty goals="{goals}"/>)"));
}

TEST_F(GoalNodesTest, IncrementIndexIncrements)
{
    blackboard_->set("index", 4);
    EXPECT_EQ(tick_once(R"(<IncrementIndex index="{index}"/>)"), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(blackboard_->get<int>("index"), 5);
}

TEST_F(GoalNodesTest, GetPopLoopVisitsEveryGoalInOrderThenStops)
{
    // The bare Get/Pop loop used by the sequential waypoint trees, with the
    // navigation step stubbed out by AlwaysSuccess.
    blackboard_->set("goals", make_goals({1.0, 2.0, 3.0}));
    const std::string xml = single_node_tree(R"(
        <Sequence>
          <ForceSuccess>
            <KeepRunningUntilFailure>
              <Sequence>
                <GetGoalFromGoals goals="{goals}" index="0" goal="{current_goal}"/>
                <AlwaysSuccess/>
                <PopFirstGoal input_goals="{goals}" output_goals="{goals}"/>
              </Sequence>
            </KeepRunningUntilFailure>
          </ForceSuccess>
          <IsGoalQueueEmpty goals="{goals}"/>
        </Sequence>)");
    auto tree = factory_.createTreeFromText(xml, blackboard_);

    std::vector<double> visited;
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int i = 0; i < 100 && status == BT::NodeStatus::RUNNING; ++i)
    {
        status = tree.tickOnce();
        visited.push_back(blackboard_->get<geometry_msgs::msg::PoseStamped>("current_goal").pose.position.x);
    }
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(blackboard_->get<Goals>("goals").empty());
    // One waypoint per tick; the final tick finds the queue empty.
    EXPECT_EQ(visited, (std::vector<double>{1.0, 2.0, 3.0, 3.0}));
}
