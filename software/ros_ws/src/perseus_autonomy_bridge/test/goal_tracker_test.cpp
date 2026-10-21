#include "perseus_autonomy_bridge/goal_tracker.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace
{
    using Handle = std::shared_ptr<int>;
    using Tracker = perseus_autonomy_bridge::GoalTracker<Handle>;
    using State = Tracker::State;
    using Action = Tracker::ResponseAction;

    Handle make_handle(int id)
    {
        return std::make_shared<int>(id);
    }
}  // namespace

TEST(GoalTrackerTest, StartsIdle)
{
    Tracker t;
    EXPECT_EQ(t.state(), State::Idle);
    EXPECT_FALSE(t.busy());
    EXPECT_EQ(t.request_cancel().was, State::Idle);
}

TEST(GoalTrackerTest, AcceptedGoalBecomesActiveAndResultClearsIt)
{
    Tracker t;
    auto begin = t.begin_request();
    EXPECT_EQ(begin.to_cancel, nullptr);
    EXPECT_EQ(t.state(), State::Pending);

    EXPECT_EQ(t.on_goal_response(begin.token, make_handle(1)), Action::Track);
    EXPECT_EQ(t.state(), State::Active);
    EXPECT_TRUE(t.is_current(begin.token));

    EXPECT_TRUE(t.on_result(begin.token));
    EXPECT_EQ(t.state(), State::Idle);
    EXPECT_FALSE(t.is_current(begin.token));
}

TEST(GoalTrackerTest, RejectedGoalReturnsToIdle)
{
    Tracker t;
    auto begin = t.begin_request();
    EXPECT_EQ(t.on_goal_response(begin.token, nullptr), Action::Rejected);
    EXPECT_EQ(t.state(), State::Idle);
}

// The reviewed bug: goal A is replaced by goal B, then A's result arrives
// after B was accepted and used to wipe B, so cancel said "No active goal"
// while the robot kept driving.
TEST(GoalTrackerTest, LateResultOfSupersededGoalDoesNotClearNewGoal)
{
    Tracker t;
    auto a = t.begin_request();
    auto handle_a = make_handle(1);
    ASSERT_EQ(t.on_goal_response(a.token, handle_a), Action::Track);

    auto b = t.begin_request();
    EXPECT_EQ(b.to_cancel, handle_a);  // caller must cancel A
    auto handle_b = make_handle(2);
    ASSERT_EQ(t.on_goal_response(b.token, handle_b), Action::Track);

    EXPECT_FALSE(t.on_result(a.token));  // A's (cancelled) result arrives late
    EXPECT_EQ(t.state(), State::Active);

    auto cancel = t.request_cancel();
    EXPECT_EQ(cancel.was, State::Active);
    EXPECT_EQ(cancel.to_cancel, handle_b);
    EXPECT_EQ(t.state(), State::Idle);
}

TEST(GoalTrackerTest, LateResultBeforeNewGoalResponseDoesNotClearPending)
{
    Tracker t;
    auto a = t.begin_request();
    ASSERT_EQ(t.on_goal_response(a.token, make_handle(1)), Action::Track);
    auto b = t.begin_request();
    EXPECT_FALSE(t.on_result(a.token));
    EXPECT_EQ(t.state(), State::Pending);
    EXPECT_EQ(t.on_goal_response(b.token, make_handle(2)), Action::Track);
}

// Cancel arriving between async_send_goal and the goal response used to be
// dropped; it must be applied as soon as the goal is accepted.
TEST(GoalTrackerTest, CancelWhilePendingIsAppliedOnAcceptance)
{
    Tracker t;
    auto begin = t.begin_request();
    auto cancel = t.request_cancel();
    EXPECT_EQ(cancel.was, State::Pending);
    EXPECT_EQ(cancel.to_cancel, nullptr);
    EXPECT_FALSE(t.busy());

    EXPECT_EQ(t.on_goal_response(begin.token, make_handle(1)), Action::Cancel);
    EXPECT_EQ(t.state(), State::Idle);
    EXPECT_FALSE(t.on_result(begin.token));
}

TEST(GoalTrackerTest, PendingGoalSupersededBeforeResponseIsCancelledOnAcceptance)
{
    Tracker t;
    auto a = t.begin_request();
    auto b = t.begin_request();
    EXPECT_EQ(b.to_cancel, nullptr);  // A had no handle yet

    EXPECT_EQ(t.on_goal_response(a.token, make_handle(1)), Action::Cancel);
    EXPECT_EQ(t.state(), State::Pending);  // still waiting for B
    EXPECT_EQ(t.on_goal_response(b.token, make_handle(2)), Action::Track);
    EXPECT_EQ(t.state(), State::Active);
}

TEST(GoalTrackerTest, StaleRejectionDoesNotTouchCurrentGoal)
{
    Tracker t;
    auto a = t.begin_request();
    auto b = t.begin_request();
    EXPECT_EQ(t.on_goal_response(a.token, nullptr), Action::Rejected);
    EXPECT_EQ(t.state(), State::Pending);
    EXPECT_EQ(t.on_goal_response(b.token, make_handle(2)), Action::Track);
}

TEST(GoalTrackerTest, ResultAfterUserCancelIsIgnored)
{
    Tracker t;
    auto a = t.begin_request();
    ASSERT_EQ(t.on_goal_response(a.token, make_handle(1)), Action::Track);
    ASSERT_EQ(t.request_cancel().was, State::Active);
    EXPECT_FALSE(t.on_result(a.token));
    EXPECT_EQ(t.request_cancel().was, State::Idle);
}
