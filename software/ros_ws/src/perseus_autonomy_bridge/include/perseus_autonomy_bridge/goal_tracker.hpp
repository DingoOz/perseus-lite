#pragma once

#include <cstdint>
#include <mutex>

namespace perseus_autonomy_bridge
{

    /**
     * @brief Tracks the single goal the bridge owns on an action server.
     *
     * Every send gets a token. Asynchronous callbacks (goal response, result)
     * carry the token of the request they belong to, and only the *current*
     * request may change the tracked state, so a late result for an older,
     * superseded goal can no longer clear the newer goal.
     *
     * States:
     *   Idle    - nothing to cancel.
     *   Pending - goal sent, response not received yet (no handle).
     *   Active  - goal accepted; handle held.
     *
     * A cancel (or a newer send) while Pending can't be applied yet because
     * there is no handle. It moves the tracker away from that token, so when the
     * response arrives on_goal_response() reports Cancel and the caller cancels
     * the goal right away instead of letting it run untracked.
     *
     * HandlePtr is the goal-handle pointer type (a shared_ptr in production,
     * anything nullable in tests). All methods are thread-safe.
     */
    template <typename HandlePtr>
    class GoalTracker
    {
    public:
        using Token = std::uint64_t;

        enum class State
        {
            Idle,
            Pending,
            Active
        };

        enum class ResponseAction
        {
            Track,     ///< accepted, current goal: now Active
            Cancel,    ///< accepted, but cancelled or superseded meanwhile: cancel it
            Rejected,  ///< rejected (null handle): nothing to do
        };

        struct BeginResult
        {
            Token token;
            /// Previously active goal the caller must cancel (null if none).
            HandlePtr to_cancel;
        };

        /// Call right before sending a new goal.
        BeginResult begin_request()
        {
            std::lock_guard<std::mutex> lock(_mutex);
            HandlePtr previous = (_state == State::Active) ? _handle : HandlePtr{};
            _handle = HandlePtr{};
            _state = State::Pending;
            return {++_current, previous};
        }

        /// Call from the goal-response callback of the request identified by @p token.
        ResponseAction on_goal_response(Token token, const HandlePtr& handle)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            const bool current_pending = (token == _current && _state == State::Pending);
            if (!handle)
            {
                if (current_pending)
                {
                    _state = State::Idle;
                }
                return ResponseAction::Rejected;
            }
            if (!current_pending)
            {
                return ResponseAction::Cancel;
            }
            _handle = handle;
            _state = State::Active;
            return ResponseAction::Track;
        }

        /// Call from the result callback. Returns true if it ended the current goal.
        bool on_result(Token token)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (token != _current || _state == State::Idle)
            {
                return false;
            }
            _handle = HandlePtr{};
            _state = State::Idle;
            return true;
        }

        struct CancelResult
        {
            State was;
            /// Handle the caller must cancel now (only set if it was Active).
            HandlePtr to_cancel;
        };

        /// User cancel. Active: returns the handle to cancel. Pending: the goal is
        /// cancelled as soon as its response arrives. Idle: nothing to do.
        CancelResult request_cancel()
        {
            std::lock_guard<std::mutex> lock(_mutex);
            CancelResult result{_state, _handle};
            _handle = HandlePtr{};
            _state = State::Idle;
            return result;
        }

        /// True if @p token is the current request and it is still running.
        bool is_current(Token token) const
        {
            std::lock_guard<std::mutex> lock(_mutex);
            return token == _current && _state != State::Idle;
        }

        State state() const
        {
            std::lock_guard<std::mutex> lock(_mutex);
            return _state;
        }

        /// A goal is pending or active (i.e. a cancel would do something).
        bool busy() const
        {
            return state() != State::Idle;
        }

    private:
        mutable std::mutex _mutex;
        Token _current = 0;
        State _state = State::Idle;
        HandlePtr _handle{};
    };

}  // namespace perseus_autonomy_bridge
