// Tests for the autonomy package's behaviour-tree XMLs.
//
// 1. Every XML must reference only node IDs and ports that are actually
//    registered by Nav2's built-in BT plugins (always loaded by bt_navigator)
//    or by perseus_bt_nodes.
// 2. The sequential NavigateThroughPoses trees must report SUCCESS only when
//    every waypoint was reached, and FAILURE (keeping the unreached waypoints
//    in {goals}) when one cannot be reached after all recoveries.
//
// Nav2's pure control/decorator nodes (RecoveryNode, PipelineSequence,
// RateController, ...) are the real implementations; Nav2 leaf nodes (which
// need a ROS node and action servers) are replaced by scripted mocks that keep
// the real nodes' port lists.

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/plugins_list.hpp"

namespace
{
    using Goals = std::vector<geometry_msgs::msg::PoseStamped>;
    namespace fs = std::filesystem;

    const fs::path kTreeDir{AUTONOMY_BT_DIR};
    const std::vector<std::string> kSequentialTrees{
        "navigate_through_poses_sequential.xml",
        "navigate_through_poses_sequential_v2.xml",
    };

    std::vector<std::string> split(const std::string& s, char delim)
    {
        std::vector<std::string> out;
        std::stringstream ss(s);
        std::string item;
        while (std::getline(ss, item, delim))
        {
            if (!item.empty())
            {
                out.push_back(item);
            }
        }
        return out;
    }

    // Register Nav2's built-in BT plugins (the list bt_navigator always loads)
    // plus perseus_bt_nodes, the only entry in our plugin_lib_names.
    void register_navigator_plugins(BT::BehaviorTreeFactory& factory)
    {
        for (const auto& lib : split(nav2::details::BT_BUILTIN_PLUGINS, ';'))
        {
            factory.registerFromPlugin(std::string(NAV2_BT_LIB_DIR) + "/lib" + lib + ".so");
        }
        factory.registerFromPlugin(PERSEUS_BT_NODES_LIB);
    }

    // Collect every element (tag) of the BehaviorTree definitions in a file.
    void collect_elements(const tinyxml2::XMLElement* el, std::vector<const tinyxml2::XMLElement*>& out)
    {
        for (auto* child = el->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
        {
            out.push_back(child);
            collect_elements(child, out);
        }
    }

    std::vector<fs::path> all_tree_files()
    {
        std::vector<fs::path> files;
        if (fs::is_directory(kTreeDir))
        {
            for (const auto& entry : fs::directory_iterator(kTreeDir))
            {
                if (entry.path().extension() == ".xml")
                {
                    files.push_back(entry.path());
                }
            }
        }
        return files;
    }

    geometry_msgs::msg::PoseStamped make_pose(double x)
    {
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = x;
        p.pose.orientation.w = 1.0;
        return p;
    }

    class ScriptedLeaf : public BT::SyncActionNode
    {
    public:
        using Fn = std::function<BT::NodeStatus(const BT::Blackboard::Ptr&)>;
        ScriptedLeaf(const std::string& name, const BT::NodeConfig& config, Fn fn)
            : BT::SyncActionNode(name, config),
              fn_(std::move(fn))
        {
        }
        BT::NodeStatus tick() override
        {
            return fn_(config().blackboard);
        }

    private:
        Fn fn_;
    };
}  // namespace

class BehaviorTreeXmlTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!fs::is_directory(kTreeDir))
        {
            GTEST_SKIP() << "autonomy behavior_trees dir not found: " << kTreeDir;
        }
        register_navigator_plugins(factory_);
    }

    BT::BehaviorTreeFactory factory_;
};

TEST_F(BehaviorTreeXmlTest, EveryNodeIdAndPortIsRegistered)
{
    const auto files = all_tree_files();
    ASSERT_FALSE(files.empty());
    const auto& manifests = factory_.manifests();

    for (const auto& file : files)
    {
        SCOPED_TRACE(file.string());
        tinyxml2::XMLDocument doc;
        ASSERT_EQ(doc.LoadFile(file.c_str()), tinyxml2::XML_SUCCESS);
        const auto* root = doc.RootElement();
        ASSERT_NE(root, nullptr);

        for (auto* bt = root->FirstChildElement("BehaviorTree"); bt != nullptr;
             bt = bt->NextSiblingElement("BehaviorTree"))
        {
            std::vector<const tinyxml2::XMLElement*> elements;
            collect_elements(bt, elements);
            for (const auto* el : elements)
            {
                const std::string id = el->Name();
                auto it = manifests.find(id);
                ASSERT_NE(it, manifests.end()) << "unregistered BT node ID <" << id << "> (line " << el->GetLineNum() << ")";
                for (auto* attr = el->FirstAttribute(); attr != nullptr; attr = attr->Next())
                {
                    const std::string key = attr->Name();
                    if (key == "name" || key == "ID" || key.rfind('_', 0) == 0)
                    {
                        continue;
                    }
                    EXPECT_EQ(it->second.ports.count(key), 1u)
                        << "<" << id << "> has no port [" << key << "] (line " << el->GetLineNum() << ")";
                }
            }
        }

        // BT.CPP's own validation (child counts, unknown IDs, ...).
        BT::BehaviorTreeFactory verify;
        register_navigator_plugins(verify);
        EXPECT_NO_THROW(verify.registerBehaviorTreeFromFile(file));
    }
}

// --- Behaviour of the sequential waypoint trees ---------------------------

class SequentialWaypointTreeTest : public ::testing::TestWithParam<std::string>
{
protected:
    void SetUp() override
    {
        if (!fs::is_directory(kTreeDir))
        {
            GTEST_SKIP() << "autonomy behavior_trees dir not found: " << kTreeDir;
        }
        // Real registrations, then swap every Nav2 leaf used by the tree for a
        // mock with the same ports.
        register_navigator_plugins(factory_);
        BT::BehaviorTreeFactory reference;
        register_navigator_plugins(reference);
        BT::BehaviorTreeFactory perseus_only;
        perseus_only.registerFromPlugin(PERSEUS_BT_NODES_LIB);

        tinyxml2::XMLDocument doc;
        ASSERT_EQ(doc.LoadFile((kTreeDir / GetParam()).c_str()), tinyxml2::XML_SUCCESS);
        std::vector<const tinyxml2::XMLElement*> elements;
        collect_elements(doc.RootElement(), elements);
        std::set<std::string> leaf_ids;
        for (const auto* el : elements)
        {
            const std::string id = el->Name();
            const auto& manifests = reference.manifests();
            auto it = manifests.find(id);
            if (it == manifests.end() || reference.builtinNodes().count(id) || perseus_only.manifests().count(id))
            {
                continue;
            }
            if (it->second.type == BT::NodeType::ACTION || it->second.type == BT::NodeType::CONDITION)
            {
                leaf_ids.insert(id);
            }
        }

        for (const auto& id : leaf_ids)
        {
            BT::TreeNodeManifest manifest = reference.manifests().at(id);
            factory_.unregisterBuilder(id);
            factory_.registerBuilder(
                manifest,
                [this, id](const std::string& name, const BT::NodeConfig& config)
                {
                    return std::make_unique<ScriptedLeaf>(
                        name, config,
                        [this, id](const BT::Blackboard::Ptr& bb)
                        {
                            return on_leaf(id, bb);
                        });
                });
        }
    }

    // Default: every Nav2 leaf succeeds, except FollowPath which fails for any
    // goal with x < 0 ("unreachable").
    BT::NodeStatus on_leaf(const std::string& id, const BT::Blackboard::Ptr& bb)
    {
        calls_[id]++;
        if (id == "FollowPath")
        {
            const auto goal = bb->get<geometry_msgs::msg::PoseStamped>("current_goal");
            followed_.push_back(goal.pose.position.x);
            return goal.pose.position.x < 0.0 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus run(const Goals& goals)
    {
        blackboard_->set("goals", goals);
        auto tree = factory_.createTreeFromFile(kTreeDir / GetParam(), blackboard_);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        for (int i = 0; i < 1000 && status == BT::NodeStatus::RUNNING; ++i)
        {
            status = tree.tickOnce();
        }
        return status;
    }

    std::vector<double> remaining_x()
    {
        std::vector<double> xs;
        for (const auto& g : blackboard_->get<Goals>("goals"))
        {
            xs.push_back(g.pose.position.x);
        }
        return xs;
    }

    BT::BehaviorTreeFactory factory_;
    BT::Blackboard::Ptr blackboard_ = BT::Blackboard::create();
    std::map<std::string, int> calls_;
    std::vector<double> followed_;
};

TEST_P(SequentialWaypointTreeTest, AllReachableWaypointsSucceedInOrder)
{
    EXPECT_EQ(run({make_pose(1.0), make_pose(2.0), make_pose(3.0)}), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(remaining_x().empty());
    EXPECT_EQ(followed_, (std::vector<double>{1.0, 2.0, 3.0}));
}

TEST_P(SequentialWaypointTreeTest, UnreachableWaypointFailsAndKeepsRemainingGoals)
{
    EXPECT_EQ(run({make_pose(1.0), make_pose(-2.0), make_pose(3.0)}), BT::NodeStatus::FAILURE);
    // The reached waypoint was popped; the failed one and the one after it
    // are still queued, and the robot never drove on to the third.
    EXPECT_EQ(remaining_x(), (std::vector<double>{-2.0, 3.0}));
    for (double x : followed_)
    {
        EXPECT_NE(x, 3.0);
    }
    // Recoveries were attempted before giving up.
    EXPECT_GT(std::count(followed_.begin(), followed_.end(), -2.0), 1);
}

TEST_P(SequentialWaypointTreeTest, FirstWaypointUnreachableFails)
{
    EXPECT_EQ(run({make_pose(-1.0)}), BT::NodeStatus::FAILURE);
    EXPECT_EQ(remaining_x(), (std::vector<double>{-1.0}));
}

INSTANTIATE_TEST_SUITE_P(Trees, SequentialWaypointTreeTest, ::testing::ValuesIn(kSequentialTrees),
                         [](const ::testing::TestParamInfo<std::string>& param_info)
                         {
                             std::string name = param_info.param.substr(0, param_info.param.find('.'));
                             for (auto& c : name)
                             {
                                 if (!std::isalnum(static_cast<unsigned char>(c)))
                                 {
                                     c = '_';
                                 }
                             }
                             return name;
                         });
