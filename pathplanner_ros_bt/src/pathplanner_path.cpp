#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include <pathplanner_ros_interfaces/action/follow_path.hpp>

using namespace BT;

class PathplannerPathAction : public RosActionNode<pathplanner_ros_interfaces::action::FollowPath>
{
public:
    PathplannerPathAction(const std::string &name,
                          const NodeConfig &conf,
                          const RosNodeParams &params)
        : RosActionNode<pathplanner_ros_interfaces::action::FollowPath>(name, conf, params)
    {
    }

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosActionNode::providedBasicPorts()
    static PortsList providedPorts()
    {
        return providedBasicPorts({OutputPort<double>("path_seconds", "Output for the current path time in seconds"),
                                   OutputPort<double>("path_error", "Output for the path error")});
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the action server
    bool setGoal(RosActionNode::Goal &goal) override
    {
        // return true, if we were able to set the goal correctly.
        return true;
    }

    // Callback executed when the reply is received.
    // Based on the reply you may decide to return SUCCESS or FAILURE.
    NodeStatus onResultReceived(const WrappedResult &wr) override
    {
        auto node = node_.lock();
        std::stringstream ss;
        ss << "Finished following path";
        RCLCPP_INFO(node->get_logger(), ss.str().c_str());
        return NodeStatus::SUCCESS;
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(node_.lock()->get_logger(), "Error: %d", error);
        return NodeStatus::FAILURE;
    }

    // we also support a callback for the feedback, as in
    // the original tutorial.
    // Usually, this callback should return RUNNING, but you
    // might decide, based on the value of the feedback, to abort
    // the action, and consider the TreeNode completed.
    // In that case, return SUCCESS or FAILURE.
    // The Cancel request will be send automatically to the server.
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Seconds into movement: " << feedback->path_seconds << " error: " << feedback->error_distance;
        setOutput("path_seconds", feedback->path_seconds);
        setOutput("path_error", feedback->error_distance);
        RCLCPP_INFO(node_.lock()->get_logger(), ss.str().c_str());
        return NodeStatus::RUNNING;
    }
};

CreateRosNodePlugin(PathplannerPathAction, "PathplannerPath");