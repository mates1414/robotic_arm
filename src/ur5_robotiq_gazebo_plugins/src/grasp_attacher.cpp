// Gazebo 11 counterpart of gz-sim's DetachableJoint: on request, fixes a link of another model (the grasped object)
// to a link of this model at their current relative pose, and removes that joint again on release.
//
//   <plugin name="grasp_attacher" filename="libur5_robotiq_grasp_attacher.so">
//     <parent_link>wrist_3_link</parent_link>      link of this model
//     <child_model>apriltag_cube</child_model>     model to attach
//     <child_link>link</child_link>                link of that model
//     <attach_topic>/ur5/grasp/attach</attach_topic>   std_msgs/Empty
//     <detach_topic>/ur5/grasp/detach</detach_topic>   std_msgs/Empty
//     <state_topic>/ur5/grasp/state</state_topic>      std_msgs/String "attached" / "detached", latched
//   </plugin>

#include <memory>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

namespace ur5_robotiq_gazebo_plugins
{
class GraspAttacher : public gazebo::ModelPlugin
{
public:
  ~GraspAttacher() override
  {
    update_connection_.reset();
    if (nh_) {
      nh_->shutdown();
    }
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    if (!ros::isInitialized()) {
      ROS_FATAL_NAMED("grasp_attacher", "ROS is not initialized; start Gazebo through gazebo_ros");
      return;
    }
    const auto param = [&sdf](const std::string & key, const std::string & fallback) {
        return sdf->HasElement(key) ? sdf->Get<std::string>(key) : fallback;
      };

    model_ = model;
    const std::string parent_link = param("parent_link", "wrist_3_link");
    parent_link_ = model_->GetLink(parent_link);
    if (!parent_link_) {
      ROS_ERROR_NAMED("grasp_attacher", "model %s has no link %s", model_->GetName().c_str(), parent_link.c_str());
      return;
    }
    child_model_ = param("child_model", "apriltag_cube");
    child_link_ = param("child_link", "link");

    // Callbacks are run from the world update below, i.e. in the physics thread, which may change joints
    nh_ = std::make_unique<ros::NodeHandle>();
    nh_->setCallbackQueue(&queue_);
    attach_sub_ = nh_->subscribe(param("attach_topic", "/ur5/grasp/attach"), 1, &GraspAttacher::OnAttach, this);
    detach_sub_ = nh_->subscribe(param("detach_topic", "/ur5/grasp/detach"), 1, &GraspAttacher::OnDetach, this);
    state_pub_ = nh_->advertise<std_msgs::String>(param("state_topic", "/ur5/grasp/state"), 1, true);
    PublishState();

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &) {
          queue_.callAvailable();
        });
  }

private:
  void OnAttach(const std_msgs::EmptyConstPtr &)
  {
    if (joint_) {
      return;
    }
    const gazebo::physics::WorldPtr world = model_->GetWorld();
    const gazebo::physics::ModelPtr object = world->ModelByName(child_model_);
    const gazebo::physics::LinkPtr child = object ? object->GetLink(child_link_) : nullptr;
    if (!child) {
      ROS_WARN_NAMED("grasp_attacher", "cannot attach: no model %s with link %s", child_model_.c_str(),
        child_link_.c_str());
      return;
    }
    // A fixed joint keeps the relative pose the two links have when it is attached
    joint_ = world->Physics()->CreateJoint("fixed", model_);
    joint_->SetName("grasp_attacher_joint");
    joint_->Attach(parent_link_, child);
    joint_->Load(parent_link_, child, ignition::math::Pose3d::Zero);
    joint_->Init();
    ROS_INFO_NAMED("grasp_attacher", "attached %s to %s", child_model_.c_str(), parent_link_->GetName().c_str());
    PublishState();
  }

  void OnDetach(const std_msgs::EmptyConstPtr &)
  {
    if (!joint_) {
      return;
    }
    joint_->Detach();
    joint_.reset();
    ROS_INFO_NAMED("grasp_attacher", "detached %s", child_model_.c_str());
    PublishState();
  }

  void PublishState()
  {
    std_msgs::String state;
    state.data = joint_ ? "attached" : "detached";
    state_pub_.publish(state);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr parent_link_;
  gazebo::physics::JointPtr joint_;
  std::string child_model_;
  std::string child_link_;

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  ros::Subscriber attach_sub_;
  ros::Subscriber detach_sub_;
  ros::Publisher state_pub_;
  gazebo::event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(GraspAttacher)
}  // namespace ur5_robotiq_gazebo_plugins
