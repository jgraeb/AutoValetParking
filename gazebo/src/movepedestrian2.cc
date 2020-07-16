#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class MovePedestrian2 : public ModelPlugin
  {
    // Pointer to the model
    private: physics::ModelPtr pedestrian;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Velocity
    double vel_x = 0;
    double vel_y = 0;
    double vel_rot = 0;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
     
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->pedestrian = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MovePedestrian2::OnUpdate, this));
      

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "move_pedestrian2",
            ros::init_options::NoSigintHandler);
      }
         
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("move_pedestrian2"));
      
        
      // Velocity
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
            "/pedestrian2_set_vel",
            1,
            boost::bind(&MovePedestrian2::SetVelocity, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&MovePedestrian2::QueueThread, this));
         
      ROS_WARN("Loaded MovePedestrian2 Plugin with parent...%s", this->pedestrian->GetName().c_str());
      
    }


    // Called by the world update start event
    public: void OnUpdate()
    {      
      this->pedestrian->SetLinearVel(ignition::math::Vector3d(this->vel_x, this->vel_y, 0));
      this->pedestrian->SetAngularVel(ignition::math::Vector3d(0, 0, this->vel_rot));
    }
    
    public: void SetVelocity(const std_msgs::Float32MultiArrayConstPtr& msg)
    {
      this->vel_x = msg->data[0];
      this->vel_y = msg->data[1];
      this->vel_rot = msg->data[2];
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MovePedestrian2)
}
