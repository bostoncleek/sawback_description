
/// \file
/// \brief Gazebo plugin for turtlbot wheel control

#include <functional>
#include <string>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/physics/World.hh"
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>


namespace gazebo
{
 class RidgebackJointPlugin : public WorldPlugin
 {
   public:
     void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
     {
       // Make sure the ROS node for Gazebo has already been initialized
       if (!ros::isInitialized())
       {
         ROS_FATAL("A ROS node for Gazebo has not been initialized."
                   "Unable to load plugin. Load the Gazebo system plugin"
                   "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
         return;
       }

       // Store the pointer to the world
       world = _world;

       // robots have not been joined
       robots_joined = false;

       // Listen to the update event. This event is broadcast every
       // simulation iteration.
       updateConnection = event::Events::ConnectWorldUpdateBegin(
           std::bind(&RidgebackJointPlugin::OnUpdate, this));


       // ridgeback link
       if(!_sdf->GetElement("ridgeback_link"))
       {
         ROS_INFO("RidgebackJointPlugin missing: ridgeback_link");
       }

       else
       {
         ridgeback_link = _sdf->GetElement("ridgeback_link")->Get<std::string>();
         ROS_INFO("%s", ridgeback_link.c_str());
       }

       // sawyer link
       if(!_sdf->GetElement("sawyer_link"))
       {
         ROS_INFO("RidgebackJointPlugin missing: sawyer_link");
       }

       else
       {
         sawyer_link = _sdf->GetElement("sawyer_link")->Get<std::string>();
         ROS_INFO("%s", sawyer_link.c_str());
       }

       // name of joint between robots
       if(!_sdf->GetElement("joint_name"))
       {
         ROS_INFO("RidgebackJointPlugin missing: joint_name");
       }

       else
       {
         joint_name = _sdf->GetElement("joint_name")->Get<std::string>();
         ROS_INFO("%s", joint_name.c_str());
       }

       ROS_INFO("RidgebackJointPlugin Plugin Loaded!");
     }


    //  // Called by the world update start event
    void OnUpdate()
    {

      // The internal stop and start request lists are not empty
      // at the beginning of the swithController() call.
      // This should not happen.


      // const auto model_count = world->ModelCount();
      // ROS_INFO("Model Count %d", model_count);
      // std::vector<physics::ModelPtr> modelV = world->Models();

      // if (!modelV.empty() && !robots_joined)
      if(!robots_joined)
      // if (!modelV.empty())
      {
        std::vector<physics::ModelPtr> modelV = world->Models();

        auto ridgeback_found = false;
        auto sawyer_found = false;

        for(const auto modelptr : modelV)
        {
          if (modelptr->GetName() == "nu_ridgeback")
          {
            ridgeback_found = true;
          }

          if (modelptr->GetName() == "sawyer")
          {
            sawyer_found = true;
          }
          // ROS_INFO("Model: %s", modelptr->GetName().c_str());
        }

        // In basic world
        // Models: 1) ground, 2) ridgeback, 3) sawyer
        if (ridgeback_found && sawyer_found)
        {
          physics::ModelPtr model1 = world->ModelByName("nu_ridgeback");
          physics::ModelPtr model2 = world->ModelByName("sawyer");

          physics::LinkPtr link1 = model1->GetLink("base_link");
          physics::LinkPtr link2 = model2->GetLink("base");

          physics::JointPtr new_joint = world->Physics()->CreateJoint("fixed", model1);
          new_joint->SetName("sawyer_base_joint");

          // physics::JointPtr new_joint = model1->CreateJoint("mount_joint", "fixed", link1, link2);

          ignition::math::Pose3d joint_pose(ignition::math::Vector3d(0, 0, 0),
                                    ignition::math::Quaterniond(0, 0, 0));

          new_joint->Load(link1, link2, joint_pose);
          new_joint->Attach(link1, link2);

          robots_joined = true;
          // ROS_INFO("ridgeback joined to sawyer");
        }
      }
    }


 private:
     // Pointer to the world
     physics::WorldPtr world;

     // Pointer to the update event connection
     event::ConnectionPtr updateConnection;

     bool robots_joined;

     std::string ridgeback_link;

     std::string sawyer_link;

     std::string joint_name;
 };

}

GZ_REGISTER_WORLD_PLUGIN(gazebo::RidgebackJointPlugin)


// end file
