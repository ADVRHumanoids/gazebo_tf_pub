#ifndef _GAZEBO_TF_PLUGIN_HH_
#define _GAZEBO_TF_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

#include <thread>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
 #include <std_srvs/Empty.h>

#include <xbot_msgs/JointState.h>

#include "gazebo_tf_pub/LinkRequested.h"

using namespace std;

static string link_requested;

namespace gazebo
{

  class GazeboTFPlugin : public ModelPlugin
  {
    public: GazeboTFPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
        std::cerr << "Invalid joint count, Centauro plugin not loaded\n";
        return;
        }

        // Store the model pointer for convenience.
        this->model = _model;
        
        link_requested="";

        this->tf_prefix="gazebo/"; 
        
        for(int i=0;i <model->GetJointCount();i++)
        {
          actual_link  = model->GetLinks()[i];
          actual_link_name=actual_link->GetName();
          cout << actual_link_name << endl;
        }
        
        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_tfix_pub",
        ros::init_options::NoSigintHandler);
        }
        

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_tfix_pub"));

        this->service_set_link      = this->rosNode->advertiseService("gazebo/pub_tf_link", pub_tf_link);
        this->service_clear_link    = this->rosNode->advertiseService("gazebo/clear_tf_link", clear_tf_link);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTFPlugin::TF_BroadCast, this,_1));
        
    }
    
     public: void TF_BroadCast(const common::UpdateInfo& _info){
       
            static tf::TransformBroadcaster br;
            tf::Transform transform_actual,trasform_parent;
            ignition::math::Pose3d pose,pose_parent;
            tf::Quaternion q;
            string past_link_requested;
            bool stop_research=false;
            
            if(link_requested!="")
            {
                past_link_requested=link_requested;
                for(int i=0;i <model->GetJointCount() || !stop_research ;i++)
                {
                    if(past_link_requested==link_requested)
                    {
                    
                        actual_link  = model->GetLinks()[i];
                        actual_link_name=actual_link->GetName();
                        actual_tf_name=tf_prefix+actual_link_name;

                        if((actual_link_name==link_requested) || (link_requested=="all"))
                        {
                            pose= actual_link->WorldPose();
                            transform_actual.setOrigin( tf::Vector3(pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z()) );
                        
                            q.setW(pose.Rot().W());
                            q.setX(pose.Rot().X());
                            q.setY(pose.Rot().Y());
                            q.setZ(pose.Rot().Z());
            
                            transform_actual.setRotation(q);
                                
                            parent_link=actual_link->GetParentJointsLinks();
                            
                            ///  ACTUAL POSE IN GAZEBO WORLD
                            if((parent_link.size()==0)||(link_requested!="all"))
                            {
                                    br.sendTransform(tf::StampedTransform(transform_actual, ros::Time::now(),"gazebo/world",actual_tf_name));
                                    stop_research=true;
                            }
                            else
                            {
                                
                                ///  GET PARENT POSE and LINK with the CHILD LINK
                                for (int i=0; i < parent_link.size(); i++) 
                                {
                                    pose_parent=parent_link[i]->WorldPose();
                                    parent_link_name=parent_link[i]->GetName();
                                    parent_tf_name=tf_prefix+parent_link_name;

                                    trasform_parent.setOrigin( tf::Vector3(pose_parent.Pos().X(),pose_parent.Pos().Y(),pose_parent.Pos().Z()) );
                                
                                    q.setW(pose_parent.Rot().W());
                                    q.setX(pose_parent.Rot().X());
                                    q.setY(pose_parent.Rot().Y());
                                    q.setZ(pose_parent.Rot().Z());

                                    trasform_parent.setRotation(q);
                                    
                                    transform_actual=trasform_parent.inverse()*transform_actual;

                                    br.sendTransform(tf::StampedTransform(transform_actual, ros::Time::now(),parent_tf_name,actual_tf_name));
                                }
                            }
                        }
                        else
                            stop_research=true;
                    }
                }
            }

    }
    public: static bool pub_tf_link(gazebo_tf_pub::LinkRequested::Request  &req, gazebo_tf_pub::LinkRequested::Response& res) {

        link_requested=req.link_name;
        return true;
    }
    public: static bool clear_tf_link(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {

    link_requested="";
    return true;
    }
    
    private: 
        
    string actual_link_name,parent_link_name,actual_tf_name,parent_tf_name;
    string tf_prefix;
        
    /// \brief Pointer to the model.
    physics::ModelPtr model;
    
    event::ConnectionPtr updateConnection;
    
    /// \brief Pointer to the Link.
    physics::LinkPtr actual_link;
    /// \brief Vector of the parent Link.
    physics::Link_V  parent_link;

    /// \brief A node use for ROS transport
    unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    ros::ServiceServer service_set_world,service_set_link,service_clear_world,service_clear_link;
    
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GazeboTFPlugin)
}
#endif

