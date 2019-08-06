#ifndef _HUMMINGBOT_PLUGIN_HH_
#define _HUMMINGBOT_PLUGIN_HH_

/*
Hummingdrone Inc.
Credits: Orkun Aşa [orkun\at/hummingdrone.co]
Project: Hummingbot
*/

#include <gazebo/gazebo.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

//ROS Headers
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
    /// Control the speed of the wheels for Hummingbot
    class HummingbotPlugin : public ModelPlugin
    {
        /// brief constructor
        public: HummingbotPlugin() {}

        /** brief The load functions is called by Gazebo when plugin is
            inserted into simulation
            \param[in] _model a pointer to the model that this plugin attached to.
            \param[in] _sdf a pointer to the plugin's SDF element.
        **/
        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            //Output to debug
            //std::cerr << "\nThe Hummingbot plugin is attached to model[" << _model->GetName() << "]\n";
            //Safety check for joint
            if (_model->GetJointCount() == 0){
                std::cerr << "Invalid joint count, Hummingbot plugin didn't load?";
                return;
            }

            //Store the model pointer for convenience.
            this->model = _model;

            //Get left wheel joint
            this->leftJoint = this->model->GetJoints()[2]; // this->model->GetJoint("left_wheel_hinge");  
            
            //joint debug
            //std::cerr << "Name: " << this->leftJoint->GetScopedName() << "\n?"; 

            //Get right wheel joint
            this->rightJoint = this->model->GetJoints()[3];// _model->GetJoint("right_wheel_hinge");


            //Setup the P-controller, with a gain of 0.1/
            this->pid = common::PID(1,0,0);

            //Apply P-controller to the joints
            this->model->GetJointController()->SetVelocityPID(
            this->leftJoint->GetScopedName(), this->pid);

            this->model->GetJointController()->SetVelocityPID(
            this->rightJoint->GetScopedName(), this->pid);

            //Set velocity as 0
            SetVelocity(this->rightJoint, 0.0);
            SetVelocity(this->leftJoint, 0.0);

            /*
            // Create a Gazebo node
            this->node = transport::NodePtr(new transport::Node());
            #if GAZEBO_MAJOR_VERSION < 8
                this->node->Init(this->model->GetWorld()->GetName());
            #else
                this->node->Init(this->model->GetWorld()->Name());
            #endif
            
            
            // Create Gazebo topic names
            std::string leftJointTopicName = "~/hummingbot/left_vel";
            std::string rightJointTopicName = "~/hummingbot/right_vel";
           
            
            // Subscribe to the topics, and register thought callbacks          
            this->leftSub = this->node->Subscribe(leftJointTopicName, &HummingbotPlugin::leftCallback, this);
            this->rightSub = this->node->Subscribe(rightJointTopicName, &HummingbotPlugin::rightCallback, this);
             */

            // Initialize the ROS if it has not already be initialized.
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(
                        argc,
                        argv,
                        "gazebo_hummingbot_client",
                        ros::init_options::NoSigintHandler
                );
            }

            //std::cerr << "Works until ROS Node";

            // Create the ROS node.
            this->rosNode.reset(new ros::NodeHandle("gazebo_hummingbot_client"));

            // Create ROS topic for left wheel and subscribe to it.
            
            ros::SubscribeOptions leftSubOptions = ros::SubscribeOptions::create<std_msgs::Float32>(
            "left_vel",
            1,
            boost::bind(&HummingbotPlugin::leftROSMsg, this, _1),
            ros::VoidPtr(), &this->rosLeftQueue);

            this->rosLeftSub = this->rosNode->subscribe(leftSubOptions);

            // Create ROS topic for left wheel and subscribe to it.
            
            ros::SubscribeOptions rightSubOptions = ros::SubscribeOptions::create<std_msgs::Float32>(
            "right_vel",
            1,
            boost::bind(&HummingbotPlugin::rightROSMsg,this, _1),
            ros::VoidPtr(), &this->rosRightQueue);
            
            this->rosRightSub = this->rosNode->subscribe(rightSubOptions);
            // Spin up the queues helper thread
            this->rosLeftQueueThread = std::thread(std::bind(&HummingbotPlugin::leftQueueThread, this));


            this->rosRightQueueThread = std::thread(std::bind(&HummingbotPlugin::rightQueueThread, this));

       }

        /// set the velocity of the Hummingbot wheel
        public: void SetVelocity(const physics::JointPtr &_joint, const double &_vel)
        {
            // Set the joint's target velocity
            this->model->GetJointController()->SetVelocityTarget(
                _joint->GetScopedName(), -1*_vel);

        }

        /// \brief handle incoming message
        /// \param[in] _msg repurpose a vector3 message.
        /// this function only use x component.
        private: void leftCallback(ConstVector3dPtr &_msg)
        {
            this->SetVelocity(this->leftJoint, _msg->x());
        }

        private: void rightCallback(ConstVector3dPtr &_msg)
        {
            this->SetVelocity(this->rightJoint, _msg->x());
        }

        /// \brief Handle and incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity of the left and right wheels
        public: void leftROSMsg(const std_msgs::Float32ConstPtr &_msg)
        {
            this->SetVelocity(this->leftJoint, _msg->data);
        }

        public: void rightROSMsg(const std_msgs::Float32ConstPtr &_msg)
        {
            this->SetVelocity(this->rightJoint, _msg->data);
        }

        /// /brief ROS helper function that processes messages
        private: void leftQueueThread()
        {
            static const double timeout = 0.01;
            while(this->rosNode->ok())
            {
                this->rosLeftQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        private: void rightQueueThread()
        {
            static const double timeout = 0.01;
            while(this->rosNode->ok())
            {
                this->rosRightQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        /// brief pointer to the model
        private: physics::ModelPtr model;

        /// brief pinter to the joints.
        private: physics::JointPtr rightJoint;
        private: physics::JointPtr leftJoint;

        /// brief a PID controller for the joint.
        private: common::PID pid;

        /// brief a node used for transport
        private: transport::NodePtr node;

        /// brief a subscribers to a named topic.
        private: transport::SubscriberPtr leftSub;
        private: transport::SubscriberPtr rightSub;

        /// brief a node which is used for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// brief a ROS subscriber for velocity of left wheel
        private: ros::Subscriber rosLeftSub;

        /// brief a ROS subscriber for velocity of right wheel
        private: ros::Subscriber rosRightSub;

        /// brief a ROS callback queues helps process messages
        private: ros::CallbackQueue rosLeftQueue;
        private: ros::CallbackQueue rosRightQueue;
        
        /// brief a thread keeps running the rosQueue
        private: std::thread rosRightQueueThread;
        private: std::thread rosLeftQueueThread;
    };
    GZ_REGISTER_MODEL_PLUGIN(HummingbotPlugin);
}
#endif
