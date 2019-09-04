#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

#include <gazebo/sensors/sensors.hh>
#include <gazebo/msgs/msgs.hh>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <gazebo/rendering/rendering.hh>

/*
Author: Orkun Aşa [orkun.at.hummingdrone.co]
Credits: Hummingdrone Inc.
*/

namespace gazebo
{

    class CameraSensorPlugin : public SensorPlugin
    {
    
        public: CameraSensorPlugin(){}
        public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
                        
            //Get Camera Sensor
            this->cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
            
            //Get ROS topic name from SDF file
            std::string topicName = "hummingdrone_camera"; //this->cameraSensor->Topic();

            //Initialize ROS
            if(!ros::isInitialized()){
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            //Create ROS Node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            ROS_INFO("Hummingdrone camera plugin is attached.");
            

            //Initialize the ROS publisher
            this->rosPub = this->rosNode->advertise<sensor_msgs::Image>(topicName, 1000);

            //Publish data every update
            this->updateConnection = this->cameraSensor->ConnectUpdated(boost::bind(&CameraSensorPlugin::OnRosMsg, this));
        
        }

        private: void OnRosMsg(){
        
            //Get imagesize
            imgSize =  this->cameraSensor->Camera()->ImageByteSize();

            //Create ROS message
            this->imgMsgROS.header.frame_id = "hummingdrone_camera";
            this->imgMsgROS.header.stamp.sec = this->cameraSensor->LastMeasurementTime().sec;
            this->imgMsgROS.header.stamp.nsec = this->cameraSensor->LastMeasurementTime().nsec;

            this->imgMsgROS.height = this->cameraSensor->ImageHeight();
            this->imgMsgROS.width = this->cameraSensor->ImageWidth();
            this->imgMsgROS.encoding = "bgr8"; //This is for cvBridge
            this->imgMsgROS.is_bigendian = 0;
            this->imgMsgROS.step = this->cameraSensor->ImageWidth() * this->cameraSensor->Camera()->ImageDepth();

          
           this->imgMsg = this->cameraSensor->Camera()->ImageData(); //2:RGB, 1:depth
              
                                   
            int tempVal = 0;
            for(int i=0; i < imgSize; i++){
               tempVal = this->imgMsg[i];
               this->frame.push_back(tempVal);
            } 

            //Debug: Image Size
            //std::cout << this->frame.size() << "\n";            

            //Copy image value 
            this->imgMsgROS.data = this->frame;

            //publish data
            this->rosPub.publish(this->imgMsgROS);

            //Clear vector
            this->frame.clear(); 
        }

        // \brief Pointer to the camera
        private: sensors::CameraSensorPtr cameraSensor;

        // \brief image size of the frame
        private: int imgSize;

        // \brief int array for frame
        private: std::vector<unsigned char> frame;

        // \brief A node used for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        // \brief A ROS publisher
        private: ros::Publisher rosPub;

        // \brief Image msgs for Gazebo
        private: const unsigned char *imgMsg; 

        // \brief Image msg for ROS
        private: sensor_msgs::Image imgMsgROS;

        // \brief A thread the keeps running the rosQueue
        private: event::ConnectionPtr updateConnection;
    };
    GZ_REGISTER_SENSOR_PLUGIN(CameraSensorPlugin);
}
