/*
 * Conveyor Belt Physics Plugin for Gazebo
 * Developer: Sai-04021210 (inthedarkshades00008@gmail.com)
 * Date: January 2025
 *
 * This plugin simulates conveyor belt movement and object interaction
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
    class ConveyorBeltPlugin : public ModelPlugin
    {
    private:
        // Gazebo components
        physics::ModelPtr model;
        physics::WorldPtr world;
        event::ConnectionPtr updateConnection;

        // ROS components
        ros::NodeHandle* rosNode;
        ros::Subscriber speedSubscriber;
        ros::Publisher statusPublisher;

        // Belt parameters
        double beltSpeed;
        double beltLength;
        double beltWidth;
        bool beltActive;

        // Physics parameters
        double frictionCoefficient;
        double surfaceVelocity;

    public:
        ConveyorBeltPlugin() : ModelPlugin()
        {
            this->beltSpeed = 0.1;  // m/s
            this->beltLength = 2.0; // m
            this->beltWidth = 0.5;  // m
            this->beltActive = true;
            this->frictionCoefficient = 0.8;
            this->surfaceVelocity = 0.0;
        }

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store model pointer
            this->model = _model;
            this->world = _model->GetWorld();

            // Initialize ROS
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "conveyor_belt_plugin");
            }

            this->rosNode = new ros::NodeHandle("conveyor_belt");

            // Load parameters from SDF
            if (_sdf->HasElement("belt_speed"))
                this->beltSpeed = _sdf->Get<double>("belt_speed");

            if (_sdf->HasElement("belt_length"))
                this->beltLength = _sdf->Get<double>("belt_length");

            if (_sdf->HasElement("belt_width"))
                this->beltWidth = _sdf->Get<double>("belt_width");

            if (_sdf->HasElement("friction_coefficient"))
                this->frictionCoefficient = _sdf->Get<double>("friction_coefficient");

            // Set up ROS subscribers and publishers
            this->speedSubscriber = this->rosNode->subscribe(
                "belt_speed", 1, &ConveyorBeltPlugin::OnSpeedMsg, this);

            this->statusPublisher = this->rosNode->advertise<std_msgs::Float64>(
                "belt_status", 1);

            // Connect to world update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ConveyorBeltPlugin::OnUpdate, this));

            ROS_INFO("Conveyor Belt Plugin loaded successfully");
            ROS_INFO("Belt Speed: %.2f m/s", this->beltSpeed);
            ROS_INFO("Belt Dimensions: %.2f x %.2f m", this->beltLength, this->beltWidth);
        }

        virtual void OnUpdate()
        {
            if (!this->beltActive)
                return;

            // Update surface velocity
            this->surfaceVelocity = this->beltSpeed;

            // Apply belt movement to objects on the belt
            this->ApplyBeltForces();

            // Publish belt status
            this->PublishStatus();

            // Process ROS callbacks
            ros::spinOnce();
        }

        void OnSpeedMsg(const std_msgs::Float64::ConstPtr& msg)
        {
            this->beltSpeed = msg->data;
            ROS_INFO("Belt speed updated to: %.2f m/s", this->beltSpeed);
        }

        void ApplyBeltForces()
        {
            // Get all models in the world
            physics::Model_V models = this->world->Models();

            for (auto& model : models)
            {
                // Skip the belt itself
                if (model == this->model)
                    continue;

                // Check if object is on the belt
                if (this->IsObjectOnBelt(model))
                {
                    this->ApplyConveyorForce(model);
                }
            }
        }

        bool IsObjectOnBelt(physics::ModelPtr object)
        {
            // Get object position
            ignition::math::Pose3d objectPose = object->WorldPose();
            ignition::math::Pose3d beltPose = this->model->WorldPose();

            // Check if object is within belt boundaries
            double relativeX = objectPose.Pos().X() - beltPose.Pos().X();
            double relativeY = objectPose.Pos().Y() - beltPose.Pos().Y();
            double relativeZ = objectPose.Pos().Z() - beltPose.Pos().Z();

            // Check X and Y boundaries
            if (std::abs(relativeX) > this->beltLength / 2.0)
                return false;

            if (std::abs(relativeY) > this->beltWidth / 2.0)
                return false;

            // Check if object is close enough to belt surface (within 10cm)
            if (relativeZ > 0.1 || relativeZ < -0.05)
                return false;

            return true;
        }

        void ApplyConveyorForce(physics::ModelPtr object)
        {
            // Get object's base link
            physics::LinkPtr baseLink = object->GetLink("base_link");
            if (!baseLink)
                baseLink = object->GetLinks()[0]; // Use first link if no base_link

            // Calculate force direction (along belt X-axis)
            ignition::math::Vector3d forceDirection(1.0, 0.0, 0.0);

            // Calculate force magnitude based on friction and object mass
            double objectMass = baseLink->GetInertial()->Mass();
            double forceMagnitude = this->frictionCoefficient * objectMass * 9.81 *
                                  (this->beltSpeed / 1.0); // Normalize by 1 m/s

            // Apply force
            ignition::math::Vector3d force = forceDirection * forceMagnitude;
            baseLink->AddForce(force);

            // Add some damping to prevent excessive acceleration
            ignition::math::Vector3d velocity = baseLink->WorldLinearVel();
            ignition::math::Vector3d dampingForce = -velocity * 0.1 * objectMass;
            baseLink->AddForce(dampingForce);
        }

        void PublishStatus()
        {
            std_msgs::Float64 statusMsg;
            statusMsg.data = this->beltSpeed;
            this->statusPublisher.publish(statusMsg);
        }

        virtual ~ConveyorBeltPlugin()
        {
            if (this->rosNode)
            {
                this->rosNode->shutdown();
                delete this->rosNode;
            }
        }
    };

    // Register plugin with Gazebo
    GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)
}
