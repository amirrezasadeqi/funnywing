#include "wing_navigator/SetDouble.h"
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>

namespace gazebo {
    class CameraZoomPlugin : public SensorPlugin {
    public:
        CameraZoomPlugin() : SensorPlugin(), targetZoomFactor(1.0), zooming(false) {}

        virtual void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
            // Check if sensor is valid and of type camera
            this->camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
            if (!this->camera) {
                gzerr << "CameraZoomPlugin requires a CameraSensor.\n";
                return;
            }

            // Saving the original HFOV of the camera
            this->baseHFOV = this->camera->Camera()->HFOV();

            // Read parameters from SDF
            if (sdf->HasElement("zoom_speed")) {
                this->zoomSpeed = sdf->Get<double>("zoom_speed"); // degrees per second
            } else {
                this->zoomSpeed = 10.0; // Default zoom speed
            }

            if (sdf->HasElement("min_zoom_factor")) {
                this->minZoomFactor = sdf->Get<double>("min_zoom_factor");
            } else {
                this->minZoomFactor = 1.0; // Default minimum zoom
            }

            if (sdf->HasElement("max_zoom_factor")) {
                this->maxZoomFactor = sdf->Get<double>("max_zoom_factor");
            } else {
                this->maxZoomFactor = 10.0; // Default maximum zoom
            }

            // Set initial zoom factor
            if (sdf->HasElement("initial_zoom_factor")) {
                this->currentZoomFactor = sdf->Get<double>("initial_zoom_factor");
            } else {
                this->currentZoomFactor = 1.0;
            }

            // Initialize the camera's field of view
            this->camera->Camera()->SetHFOV(this->baseHFOV / this->currentZoomFactor);

            // Construct the service name dynamically
            std::string pluginName = sdf->Get<std::string>("name"); // Plugin name
            this->serviceName = "/" + this->camera->Name() + "/" + pluginName + "/set_camera_zoom";

            gzmsg << "CameraZoomPlugin loaded with parameters:\n"
                  << "  Zoom Speed: " << this->zoomSpeed << " degrees/second\n"
                  << "  Min Zoom Factor: " << this->minZoomFactor << "\n"
                  << "  Max Zoom Factor: " << this->maxZoomFactor << "\n"
                  << "  Service Name: " << this->serviceName << "\n";

            // Initialize ROS node (if it hasn't been initialized elsewhere)
            if (!ros::isInitialized()) {
                gzerr << "A ROS node for Gazebo has not been initialized, unable to load "
                         "plugin.\n";
                return;
            }

            this->rosNode.reset(new ros::NodeHandle("camera_zoom_plugin"));

            // Advertise the service
            this->rosService = this->rosNode->advertiseService( this->serviceName, &CameraZoomPlugin::SetZoom, this);

            // Start the update loop
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&CameraZoomPlugin::OnUpdate, this));
        }

    private:
        // ROS service callback to set a new zoom factor
        bool SetZoom(wing_navigator::SetDouble::Request &req,
                     wing_navigator::SetDouble::Response &res) {
            if (req.data < this->minZoomFactor || req.data > this->maxZoomFactor) {
                res.success = false;
                gzerr << "Zoom factor out of range (" << this->minZoomFactor << " to "
                      << this->maxZoomFactor << "). Requested: " << req.data << "\n";
                return true;
            }

            this->targetZoomFactor = req.data;
            this->zooming = true;

            // Reinitialize the last update time
            this->lastUpdateTime = ros::Time::now();

            gzmsg << "Starting zoom to factor: " << this->targetZoomFactor << "\n";
            res.success = true;
            return true;
        }

        void OnUpdate() {
            if (this->zooming) {
                // Calculate delta time
                ros::Time now = ros::Time::now();
                double deltaTime = (now - this->lastUpdateTime).toSec();
                this->lastUpdateTime = now;

                // Calculate the zoom step based on the elapsed time
                double zoomStep =
                        this->zoomSpeed * deltaTime; // Amount to zoom in this step

                if (std::abs(this->currentZoomFactor - this->targetZoomFactor) <=
                    zoomStep) {
                    // Reached the target zoom
                    this->currentZoomFactor = this->targetZoomFactor;
                    this->camera->Camera()->SetHFOV(this->baseHFOV *
                                                    (1.0 / this->currentZoomFactor));
                    this->zooming = false;
                    gzmsg << "Zoom completed. Final zoom factor: "
                          << this->currentZoomFactor << "\n";
                } else {
                    // Update zoom factor incrementally
                    if (this->currentZoomFactor < this->targetZoomFactor) {
                        this->currentZoomFactor += zoomStep;
                    } else {
                        this->currentZoomFactor -= zoomStep;
                    }

                    this->camera->Camera()->SetHFOV(this->baseHFOV *
                                                    (1.0 / this->currentZoomFactor));
                }
            }
        }

        sensors::CameraSensorPtr camera;
        ignition::math::Angle baseHFOV; // The intrinsic and origianl FOV of the
        // camera that zooming changes it!
        double currentZoomFactor;       // Current zoom factor
        double targetZoomFactor;        // Target zoom factor
        double zoomSpeed;               // Zoom speed in degrees per second
        double minZoomFactor;           // Minimum zoom factor
        double maxZoomFactor;           // Maximum zoom factor
        bool zooming;                   // Is zoom in progress?

        ros::Time lastUpdateTime; // Time of the last update
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::ServiceServer rosService;

        std::string serviceName;

        event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_SENSOR_PLUGIN(CameraZoomPlugin)
} // namespace gazebo
