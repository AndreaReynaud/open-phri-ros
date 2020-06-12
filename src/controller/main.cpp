
#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/vrep_driver.h>
#include <physical_quantities/units/units.h>

#include "open_phri_ros/add.h"
#include "open_phri_ros/addForceGenerator.h"

//#include "open_phri_ros/remove.h"


#include <ros/ros.h>

#include <iostream>
#include <utility>
#include <vector>


class ServiceHandler{

    public :
    ServiceHandler(){
        ROS_INFO("Services handler created");
    };


    void update(phri::SafetyController& controller) {
        // Add constraints and generators
        while(not functions_.empty()) {
            functions_.back()(controller);
            //contraints_to_add_.pop_back(); // utilité si fonctions_?
        }
    }

    protected :

    std::vector<ros::ServiceServer> services_;
    std::vector<std::function<void(phri::SafetyController&)>> functions_;

};

class ConstraintHandler : ServiceHandler {
public:
    ConstraintHandler() {
        ros::NodeHandle n;
        services_.push_back(n.advertiseService("add_constraint/velocity", &ConstraintHandler::addVelocityConstraint, this));
        // services_.push_back(n.advertiseService("remove_constraint", &ConstraintHandler::removeConstraint, this));

        // TODO add more
    }

    /*void update(phri::SafetyController& controller) {
        // Add constraints and generators
        while(not functions_.empty()) {
            functions_.back()(controller);
            contraints_to_add_.pop_back(); // utilité si fonctions_?
        }
    }*/

private:
    bool addVelocityConstraint(open_phri_ros::add::Request &req, open_phri_ros::add::Response &res) {
        functions_.push_back(
            [req](phri::SafetyController& controller){
                controller.add<phri::VelocityConstraint>(req.name, scalar::Velocity(req.max_velocity));
            });
        ;
        res.stateConstraint=true;

        return true;
    }
/*
    bool removeConstraint(open_phri_ros::remove::Request &req, open_phri_ros::remove::Response &res){
        functions_.push_back([req](phri::SafetyController& controller){
            controller.removeConstraint(req.name); //generaliser pour generator ? ou utiliser remove<phri::VelocityConstraint>(req.name)
        })
    }
    */

   // std::vector<ros::ServiceServer> services_;
   // std::vector<std::function<void(phri::SafetyController&)>> functions_;
    std::vector<phri::VelocityConstraint> contraints_to_add_;
};

class ForceGeneratorHandler : ServiceHandler {
    public:
        ForceGeneratorHandler() {
            ros::NodeHandle n;
            services_.push_back(n.advertiseService("add_force_generator", &ForceGeneratorHandler::addForceGenerator, this));
            
        }

    private :
           bool addForceGenerator(open_phri_ros::addForceGenerator::Request &req, open_phri_ros::addForceGenerator::Response &res) {
        functions_.push_back(
            [req](phri::SafetyController& controller){
                controller.add<phri::ForceGenerator>(req.name, spatial::Force(req.force));
            });
        ;

        return true;
    }



};

int main(int argc, char* argv[]) {
    using namespace spatial::literals;
    using namespace units::literals;
    ros::init(argc, argv, "open_phri/controller");
    
    // ServiceHandler service_handler;
    ConstraintHandler contraint_handler;

    // TODO reconfigure the controller according to possible service calls
    // ros::NodeHandle n;
    // ros::ServiceClient controllerClient = n.serviceClient<open_phri_ros::add>("add"); //"controllerClient" mauvais nom?
    // open_phri_ros::add srv; 
    // srv.request.type_of_constraint = "velocity_constraint";
    // srv.request.max_velocity = 2; //pas sur du format - pour le moment, simple valeur

    // if (controllerClient.call(srv))
    // {
    //  ROS_INFO("Constraint added");
    // }
    // else
    // {
    // ROS_ERROR("Failed to call service add");
    // }

    constexpr double time_step = 10.0_ms;
    constexpr size_t joint_count = 7;

    auto robot = phri::Robot{"end-effector"_frame, "base"_frame, "LBR4p", joint_count};
    auto model = phri::RobotModel{robot, "robot_models/kuka_lwr4.yaml", "end-effector"};
    auto controller = phri::SafetyController{robot};
    auto driver = phri::VREPDriver(robot, time_step); 
    auto clock = phri::Clock{time_step}; 
    auto data_logger = phri::DataLogger{"/tmp", clock.getTime(), true};

    data_logger.logSafetyControllerData(&controller);
    data_logger.logRobotData(&robot);
    driver.setScene("vrep_scenes/KukaLWR4.ttt");
    driver.start();

    // Set the task space damping matrix
    robot.control().task().damping().diagonal().setConstant(100.);

	// Initialize the application. Exit on failure.
	if (driver.init()) {
		ROS_INFO("Starting main loop");
	} else {
		ROS_ERROR("Initialization failed");
		std::exit(-1);
	}

	// Run the main loop
	while (ros::ok()) {
        if(not driver.syncThenRead()) {
            ROS_ERROR("Cannot read data from the robot");
            break;
        }

        service_handler.update(controller);

        model.forwardKinematics();
        controller();
        if(not driver.send()) {
            ROS_ERROR("Cannot send data to the robot");
            break;
        }
        ros::spinOnce();
	}

	ROS_INFO("Exiting");
    driver.stop();
}

