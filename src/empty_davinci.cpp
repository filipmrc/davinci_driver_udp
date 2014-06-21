#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

class empty_davinci : public hardware_interface::RobotHW
{
public:
    empty_davinci(){
        pos[0] = 0.0;
        vel[0] = 0.0;
        eff[0] = 0.0;

        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_roll("roll", &pos[0], &vel[0], &eff[0]);
        _joint_state_interface.registerHandle(state_handle_roll);

        registerInterface(&_joint_state_interface);
    };

    ~empty_davinci(){};

    void read(){};
    void write(){};

private:
    hardware_interface::JointStateInterface _joint_state_interface;

    double pos[1];
    double vel[1];
    double eff[1];
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "empty_davinci_node");

    empty_davinci ed;
    controller_manager::ControllerManager cm(&ed);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    ros::Time last;
    ros::Rate r(100);

    while(ros::ok())
    {
        ros::Time current_time = ros::Time::now();

        ed.read();
        cm.update(current_time, current_time - last);
        ed.write();

        last = current_time;

        r.sleep();
    }

    return 0;
}