/*
 ** Copyright 2021 Robotic Systems Lab - ETH Zurich:
 ** Lennart Nachtigall, Jonas Junger
 ** Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 **
 ** 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 **
 ** 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 **
 ** 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "hardware_interface/EthercatDeviceConfigurator.hpp"
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <thread>
#include <csignal>
std::unique_ptr<std::thread> worker_thread;
bool abrt = false;

EthercatDeviceConfigurator::SharedPtr configurator;

unsigned int counter = 0;

//act_torques
std::vector<float>data(2);

float velocity_command = 0.0;

//callback for /act_torques
void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr &msg){
    //ROS_INFO_STREAM("/act_torques msg received in hardware_interface: T1= " << msg->data[0] <<" and T2= "<<msg->data[1]);
    data = msg->data;
}

void velocityCallback( const std_msgs::Float32::ConstPtr &msg){
	velocity_command = msg->data;
}

void worker()
{

    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("act_torques", 1000, chatterCallback);
	ros::Subscriber sub_vel = n.subscribe("conveyor_velocity_command", 1000, velocityCallback);
    ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("act_states",1000);
    ros::Rate loop_rate(400);
    ros::spinOnce();

    bool rtSuccess = true;
    for(const auto & master: configurator->getMasters())
    {
        rtSuccess &= master->setRealtimePriority(99);
    }
    std::cout << "Setting RT Priority: " << (rtSuccess? "successful." : "not successful. Check user privileges.") << std::endl;

    bool maxonEnabledAfterStartup = false;
    /*
    ** The communication update loop.
    ** This loop is supposed to be executed at a constant rate.
    ** The EthercatMaster::update function incorporates a mechanism
    ** to create a constant rate.
     */

    while(ros::ok()){
        while(!abrt)
        {
            std_msgs::Float32MultiArray msg1;
            std::vector<float> act_states(4);

            /*
            ** Update each master.
            ** This sends the last staged commands and reads the latest readings over EtherCAT.
            ** The StandaloneEnforceRate update mode is used.
            ** This means that average update rate will be close to the target rate (if possible).
            */
            for(const auto & master: configurator->getMasters() )
            {
                master->update(ecat_master::UpdateMode::StandaloneEnforceStep);
            }

            /*
            ** Here are the commands coded which are sent to the actuators
            */
            for(const auto & slave:configurator->getSlaves()) 
            {
                if (configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Maxon)
                {

                    std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);
                    
                    //std::vector<float> act_torques = data;
					
                    
                    if(slave->getName()=="Maxon1"){

                        if (!maxonEnabledAfterStartup)
                    {
                        // Set maxons to operation enabled state, do not block the call!
                        maxon_slave_ptr->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
                    }
                    
                    if (maxon_slave_ptr->lastPdoStateChangeSuccessful() &&
                            maxon_slave_ptr->getReading().getDriveState() == maxon::DriveState::OperationEnabled)
                        {
                            
                            maxon::Command command;
                            command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode);
                            auto reading = maxon_slave_ptr->getReading();
							// Target velocity is given in rad/s
                            command.setTargetVelocity(velocity_command);
                            maxon_slave_ptr->stageCommand(command);
                            act_states[0] = reading.getActualPosition();
                            act_states[1] = reading.getActualVelocity();
                            
                        }
                    }

                    else
                    {
                        MELO_WARN_STREAM("Maxon '" << maxon_slave_ptr->getName()
                                                                            << "': " << maxon_slave_ptr->getReading().getDriveState());
                    }

                }
            }
            maxonEnabledAfterStartup = true;
	        counter++;

            msg1.data = act_states;
            pub.publish(msg1);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    
}

/*
** Handle the interrupt signal.
** This is the shutdown routine.
** Note: This logic is executed in a thread separated from the communication update!
 */
void signal_handler(int sig)
{
    /*
    ** Pre shutdown procedure.
    ** The devices execute procedures (e.g. state changes) that are necessary for a
    ** proper shutdown and that must be done with PDO communication.
    ** The communication update loop (i.e. PDO loop) continues to run!
    ** You might thus want to implement some logic that stages zero torque / velocity commands
    ** or simliar safety measures at this point using e.g. atomic variables and checking them
    ** in the communication update loop.
     */
    for(const auto & master: configurator->getMasters())
    {
        master->preShutdown();
    }

    // stop the PDO communication at the next update of the communication loop
    abrt = true;
    worker_thread->join();

    /*
    ** Completely halt the EtherCAT communication.
    ** No online communication is possible afterwards, including SDOs.
     */
    for(const auto & master: configurator->getMasters())
    {
        master->shutdown();
    }

    // Exit this executable
    std::cout << "Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}



/*
** Program entry. setup.yaml is passed as an argument in the roslaunch file.
 */
int main(int argc, char**argv)
{
    // Set the abrt_ flag upon receiving an interrupt signal (e.g. Ctrl-c)
    std::signal(SIGINT, signal_handler);
    ros::init(argc, argv, "hardware_interface");

    if(argc < 2)
    {
        std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
        return EXIT_FAILURE;
    }
    // a new EthercatDeviceConfigurator object (path to setup.yaml as constructor argument)
    configurator = std::make_shared<EthercatDeviceConfigurator>(argv[1]);

    /*
    ** Start all masters.
    ** There is exactly one bus per master which is also started.
    ** All online (i.e. SDO) configuration is done during this call.
    ** The EtherCAT interface is active afterwards, all drives are in Operational
    ** EtherCAT state and PDO communication may begin.
     */
    for(auto & master: configurator->getMasters())
    {
        if(!master->startup())
        {
            std::cerr << "Startup not successful." << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Start the PDO loop in a new thread.
    worker_thread = std::make_unique<std::thread>(&worker);

    /*
    ** Wait for a few PDO cycles to pass.
     */
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for(auto & slave: configurator->getSlaves())
    {
        std::cout << " " << slave->getName() << ": " << slave->getAddress() << std::endl;
    }

    std::cout << "Startup finished" << std::endl;

    // nothing further to do in this thread.
    pause();
}
