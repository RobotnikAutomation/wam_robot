/* WAM + SUMMIT_XL DEMO
The program convert WAM joint position to Summit XL twist commands.  */

#include <iostream>
#include <string>
#include <unistd.h>

//LIBBARRETT INCLUDES
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/standard_main_function.h>

//ROS INCLUDES
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#define DEFAULT_SCALE_LINEAR   10.0
#define DEFAULT_SCALE_ANGULAR  10.0
#define DEFAULT_LINEAR_X_JOINT   3
#define DEFAULT_LINEAR_Y_JOINT   2
#define DEFAULT_ANGULAR_JOINT  0
#define DEFAULT_STANDARD_VEL 0.1

using namespace barrett;
using detail::waitForEnter;

//JointSpring class: apply a sping to every joint
template<size_t DOF>
class JointSpring : public systems::SingleIO<typename units::JointPositions<DOF>::type, typename units::JointTorques<DOF>::type> {
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  
public:
  explicit JointSpring(const jp_type& centerAngles, const std::string& sysName = "JointSpring") :
		systems::SingleIO<jp_type, jt_type>(sysName), jointRangeCenter(centerAngles) {
     for (unsigned int i=0;i<DOF;i++)
      {
        ROS_INFO("Center angle joint %d: %4.4f",i,jointRangeCenter[i]);
      }
}

  virtual ~JointSpring() { this->mandatoryCleanUp(); }

  void setSpringConstants(std::vector<double> &spring)
  {
    springConstants=spring;
    for (unsigned int i=0;i<DOF;i++)
      {
        ROS_INFO("Spring constant %d: %4.4f",i,springConstants[i]);
      }
  }
  
  double getJointCenter(unsigned int index)
  { return jointRangeCenter[index]; }

protected:
  std::vector<double> springConstants;
  jp_type jointRangeCenter, wamSpringJP;
  jt_type jtSpring;
  std::vector<double> theta;

  virtual void operate() {
    wamSpringJP = this->input.getValue();

    for (unsigned int i=0;i<DOF;i++)
      {
	jtSpring[i] = (jointRangeCenter[i] - wamSpringJP[i]) * springConstants[i];
      }
    this->outputValue->setData(&jtSpring);
  }

private:
  DISALLOW_COPY_AND_ASSIGN(JointSpring);
};

//wam_main Function
template<size_t DOF>
   int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam)
{
  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  
  ros::init(argc, argv, "wam_joystick");
  ros::NodeHandle nh("~");

  double a_scale, x_scale, y_scale;
  int linear_x, linear_y, angular;
  double standard_vel;
  std::string cmd_vel_topic;

  // Get parameters
  nh.param("linear_x_scaling_factor", x_scale, DEFAULT_SCALE_LINEAR);
  nh.param("linear_y_scaling_factor", y_scale, DEFAULT_SCALE_LINEAR);
  nh.param("angular_z_scaling_factor", a_scale, DEFAULT_SCALE_ANGULAR);
  nh.param("linear_x_joint", linear_x, DEFAULT_LINEAR_X_JOINT);
  nh.param("linear_y_joint", linear_y, DEFAULT_LINEAR_Y_JOINT);
  nh.param("angular_z_joint", angular, DEFAULT_ANGULAR_JOINT);
  nh.param("standard_vel", standard_vel, DEFAULT_STANDARD_VEL);
  nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/summit_xl_controller/command");

  wam.gravityCompensate();
  ROS_INFO("Press [Enter] to start the initialization");
  waitForEnter();

  /* INITIALIZATION */

  jp_type jp_cmd(0.0); //Initialize all joint position to 0.0
  jp_cmd[1]=-1.97;
  // jp_cmd[2]=0.13;
  jp_cmd[3]=1.95;

  wam.moveTo(jp_cmd);
  wam.idle();

  usleep(500000);
  
  JointSpring<DOF> js(wam.getJointPositions());

  std::vector<double> spring_constants(DOF,0.0);
  spring_constants[linear_x]=10.0;
  spring_constants[linear_y]=20.0;
  spring_constants[angular]=40.0;

  for(unsigned int i=0;i<DOF;i++)
  { 
     if (i!=(unsigned int)linear_x && i!=(unsigned int)angular && i!=(unsigned int)linear_y)
       spring_constants[i]=30.0; //High value to avoid rotations of the other joints
  }
  js.setSpringConstants(spring_constants);
  
  systems::connect(wam.jpOutput, js.input);
  wam.trackReferenceSignal(js.output);
  
  ROS_INFO("Press [Enter] to start the demo.\n");
  detail::waitForEnter();

  ros::Publisher vel_pub=nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

  /* DEMO START */
  ros::Rate r(50.0);

  //Ctrl+C to stop the demo (WAM go back to home position). Shift+Idle at the end of the program
  while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
    {    
        if(!ros::isShuttingDown()){  
	  geometry_msgs::Twist vel;
	  vel.angular.x = 0.0;  vel.angular.y = 0.0; vel.angular.z = 0.0;
	  vel.linear.x = 0.0;   vel.linear.y = 0.0; vel.linear.z = 0.0;

	  jp_type jp = wam.getJointPositions();

	  double delta_x=jp[linear_x]-js.getJointCenter(linear_x);
	  double delta_y=jp[linear_y]-js.getJointCenter(linear_y);
	  double delta_theta=jp[angular]-js.getJointCenter(angular);

	  if(fabs(delta_x)>0.05){	//When the joint spring push back the handle, this check avoids publishing no-zero speeds
	     if(fabs(delta_x)>0.1) delta_x<0 ? delta_x=-0.1 : delta_x=0.1; //Avoid too high speeds
	     vel.linear.x = standard_vel*x_scale*delta_x;
	  }

          if(fabs(delta_y)>0.05){       //When the joint spring push back the handle, this check avoids publishing no-zero speeds
             if(fabs(delta_y)>0.1) delta_y<0 ? delta_y=-0.1 : delta_y=0.1; //Avoid too high speeds
             vel.linear.y = standard_vel*y_scale*delta_y;
          }

	  if(fabs(delta_theta)>0.05){
	     if(fabs(delta_theta)>0.1) delta_theta<0 ? delta_theta=-0.1 : delta_theta=0.1; 
  	     vel.angular.z = standard_vel*a_scale*delta_theta;
	  }
	  vel_pub.publish(vel);

	  ROS_INFO("Command speed: linear_x %4.4f, linear_y %4.4f, angular %4.4f", vel.linear.x, vel.linear.y, vel.angular.z);
      	  r.sleep();
        } 
        else{
	  wam.moveHome();
	  usleep(1000000);
          ROS_INFO("End of the DEMO");
	  break;
        }  
    }

  return 0;
}
