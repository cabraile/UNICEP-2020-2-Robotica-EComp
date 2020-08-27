#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "stdlib.h"
/*
 * Objetivo: enviar comandos para o robo se mover
 */


int main(int ac, char * av[]) {
	ros::init(ac, av, "aula1_node" ); // Falar pro ROS que esse vai ser um cliente
	ros::NodeHandle n;		  // O NodeHandle n ele serve pra fazer interface com o servidor

	ros::Publisher cmd_pub = n.advertise< geometry_msgs::Twist  >("cmd_vel_mux/input/teleop", 10);
	srand(time(NULL));
	ros::Rate loop_rate(10);
	while( ros::ok() ) {

		geometry_msgs::Twist cmd;
		cmd.linear.x = 0.5;
		cmd.angular.z = 2*(rand() % 5)/5.0 - 1;
			
		cmd_pub.publish(cmd);
		loop_rate.sleep();
	}

	return 0;
}
