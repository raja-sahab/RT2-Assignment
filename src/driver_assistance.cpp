/**
 * 
 * \file driver_assistance.cpp
 * 
 * \brief Node to drive the robot avoiding obstacles.
 * 
 * \author Raja Farooq Dilshad
 *
 * \version 1.0
 * 
 * \date 29/07/2022
 *
 * \details
 *
 * Subscribes to: <BR>
 *  /base_scan
 *
 * Publishes to: <BR>
 * /my_vel
 *
 * Services : <BR>
 * /command
 *
 * Description :
 *
 * This node aims to control the robot in drive assistance mode. Through the service /command, the user can 
 * increase or decrease the linear and angular velocity of the robot.
 * Reading data from /base_scan the robot can know the distance from obstacles; then it is implemented 
 * a logic that allows the robot to move without hurting them.
 *
 */

/* LIBRARIES */
#include "ros/ros.h"
#include "algorithm"
#include "cmath"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "final_assignment/Command.h"

/* FUNCTION HEADERS */
void functionCallback ( const sensor_msgs::LaserScan::ConstPtr& msg );
void scanSectors( float * ranges, float * sectors );
int logic( float * sectors );
void integral_logic( float * ranges );
double integral( float * values, int start, int end );
void drive( float straight, float turn );
bool server_response( final_assignment::Command::Request &req, final_assignment::Command::Response &res );

/* GLOBAL VARIABLES */
bool is_active = false;                       ///< Variable to turn on and off the node.
float d_br;                                   ///< Alert distance for avoiding obstacles, distance break.
float speed;								  ///< Linear speed of the robot.
float turn;                                   ///< Angular speed of the robot.
int nsect = 9; 								  ///< Number of sectors.
int front = std::floor( nsect / 2 );          ///< Index of the frontal sector.
int sector_nelem = std::floor( 720/nsect );   ///< Number of laser surveys per sector.
ros::Publisher pub;                           ///< Publisher on cmd_vel.

/* FUNCTIONS */
/**
 * 
 * \brief Function callback for the base_scan subscriber.
 * 
 * \param msg defines the message of type LaserScan published on the /base_scan topic.
 *
 * If the flag *is_active* is true, this function calls the *scanSectors* function, then the function *logic*
 * implements the choice made through sectors. 
 * If the function *logic* does not take any decision, then the *integral_logic* function is called.
 * 
 */
void functionCallback ( const sensor_msgs::LaserScan::ConstPtr& msg ) {

	if ( is_active == true ) {
		/* Preallocates variables. */
		float sectors[nsect];             // Closest distance from obstacle in each sector.
		float ranges[720];                // Copy of the laser_scan ranges.

		std::fill_n(sectors, nsect, 10);  // Initializes the values in the sectors float array to 10.

		/* Copies the ranges values in the base_scan message into an array. */
		for (int i = 0; i < 720; i++) {
			ranges[i] = msg -> ranges[i];
		}

		/* Calls the function scanSector to fill the sectors array. */
		scanSectors( ranges, sectors );

		/* Call the function logic to decide what to do. If it can not take a decision, then call the function
	   	integral_logic. */
		if ( !logic( sectors ) ) {

			integral_logic( ranges );
		}
	}
}

/**
 * 
 * \brief Function to search for the closest obstacle in each sector.
 * 
 * \param ranges defines the vector provided by the laser scanner.
 * 
 * \param sectors defines the vector to fill.
 *
 * It fills sectors with the distance of the closest obstacle in this specific sector, searching in the ranges
 * vector.
 * 
 */
void scanSectors( float * ranges, float * sectors ) {

    for ( int i = 1; i <= nsect; i++ ) {           // For all sectors.
    	for ( int j = 0; j < sector_nelem; j++) {  // For all elements in each sector.

    		/* If it finds a closest obstacle, than update the sectors array. */
    		if ( ranges[i*sector_nelem+j] < sectors[i]) {
    		    sectors[i] = ranges[i*sector_nelem+j];
    		}

    	}

    }
}

/**
 * 
 * \brief Function that rapresents the robot's logic implementation.
 * 
 * \param sectors defines the vector of distances.
 * 
 * \return 1 choice made
 *  	   0 choice not made
 *
 * This function implements the logical part of the code, choosing whether to drive the robot forward or 
 * to make it turn to avoid obstacles. 
 * It's based on the information in the *sectors* vector, so previously filtered by the *scanSector* function. 
 * According to the choice made, it calls the *drive* function to move the robot.
 * 
 */
int logic( float * sectors ) {

	if ( sectors[front] > d_br ) { // The frontal sector is obstacle-free.

		/* Searchs in the front-right and in the front-left sectors if there are obstacles, to line up with
		   the track. */
		if ( (sectors[front+1] <= 0.8 * d_br) && (sectors[front-1] >= d_br) ) {
			ROS_INFO("dist: %.2f, speed: %.2f, free road, turn right", sectors[front], speed);
            drive( speed, turn - 0.4 );

		} else if ( (sectors[front-1] <= 0.8 * d_br) && (sectors[front+1] >= d_br) ) {
			ROS_INFO("dist: %.2f, speed: %.2f, free road, turn left", sectors[front], speed);
            drive( speed, turn + 0.4 );

		} else {
			ROS_INFO("dist: %.2f, speed: %.2f, free road", sectors[front], speed);
		    drive( speed, turn );
		}

		return 1;

	} else { // There is an obstacle in the frontal sector.

		/* Searchs if there is an obstacle-free sector. */
		for ( int j = 1; j <= ( front - 1 ); j++ ) { // Looks in all sectors without the frontal one.

		    if ( (sectors[front+j] >= d_br ) && ( sectors[front+j] >= sectors[front-j] ) ){ // First looks left.
		    	ROS_INFO("dist: %.2f, speed: %.2f, obstacle , turn left", sectors[front], 0.2);
		        drive( 0.1, 0.5 );
		        return 1;

		    } else if ( ( sectors[front-j] >= d_br ) && ( sectors[front-j] >= sectors[front+j] ) ) { // Then looks right.
		        drive( 0.1, -0.5 );
		        ROS_INFO("dist: %.2f, speed: %.2f, obstacle , turn right", sectors[front], 0.2);
		        return 1;
		    }
		}
	}
	/* If there is not one obstacle-free sector, then it can not make any choice.*/
	return 0;
}

/**
 * 
 * \brief Function to decide where to go when there are obstales all around the robot.
 * 
 * \param ranges defines the vector provided by the laser scanner.
 *
 * This function implements the second logical part of the code, and it's executed only when the first one 
 * can not make any choice. It can only turn the robot, and it does it based on the information included in 
 * the *ranges* vector. It computes the integral (calling the *integral* function) of the distance on the 
 * right-side and the left-side of the robot, obtaining the left and right area. 
 * Lastly, comparing these two values decides where to turn the robot; and calls the *drive* function 
 * to move it.
 * 
 */
void integral_logic( float * ranges ) {

	double right_area = integral( ranges, 0, 360 );   // Right-side free area.
	double left_area = integral( ranges, 360, 720 );  // Left-side free area.

		if ( right_area > left_area ) {
			ROS_INFO("area: %.2f, speed: %.2f, OBSTACLE , turn right", right_area, 0.2);
			drive( 0, -1 );

		} else {
			ROS_INFO("area: %.2f, speed: %.2f, OBSTACLE , turn left", left_area, 0.2);
			drive( 0, 1 );

		}
}

/**
 * 
 * \brief Function to perform a discrete integral with the trapezium method.
 * 
 * \param values defines the vector on which it computes the discrete integral.
 * 
 * \param start defines the index of the initial value.
 * 
 * \param end defines the index of the final value.
 * 
 * \return double with the calculated area.
 * 
 * The discrete integral is computed between start and end inclusive.
 * 
 */
double integral( float * values, int start, int end ) {

	double result = 0;
	for ( int i = start; i < end; i++ ) {
		result = result + ( values[i] + values[i+1]) / 2;
	}
	return result;
}

/**
 * 
 * \brief Function to drive the robot.
 * 
 * \param straight defines the linear velocity.
 * 
 * \param turn defines the angular velocity.
 * 
 * This function fills the *geometry_msg* and publishes it on the topic /cmd_vel.
 * 
 */
void drive( float straight, float turn ) {

	geometry_msgs::Twist my_vel;
	my_vel.linear.x = straight;
	my_vel.angular.z = turn;
	pub.publish(my_vel);
}

/**
 * 
 * \brief Function callback to the /command service.
 * 
 * \param req defines the service's request.
 * 
 * \param res defines the service's response.
 * 
 * \return always true.
 * 
 * This function increases or decreases the speed of the motor depending on the message received and replies 
 * with the updated velocity. It also updates the robot's distance break: which is proportional to its speed.
 * 
 */
bool server_response( final_assignment::Command::Request &req, final_assignment::Command::Response &res ) {

	if ( req.command == 's' && speed >= 0.1 ) {
		speed = speed - 0.1;

	} else if ( req.command == 'w' ) {
		speed = speed + 0.1;

	} else if ( req.command == 'x' ) {
		speed = 0.0;

	} else if ( req.command == 'z' ) {
		turn = 0.0;

	}else if ( req.command == 'd' ) {
		turn = turn - 0.1;

	} else if ( req.command == 'a' ) {
		turn = turn + 0.1;

	} else if (req.command == '0') {
		is_active = true;

	} else if (req.command == '1') {
		is_active = false;
	}

	/* The distance break depends on the velocity, when the robot has an higer speed it increases the d_br variable. */
	d_br = 1.2 + speed/18;

	res.linear = speed;
	res.angular = turn;

	return true;
}

/* MAIN */
int main ( int argc, char ** argv ) {
  	
  	/* Init the ros node. */
	ros::init(argc, argv, "driver_assistance");
	ros::NodeHandle nh;

	/* Initialises the values of the global variables speed and d_br. */
	speed = 0;
	d_br = 1.2;

	/* Subscribes to the topic base_scan to receive informations about the distance from obstacles from the 
	   laser scanner on the robot. */
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, functionCallback);

	/* Creates a service for the keyboard_pilot_node to control the speed of the robot, to reset the position, 
	   and to exit from the simulation. */
	ros::ServiceServer service = nh.advertiseService("/command", server_response);

	/* Creates a publisher to the cmd_vel topic, to guide the robot in real-time controlling its velocity. */
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ros::spin();

	return 0;
}
