// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

/**
 * @file my_controller_cpp.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-18
 * @extras: 
 * Se ha añadido los de las velocidades tangenciales
 * Utilización de la camera
 * 
 * 
 */

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/LED.hpp>

#include <iostream>
#include <fstream>
#include <limits>
#include <math.h>

// Definition of the dimensions of the robot (https://cyberbotics.com/doc/guide/epuck)
#define WHEEL_RADIUS 0.0205 // [m]
#define AXLE_LENGTH 0.052 // [m]

#define ANGULAR_WHEEL_SPEED_ROTATE 4.0 // velocidad angular de las ruedas para realizar el giro
#define MAX_SPEED 6.28 // velocidad angular de las ruedas para moverse en linea recta (max vel)

#define DISTANCE_LIMIT 80.0 // Distancia límite para los sensores

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// ================= Statement of functions =================

/**
 * @brief Calculate the angle between two points
 * 
 * @param current_position Current position of the robot
 * @param destination_position Destination position
 * @return Angle between two points
 */
double calcDestHeading ( double current_position[2], double destination_position[2] )
{
	double delta_x = destination_position[0] - current_position[0];
	double delta_z = destination_position[1] - current_position[1];
	double angle = 0.0;
	angle = std::fabs ( std::atan2 ( delta_z, delta_x ) ) * 180 / M_PI;
	double norm_angle = 0.0;
	if ( delta_x >= 0 && delta_z >= 0 ) // I Quadrant
	{
		norm_angle =  90 - angle;
	} else if ( delta_x >= 0 && delta_z <= 0 ) // II Quadrant
	{
		norm_angle = 90 + angle;
	} else if ( delta_x >= 0 && delta_z <= 0 ) // III Quadrant
	{
		norm_angle = 270 - angle;
	} else // IV Quadrant
	{
		norm_angle = 270 + angle;
	}
	norm_angle = std::remainder ( norm_angle, 360 );
	return norm_angle;
}

/**
 * @brief 
 * 
 * @return double 
 */
double normHeading ( double heading )
{
	double norm_angle = heading;
	if ( heading < 0 )
	{
		norm_angle = heading + 360;
	}
	return norm_angle;
}

/**
 * @brief 
 * 
 * @param angular_wheel_speed 
 * @return double 
 */
double calcTangentialVelocity ( double angular_wheel_speed )
{
	return angular_wheel_speed * WHEEL_RADIUS;
}

/**
 * @brief 
 * 
 * @param tangential_speed 
 * @return double 
 */
double calcAngularVelocity ( double tangential_speed )
{
	return ( tangential_speed / AXLE_LENGTH ) * 2;
}

/**
 * @brief 
 * 
 * @param current_position 
 * @param destination_position 
 * @return double 
 */
double calcEuclideanDistance ( double current_position[2], double destination_position[2] )
{
	return std::sqrt ( std::pow ( destination_position[0] - current_position[0], 2 ) + std::pow ( destination_position[1] - current_position[1], 2 ) );
}

void getPercentageOfColourImage ( Camera* camera )
{
	// get current image from forward camera
	const unsigned char* image = camera->getImage();

	int image_width = camera->getWidth();
    int image_height = camera->getHeight();
	int sumLeftGreen = 0;
	int sumRightGreen = 0;
	int sumLeftWhite = 0;
	int sumRightWhite = 0;
	double greenLeft = 0, redLeft = 0, blueLeft = 0;
	double greenRight = 0, redRight = 0, blueRight = 0;
	double percentage_greenLeft = 0.0;
	double percentage_greenRight = 0.0;
	double percentage_whiteLeft = 0.0;
	double percentage_whiteRight = 0.0;
	// count number of pixels that are white
	// (here assumed to have pixel value > 245 out of 255 for all color components)
	for (int x = 0; x < image_width / 2; x++) {
		for (int y = image_height / 2; y < image_height; y++) {
			greenLeft = camera->imageGetGreen(image, image_width, x, y);
			redLeft = camera->imageGetRed(image, image_width, x, y);
			blueLeft = camera->imageGetBlue(image, image_width, x, y);

			if ((greenLeft > 85) && (redLeft > 85) && (blueLeft > 85)) {
				sumLeftWhite += 1;
			}
		}

	}
	for (int x = image_width / 2; x < image_width; x++) {
		for (int y = image_height / 2; y < image_height; y++) {
			greenRight = camera->imageGetGreen(image, image_width, x, y);
			redRight = camera->imageGetRed(image, image_width, x, y);
			blueRight = camera->imageGetBlue(image, image_width, x, y);

			if ((greenRight > 85) && (redRight > 85) && (blueRight > 85)) {
				sumRightWhite += 1;
			}
		}

	}
	percentage_whiteLeft = (sumLeftWhite / (float)(image_width * image_height)) * 400; // Multiplicamos por 200 por que cogemos la mitad de la imgagen
	percentage_whiteRight = (sumRightWhite / (float)(image_width * image_height)) * 400;
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) 
{
	std::string method ( argv[1] ); // De los argumentos (breadth / depth / best / a_star / sensors / default)
	// create the Robot instance
	Robot *robot = new Robot();

	// get the time step of the current world.
	int timeStep = (int)robot->getBasicTimeStep();

	// You should insert a getDevice-like function in order to get the
	// instance of a device of the robot. Something like:
	Motor* motor_left = robot->getMotor("left wheel motor");
	Motor* motor_right = robot->getMotor("right wheel motor");
	Camera* camera = robot->getCamera("camera");
	// 8 proximity sensors available
	DistanceSensor *ds[8];
	for ( size_t i = 0; i < 8; i++)
	{
		std::string name_ds = "ps" + std::to_string ( i );
		ds[i] = robot->getDistanceSensor ( name_ds );
		ds[i]->enable (timeStep);
	}
	LED* leds [10];
	for ( size_t i = 0; i < 10; i++)
	{
		std::string name_led = "led" + std::to_string ( i );
		leds[i] = robot->getLED ( name_led );
	}
 
	GPS* gps = robot->getGPS("gps");
	InertialUnit* imu = robot->getInertialUnit("inertial unit");

	gps->enable(timeStep);
	imu->enable(timeStep);
	camera->enable(timeStep);

	motor_left->setVelocity(0.0);
	motor_right->setVelocity(0.0);

	motor_left->setPosition(std::numeric_limits<double>::infinity());
	motor_right->setPosition(std::numeric_limits<double>::infinity());

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	bool get_heading = true;
	bool get_distance = true;
	double start_time;
	bool correct_heading = false;
	int point = 0;

	double t_heading = 0;
	double t_distance = 0;
	double delta_theta;
	double destination_heading = 0.0;
	double current_heading = 0.0;
	double comparation_angle = 0.0; // To see the direction of rotation of the robot

	double destination_position [2] = { 12.5, 14.5 };

	double ds_values [8] = {0.0};

	bool avoid_obstacles = false;
	bool use_path_planning = true;

	double left_speed = 0.0;
	double right_speed = 0.0;

	std::vector < std::vector<double> > coords;
	if (method == "breadth" || method == "depth" || method == "best" || method == "a_star")
	{
		point = 1; // Para no coger el punto de inicio

		// --------------------- read points for map --------------------- //
		std::string path_2_points ( argv [2] ); // Ruta hacia los txt donde se almacenan los puntos resultantes de los algoritmos de path planning
		std::ifstream file_points ( path_2_points + method + "_points.txt" );
		std::cout << argv[0] << std::endl;
		std::string line;
		std::vector < std::vector<double> > coords;
		if ( file_points.is_open() )
		{
			while (std::getline (file_points, line))
			{
				int start = 0;
				int end = line.find(" ");
				line.substr (0, end - start);
				std::string space_delimiter = " ";

				size_t pos = 0;
				std::vector <double> coord;
				while ((pos = line.find(space_delimiter)) != std::string::npos) 
				{
					coord.push_back( std::stold( line.substr(0, pos) ) + 0.5 );
					line.erase(0, pos + space_delimiter.length());
				}
				coord.push_back ( std::stold( line ) + 0.5 );
				coords.push_back ( coord );
			}
			// Se invierte el vector
			std::vector<double> aux; 
			for (size_t i = 0; i < coords.size()/2; i++)
			{
				aux = coords[i]; 
				coords[i] = coords[coords.size()-1 -i];
				coords[coords.size()-1 -i] = aux;
			}
		}
		else
		{
			std::cout << "No se ha podido abrir el archivo: " << path_2_points + method + "_points.txt | se ejecutara el modo por defecto" << std::endl;
			method = "default";
		}
	}

	if ( method == "sensors" )
	{
		destination_position[0] = std::stod ( std::string ( argv[3] ) );
		destination_position[1] = std::stod ( std::string ( argv[4] ) );
		while (robot->step(timeStep) != -1)
		{
			for ( size_t i = 0; i < 8; i++)
			{
				ds_values[i] = ds[i]->getValue();
			}
			const double * pos = gps->getValues();
			const double * imu_rads = imu->getRollPitchYaw();

			double current_position [2];
			current_position [0] = pos [0]; // X
			current_position [1] = pos [2]; // Z

			destination_heading = calcDestHeading ( current_position, destination_position );
			current_heading = normHeading ( imu_rads[2] * 180.0 / M_PI );
			delta_theta = std::fabs (destination_heading - current_heading);
			delta_theta = delta_theta > 180 ? 360-delta_theta : delta_theta;
			comparation_angle = destination_heading < 180 ? destination_heading + 180.0 : destination_heading - 180.0;
			double distance_to_destination = calcEuclideanDistance ( current_position, destination_position );
			
			if ( distance_to_destination < 0.1 )
			{
				left_speed = 0.0;
				right_speed = 0.0;
				motor_left->setVelocity(left_speed);
				motor_right->setVelocity(right_speed);
				break;
			}
			else if ( ds_values[0] > DISTANCE_LIMIT + 60 || ds_values[1] > DISTANCE_LIMIT + 60 ) // Obstaculo delante/derecha
			{
				std::cout << "Obstaculo delante a la derecha" << std::endl;
				// set robot motor to rotate left
				left_speed = -MAX_SPEED * 0.30;
				right_speed = MAX_SPEED * 0.30;
			}
			else if ( ds_values[2] > DISTANCE_LIMIT ) // Obstaculo derecha
			{
				std::cout << "Obstaculo a la derecha" << std::endl;
				// set robot motor to rotate left
				left_speed = MAX_SPEED * 0.30;
				right_speed = MAX_SPEED * 0.30;
			}
			else if ( ds_values[7] > DISTANCE_LIMIT + 60 || ds_values[6] > DISTANCE_LIMIT + 60 ) // Obstaculo delante/izquierda
			{
				std::cout << "Obstaculo delante a la izquierda" << std::endl;
				// set robot motor to rotate left
				left_speed = MAX_SPEED * 0.30;
				right_speed = -MAX_SPEED * 0.30;
			}
			else if ( ds_values[5] > DISTANCE_LIMIT ) // Obstaculo derecha
			{
				std::cout << "Obstaculo a la izquierda" << std::endl;
				// set robot motor to rotate left
				left_speed = MAX_SPEED * 0.30;
				right_speed = MAX_SPEED * 0.30;
			}
			else if (std::fabs (delta_theta) > 2.0 )
			{
				std::cout << "Go to heading" << std::endl;
				if ( current_heading > comparation_angle || current_heading < destination_heading )
				{
					// set robot motor to rotate left
					left_speed = -MAX_SPEED * 0.30;
					right_speed = MAX_SPEED * 0.30;
				}
				// if thetaDot < 0, robot will rotate to right
				else 
				{
					// set robot motor to rotate right
					left_speed = MAX_SPEED * 0.30;
					right_speed = -MAX_SPEED * 0.30;
				}
			}
			else
			{
				left_speed = MAX_SPEED;
				right_speed = MAX_SPEED;
			}
			motor_left->setVelocity(left_speed);
			motor_right->setVelocity(right_speed);
		}
	}
	else if (method == "breadth" || method == "depth" || method == "best" || method == "a_star")
	{
		point = 1;; // Para no coger el punto de inicio

		// --------------------- read points for map --------------------- //
		std::string path_2_points ( argv [2] ); // Ruta hacia los txt donde se almacenan los puntos resultantes de los algoritmos de path planning
		std::ifstream file_points ( path_2_points + method + "_points.txt" );
		std::string line;
		if ( file_points.is_open() )
		{
			while (std::getline (file_points, line))
			{
				int start = 0;
				int end = line.find(" ");
				line.substr (0, end - start);
				std::string space_delimiter = " ";

				size_t pos = 0;
				std::vector <double> coord;
				while ((pos = line.find(space_delimiter)) != std::string::npos) 
				{
					coord.push_back( std::stold( line.substr(0, pos) ) + 0.5 );
					line.erase(0, pos + space_delimiter.length());
				}
				coord.push_back ( std::stold( line ) + 0.5 );
				coords.push_back ( coord );
			}
			// Se invierte el vector
			std::vector<double> aux; 
			for (size_t i = 0; i < coords.size()/2; i++)
			{
				aux = coords[i]; 
				coords[i] = coords[coords.size()-1 -i];
				coords[coords.size()-1 -i] = aux;
			}
		}
		while (robot->step(timeStep) != -1) 
		{
			// Read the sensors:
			// Enter here functions to read sensor data, like:
			for ( size_t i = 0; i < 8; i++)
			{
				double val = ds[i]->getValue();
			}
			
			const double * pos = gps->getValues();
			const double * imu_rads = imu->getRollPitchYaw();

			// Process sensor data here.
			double current_position [2];
			current_position [0] = pos [0]; // X
			current_position [1] = pos [2]; // Z

			if ( point == coords.size() )
			{
				break;
			}

			destination_position[0] = coords[point][1];
			destination_position[1] = coords[point][0];
			if ( calcEuclideanDistance ( current_position, destination_position ) < 0.05 )
			{
				point++;
			}
			else if ( get_heading )
			{
				double tangential_speed = calcTangentialVelocity ( ANGULAR_WHEEL_SPEED_ROTATE );
				start_time = robot->getTime ();
				get_heading = false;
				destination_heading = calcDestHeading ( current_position, destination_position );
				
				current_heading = normHeading ( imu_rads[2] * 180.0 / M_PI );
				delta_theta = std::fabs (destination_heading - current_heading);
				delta_theta = delta_theta > 180 ? 360-delta_theta : delta_theta;

				double angular_speed = calcAngularVelocity ( tangential_speed ) * 180 / M_PI;
				t_heading = std::abs(delta_theta) / angular_speed;

				comparation_angle = destination_heading < 180 ? destination_heading + 180.0 : destination_heading - 180.0;
			}
			
			if ( robot->getTime () < start_time + t_heading )
			{
				if ( current_heading > comparation_angle || current_heading < destination_heading )
				{
					// set robot motor to rotate left
					motor_left->setVelocity(-ANGULAR_WHEEL_SPEED_ROTATE);
					motor_right->setVelocity(ANGULAR_WHEEL_SPEED_ROTATE);
				}
				// if thetaDot < 0, robot will rotate to right
				else
				{
					// set robot motor to rotate right
					motor_left->setVelocity(ANGULAR_WHEEL_SPEED_ROTATE);
					motor_right->setVelocity(-ANGULAR_WHEEL_SPEED_ROTATE);
				}
			} 
			else
			{
				motor_left->setVelocity(0.0);
				motor_right->setVelocity(0.0);
				correct_heading = true;
				// get_distance = true;

				if ( get_distance )
				{
					double tangential_speed = calcTangentialVelocity ( MAX_SPEED );
					start_time = robot->getTime ();
					get_distance = false;
					double distance_to_destination = calcEuclideanDistance ( current_position, destination_position );
					t_distance = distance_to_destination / tangential_speed;
				}
				// Corregir el heading si es necesario por la inercia del robot
				current_heading = normHeading ( imu_rads[2] * 180.0 / M_PI );
				delta_theta = std::fabs (destination_heading - current_heading);
				delta_theta = delta_theta > 180 ? 360-delta_theta : delta_theta;
				comparation_angle = destination_heading < 180 ? destination_heading + 180.0 : destination_heading - 180.0;
				
				if (std::fabs (delta_theta) > 2.0 )
				{
					if ( current_heading > comparation_angle || current_heading < destination_heading )
					{
						// set robot motor to rotate left
						motor_left->setVelocity(-2.0); // Velocidades bajas para evitar el exceso de giro
						motor_right->setVelocity(2.0);
					}
					// if thetaDot < 0, robot will rotate to right
					else 
					{
						// set robot motor to rotate right
						motor_left->setVelocity(2.0);
						motor_right->setVelocity(-2.0);
					}
				}
				else if ( robot->getTime () < start_time + t_distance )
				{
					motor_left->setVelocity(6.0);
					motor_right->setVelocity(6.0);
				} 
				else
				{
					motor_left->setVelocity(0.0);
					motor_right->setVelocity(0.0);
					get_distance = true;
					get_heading = true;
					correct_heading = false;
					point++;
				}
			}

		};
	}
	else 
	{
		while (robot->step(timeStep) != -1) 
		{
			// Read the sensors:
			// Enter here functions to read sensor data, like:
			for ( size_t i = 0; i < 8; i++)
			{
				double val = ds[i]->getValue();
			}
			
			const double * pos = gps->getValues();
			const double * imu_rads = imu->getRollPitchYaw();

			// Process sensor data here.
			double current_position [2];
			current_position [0] = pos [0]; // X
			current_position [1] = pos [2]; // Z


			switch (point)
			{
			case 0:
				destination_position[0] = 4.41;
				destination_position[1] = 7.88;
				break;
			case 1:
				destination_position[0] = 5.73;
				destination_position[1] = 10.4;
				break;
			case 2:
				destination_position[0] = 9.66;
				destination_position[1] = 13.1;
				break;
			case 3:
				destination_position[0] = 12.5;
				destination_position[1] = 14.5;
				break;
			default:
				motor_left->setVelocity(0.0);
				motor_right->setVelocity(0.0);
				break;
			}

			if ( get_heading )
			{
				double tangential_speed = calcTangentialVelocity ( ANGULAR_WHEEL_SPEED_ROTATE );
				start_time = robot->getTime ();
				get_heading = false;
				destination_heading = calcDestHeading ( current_position, destination_position );
				
				current_heading = normHeading ( imu_rads[2] * 180.0 / M_PI );
				delta_theta = std::fabs (destination_heading - current_heading);
				delta_theta = delta_theta > 180 ? 360-delta_theta : delta_theta;

				double angular_speed = calcAngularVelocity ( tangential_speed ) * 180 / M_PI;
				t_heading = std::abs(delta_theta) / angular_speed;

				comparation_angle = destination_heading < 180 ? destination_heading + 180.0 : destination_heading - 180.0;
			}
			
			if ( robot->getTime () < start_time + t_heading )
			{
				if ( current_heading > comparation_angle || current_heading < destination_heading )
				{
					// set robot motor to rotate left
					motor_left->setVelocity(-ANGULAR_WHEEL_SPEED_ROTATE);
					motor_right->setVelocity(ANGULAR_WHEEL_SPEED_ROTATE);
				}
				// if thetaDot < 0, robot will rotate to right
				else
				{
					// set robot motor to rotate right
					motor_left->setVelocity(ANGULAR_WHEEL_SPEED_ROTATE);
					motor_right->setVelocity(-ANGULAR_WHEEL_SPEED_ROTATE);
				}
			} 
			else
			{
				motor_left->setVelocity(0.0);
				motor_right->setVelocity(0.0);
				correct_heading = true;
				// get_distance = true;

				if ( get_distance )
				{
					double tangential_speed = calcTangentialVelocity ( MAX_SPEED );
					start_time = robot->getTime ();
					get_distance = false;
					double distance_to_destination = calcEuclideanDistance ( current_position, destination_position );
					t_distance = distance_to_destination / tangential_speed;
				}
				// Corregir el heading si es necesario por la inercia del robot
				current_heading = normHeading ( imu_rads[2] * 180.0 / M_PI );
				delta_theta = std::fabs (destination_heading - current_heading);
				delta_theta = delta_theta > 180 ? 360-delta_theta : delta_theta;
				comparation_angle = destination_heading < 180 ? destination_heading + 180.0 : destination_heading - 180.0;
				
				if (std::fabs (delta_theta) > 1.0 )
				{
					if ( current_heading > comparation_angle || current_heading < destination_heading )
					{
						// set robot motor to rotate left
						motor_left->setVelocity(-2.0); // Velocidades bajas para evitar el exceso de giro
						motor_right->setVelocity(2.0);
					}
					// if thetaDot < 0, robot will rotate to right
					else 
					{
						// set robot motor to rotate right
						motor_left->setVelocity(2.0);
						motor_right->setVelocity(-2.0);
					}
				}
				else if ( robot->getTime () < start_time + t_distance )
				{
					motor_left->setVelocity(6.0);
					motor_right->setVelocity(6.0);
				} 
				else
				{
					motor_left->setVelocity(0.0);
					motor_right->setVelocity(0.0);
					get_distance = true;
					get_heading = true;
					correct_heading = false;
					point++;
				}
			}

		};
	}
	std::cout << "Se ha llegado al punto objetivo !!" << std::endl;
	while (robot->step(timeStep) != -1)
	{
		for ( size_t i = 0; i < 10; i++)
		{
			leds[i]->set ( 250 );
		}
	}

	

	// Enter here exit cleanup code.
	std::cout << "Bye from c++!" << std::endl;

	delete robot;
	return 0;
}