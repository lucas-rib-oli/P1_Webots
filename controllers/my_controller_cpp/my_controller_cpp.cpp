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
#include <stdio.h>
#include <unistd.h>

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
 * @brief Calculo del angulo entre dos puntos
 * 
 * @param current_position Posicion actual del robot
 * @param destination_position Punto de destino
 * @return Angulo entre dos puntos
 */
double calcDestHeading ( double current_position[2], double destination_position[2] )
{
	double delta_x = destination_position[0] - current_position[0];
	double delta_z = destination_position[1] - current_position[1];

	double angle = 0.0;
	// angle = std::fabs ( std::atan2 ( delta_z, delta_x ) ) * 180 / M_PI;
	angle = std::fabs ( std::atan ( delta_z / delta_x ) ) * 180 / M_PI;
	double norm_angle = 0.0;
	if ( delta_x >= 0 && delta_z >= 0 ) // I Quadrant
	{
		norm_angle =  90 - angle;
	} else if ( delta_x > 0 && delta_z < 0 ) // II Quadrant
	{
		norm_angle = 90 + angle;
	} else if ( delta_x > 0 && delta_z < 0 ) // III Quadrant
	{
		norm_angle = 270 - angle;
	} else // IV Quadrant
	{
		norm_angle = 270 + angle;
	}
	// norm_angle = std::remainder ( norm_angle, 360 );
	if ( norm_angle >= 360 )
	{
		norm_angle = norm_angle - 360;
	}
	return norm_angle;
}

/**
 * @brief Normalización del heading
 * 
 * @return double Angulo a normalizar
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
 * @brief Calculo de la velocidad tangencial del robot
 * 
 * @param angular_wheel_speed Velocidad de giro del robot
 * @return double 
 */
double calcTangentialVelocity ( double angular_wheel_speed )
{
	return angular_wheel_speed * WHEEL_RADIUS;
}

/**
 * @brief Calculo de la velocidad angular del robot
 * 
 * @param tangential_speed Velocidad tangencial del robot
 * @return double 
 */
double calcAngularVelocity ( double tangential_speed )
{
	return ( tangential_speed / AXLE_LENGTH ) * 2;
}

/**
 * @brief Calculo de la distancia euclidea entre dos puntos
 * 
 * @param current_position Actual posicion del robot
 * @param destination_position Punto objetivo
 * @return double 
 */
double calcEuclideanDistance ( double current_position[2], double destination_position[2] )
{
	return std::sqrt ( std::pow ( destination_position[0] - current_position[0], 2 ) + std::pow ( destination_position[1] - current_position[1], 2 ) );
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
	int point = 0;

	double t_heading = 0;
	double t_distance = 0;
	double delta_theta;
	double destination_heading = 0.0;
	double current_heading = 0.0;
	double comparation_angle = 0.0; // To see the direction of rotation of the robot

	double destination_position [2] = { 12.5, 14.5 };

	double ds_values [8] = {0.0};
	
	double left_speed = 0.0;
	double right_speed = 0.0;

	std::vector < std::vector<double> > coords;
	if (method == "breadth" || method == "depth" || method == "best" || method == "a_star")
	{
		// --------------------- read points for map --------------------- //
		std::string line;
		char s[100];
		// recursividad de carpetas 
		chdir("..");
		chdir("..");
		std::string path_2_points ( getcwd (s, 100) );
		path_2_points += "/webots-tools-master/"; // concatenar ruta
		// Ruta hacia los txt donde se almacenan los puntos resultantes de los algoritmos de path planning
		std::ifstream file_points ( path_2_points + method + "_points.txt" );
		
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
					coord.push_back( std::stold( line.substr(0, pos) ) + 0.5 ); // Se guardan los puntos del fichero
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
		destination_position[0] = std::stod ( std::string ( argv[2] ) );
		destination_position[1] = std::stod ( std::string ( argv[3] ) );
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

			destination_heading = calcDestHeading ( current_position, destination_position ); // Calcular el heading de destion
			current_heading = normHeading ( imu_rads[2] * 180.0 / M_PI ); // Normalizar el heading actual
			delta_theta = std::fabs (destination_heading - current_heading); // Calcular la diferencia de angulos
			delta_theta = delta_theta > 180 ? 360-delta_theta : delta_theta;
			comparation_angle = destination_heading < 180 ? destination_heading + 180.0 : destination_heading - 180.0;
			double distance_to_destination = calcEuclideanDistance ( current_position, destination_position ); // Obtener la distancia al punto objetvo

			if ( distance_to_destination < 0.1 ) //  Se llega al destino
			{
				left_speed = 0.0;
				right_speed = 0.0;
				motor_left->setVelocity(left_speed);
				motor_right->setVelocity(right_speed);
				break;
			}
			else if ( ds_values[0] > DISTANCE_LIMIT + 60 || ds_values[1] > DISTANCE_LIMIT + 60 ) // Obstaculo delante/derecha
			{
				left_speed = -MAX_SPEED * 0.20;
				right_speed = MAX_SPEED * 0.20;
			}
			else if ( ds_values[2] > DISTANCE_LIMIT ) // Obstaculo derecha
			{
				left_speed = MAX_SPEED * 0.30;
				right_speed = MAX_SPEED * 0.30;
			}
			else if ( ds_values[7] > DISTANCE_LIMIT + 60 || ds_values[6] > DISTANCE_LIMIT + 60 ) // Obstaculo delante/izquierda
			{
				left_speed = MAX_SPEED * 0.20;
				right_speed = -MAX_SPEED * 0.20;
			}
			else if ( ds_values[5] > DISTANCE_LIMIT ) // Obstaculo derecha
			{
				left_speed = MAX_SPEED * 0.30;
				right_speed = MAX_SPEED * 0.30;
			}
			else if (std::fabs (delta_theta) > 2.0 && ds_values[0] < DISTANCE_LIMIT + 20 && ds_values[1] < DISTANCE_LIMIT + 20 && ds_values[2] < DISTANCE_LIMIT + 20
			&& ds_values[3] < DISTANCE_LIMIT + 20 && ds_values[4] < DISTANCE_LIMIT + 20 && ds_values[5] < DISTANCE_LIMIT + 20 && ds_values[6] < DISTANCE_LIMIT + 20 && ds_values[7] < DISTANCE_LIMIT + 20)
			{
				if ( destination_heading < 180 )
				{
					if ( current_heading > destination_heading && current_heading < comparation_angle )
					{
						left_speed = MAX_SPEED * 0.20;
						right_speed = -MAX_SPEED * 0.20;
					}
					else 
					{
						left_speed = -MAX_SPEED * 0.20;
						right_speed = MAX_SPEED * 0.20;
					}
				} else
				{
					if ( current_heading > comparation_angle && current_heading < destination_heading )
					{
						left_speed = -MAX_SPEED * 0.20;
						right_speed = MAX_SPEED * 0.20;
					}
					else 
					{
						left_speed = MAX_SPEED * 0.20;
						right_speed = -MAX_SPEED * 0.20;
					}
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
		point = 1; // Para no coger el punto de inicio
		while (robot->step(timeStep) != -1) 
		{
			
			// Leemos sensores
			const double * pos = gps->getValues();
			const double * imu_rads = imu->getRollPitchYaw();

			double current_position [2];
			current_position [0] = pos [0]; // X
			current_position [1] = pos [2]; // Z

			if ( point == coords.size() )
			{
				break;
			}

			destination_position[0] = coords[point][1];
			destination_position[1] = coords[point][0];
			if ( calcEuclideanDistance ( current_position, destination_position ) < 0.02 )
			{
				point++;
			}
			else if ( get_heading )
			{
				double tangential_speed = calcTangentialVelocity ( ANGULAR_WHEEL_SPEED_ROTATE ); // Calculo de la velocidad tangencial
				start_time = robot->getTime ();
				get_heading = false;
				destination_heading = calcDestHeading ( current_position, destination_position ); // Calculo del heading de destino
				
				current_heading = normHeading ( imu_rads[2] * 180.0 / M_PI ); // Normalizacion
				delta_theta = std::fabs (destination_heading - current_heading);
				delta_theta = delta_theta > 180 ? 360-delta_theta : delta_theta;

				double angular_speed = calcAngularVelocity ( tangential_speed ) * 180 / M_PI; // Calculo de velocidad angular
				t_heading = std::abs(delta_theta) / angular_speed; // Tiempo en alcanzar el heading de destino

				comparation_angle = destination_heading < 180 ? destination_heading + 180.0 : destination_heading - 180.0; // Angulo suplementario
			}
			
			if ( robot->getTime () < start_time + t_heading )
			{
				
				if ( destination_heading < 180 )
				{
					if ( current_heading > destination_heading && current_heading < comparation_angle )
					{
						left_speed = ANGULAR_WHEEL_SPEED_ROTATE;
						right_speed = -ANGULAR_WHEEL_SPEED_ROTATE;
					}
					else 
					{
						left_speed = -ANGULAR_WHEEL_SPEED_ROTATE;
						right_speed = ANGULAR_WHEEL_SPEED_ROTATE;
					}
				} else
				{
					if ( current_heading > comparation_angle && current_heading < destination_heading )
					{
						left_speed = -ANGULAR_WHEEL_SPEED_ROTATE;
						right_speed = ANGULAR_WHEEL_SPEED_ROTATE;
					}
					else 
					{
						left_speed = ANGULAR_WHEEL_SPEED_ROTATE;
						right_speed = -ANGULAR_WHEEL_SPEED_ROTATE;
					}
				}
			} 
			else
			{
				if ( get_distance )
				{
					double tangential_speed = calcTangentialVelocity ( MAX_SPEED ); // Calculo de la velocidad tangencial
					start_time = robot->getTime ();
					get_distance = false;
					double distance_to_destination = calcEuclideanDistance ( current_position, destination_position );
					t_distance = distance_to_destination / tangential_speed; // Tiempo en recorrer la distancia hacia el punto objetivo
				}
				// Corregir el heading si es necesario por la inercia del robot
				current_heading = normHeading ( imu_rads[2] * 180.0 / M_PI );
				delta_theta = std::fabs (destination_heading - current_heading);
				delta_theta = delta_theta > 180 ? 360-delta_theta : delta_theta;
				comparation_angle = destination_heading < 180 ? destination_heading + 180.0 : destination_heading - 180.0;
				
				if (std::fabs (delta_theta) > 2.0 )
				{
					if ( destination_heading < 180 )
					{
						if ( current_heading > destination_heading && current_heading < comparation_angle )
						{
							left_speed = MAX_SPEED * 0.20;
							right_speed = -MAX_SPEED * 0.20;
						}
						else 
						{
							left_speed = -MAX_SPEED * 0.20;
							right_speed = MAX_SPEED * 0.20;
						}
					} else
					{
						if ( current_heading > comparation_angle && current_heading < destination_heading )
						{
							left_speed = -MAX_SPEED * 0.20;
							right_speed = MAX_SPEED * 0.20;
						}
						else 
						{
							left_speed = MAX_SPEED * 0.20;
							right_speed = -MAX_SPEED * 0.20;
						}
					}
					
				}
				else if ( robot->getTime () < start_time + t_distance )
				{
					left_speed = MAX_SPEED;
					right_speed = MAX_SPEED;
				} 
				else
				{
					left_speed = 0.0;
					right_speed = 0.0;
					get_distance = true;
					get_heading = true;
					point++;
				}
			}
			motor_left->setVelocity(left_speed);
			motor_right->setVelocity(right_speed);
		};
	}
	else 
	{
		while (robot->step(timeStep) != -1) 
		{
			
			// Leemos sensores
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
			// Activamos LEDs
			leds[i]->set ( 32 );
		}
		break;
	}

	// Enter here exit cleanup code.
	std::cout << "Bye from c++!" << std::endl;

	delete robot;
	return 0;
}