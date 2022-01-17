/*
MIT License

Copyright (c) 2021 Ignacio Pérez Hurtado de Mendoza

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tars/AgentsMsg.h>
#include <tars/RobotGoalSrv.h>
#include <tars/vector2d.hpp>
#include <tars/angle.hpp>
#include <list>

#define K1 10
#define K2 0.1


// Posibles estados del robot
enum State {INIT, WAITING, MOVING};


// Estructura para almacenar toda la informacion sobre el robot
struct Robot
{
	State state;
	std::string id;  // Identificador del robot 
	double radius; // radio del robot (m)
	double maxVel; // velocidad maxima del robot  (m/s)
	double dt; // tiempo entre comandos de movimiento (s)
	utils::Vector2d position; // Posicion del robot en el frame "map"
	utils::Angle yaw; // Angulo del robot
	utils::Vector2d velocity; // Vector de velocidad instantanea del robot
	std::list<utils::Vector2d> goals; // Goals recibidos
	utils::Vector2d currentGoal; // Goal actual al que se esta navegando
	ros::Time time; // Instante de tiempo en el que comienza a navegar al siguiente goal
	ros::Publisher cmdVelPublisher; // metemos el cmdPublisher aqui porque lo usaremos en varias funciones
} robot;


/***************************/
/* DEFINICION DE FUNCIONES */
/***************************/

// Funcion main
int main(int argc, char** argv);

// Funcion para procesar la recepcion de un goal
void goalReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

// Funcion para procesar la recepcion de un mensaje de tracking de agentes
void agentsReceivedCallback(const tars::AgentsMsg::ConstPtr& agents);

// Funcion que controla al robot, segun la informacion actual, decide el siguiente comando de movimiento y actualiza el estado del robot
void control(ros::NodeHandle &n); 

// Implementacion sencilla del Dynamic Window Approach usando un vector de fuerza como input
void dwa(const utils::Vector2d& globalForce);

// Visualizacion en RVIZ de los goals
void publishGoals(ros::Publisher& goalsPub);

/****************/
/* FUNCION MAIN */
/****************/
int main(int argc, char** argv) {

	/*******************/
	/* INICIALIZACION  */
	/*******************/
	// La inicializacion de ROS, es completamente necesaria antes de ejecutar ninguna otra funcion de ROS
	ros::init(argc,argv,"tars_sfm_planning");

	// Definimos dos manejadores, uno publico y otro privado, para comunicarnos con el sistema ROS
	ros::NodeHandle n, pn("~");


	/*************************/
	/* LECTURA DE PARAMETROS */
	/*************************/

	// frecuencia del bucle principal
	double freq;
	pn.param<double>("freq",freq,20);
	robot.dt = 1.0 / freq;

	// Identificador del robot
	pn.param<std::string>("robot_id",robot.id,"09");

	// Radio del robot (m)
	pn.param<double>("robot_radius",robot.radius,0.2);

	// Velocidad maxima del robot (m/s)
	pn.param<double>("robot_max_vel",robot.maxVel,0.6);

	// topic del tracking de agentes
	std::string tracking_topic;
	pn.param<std::string>("tracking_topic",tracking_topic,"/tars/09/agents_tracking");

	// topic para recibir los goals
	std::string goal_topic;
	pn.param<std::string>("goal_topic",goal_topic,"/move_base_simple/goal");	

	// topic de comandos de velocidad 
	std::string cmd_vel_topic;
	pn.param<std::string>("cmd_vel_topic",cmd_vel_topic,"/tars/09/cmd_vel");


	/****************************/
	/* INICIALIZACION DEL ROBOT */
	/****************************/
	robot.state = INIT;
	ROS_INFO("State: INIT");

	/*************************/
	/* SUBSCRIPCION A TOPICS */
	/*************************/
	
	// Topic de deteccion de agentes 
	ros::Subscriber trackingSubscriber = n.subscribe<tars::AgentsMsg>(tracking_topic,1,agentsReceivedCallback);

	// Topic para recibir los goals
	ros::Subscriber goalSubscriber = n.subscribe<geometry_msgs::PoseStamped>(goal_topic,1,goalReceivedCallback);

	/*************************/
	/* PUBLICACION DE TOPICS */
	/*************************/

	// Topic de comandos de velocidad (cmd_vel)
	robot.cmdVelPublisher = pn.advertise<geometry_msgs::Twist>(cmd_vel_topic,1);

	// Topic de publicacion de visualizacion de goals para RVIZ
	ros::Publisher goalsPub = pn.advertise<visualization_msgs::MarkerArray>("/tars/visualization/"+robot.id+"/goals",1);

	/**********************/
 	/* BUCLE PRINCIPAL    */
 	/**********************/
	// La frecuencia a la que queremos que se ejecute el bucle
	ros::Rate r(freq);	

	// El Bucle principal, que funcionara idealmente a una tasa constante r
	while (n.ok()) {

		// Controlamos al robot
		control(n); 

		// Publicamos los goals
		publishGoals(goalsPub);

		// El nodo se duerme para ajustarse a la frecuencia deseada
		r.sleep();

		// Se leen y procesan los mensajes que llegan por los topics
		ros::spinOnce();
	}

	return 0;
}

/************************/
/* FUNCIONES CALLBACK   */
/************************/

// Funcion para procesar la recepcion de un goal
void goalReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	if (msg->header.frame_id != "map") {
		ROS_WARN("goalReceivedCallback: Invalid frame_id '%s'",msg->header.frame_id.c_str());
		return;
	}
	//Insertamos el goal al final de la lista, para ir recorriendolos en orden
	utils::Vector2d goal(msg->pose.position.x, msg->pose.position.y);
	robot.goals.push_back(goal);
	ROS_INFO("Goal received (%f, %f)",goal.getX(),goal.getY());
}


// Funcion para procesar la recepcion de un mensaje de tracking de agentes
void agentsReceivedCallback(const tars::AgentsMsg::ConstPtr& agents) {
	for (unsigned i=0;i<agents->size;i++) {
		if (agents->agents[i].id == robot.id) {
			robot.position.set(agents->agents[i].position.x, agents->agents[i].position.y);
			robot.yaw.setRadian(agents->agents[i].yaw);
			robot.velocity.set(agents->agents[i].velocity.x, agents->agents[i].velocity.y);
			if (robot.state == INIT) {
				robot.state = WAITING;
				ROS_INFO("State: WAITING");
			} else if (robot.state == MOVING) {
				utils::Vector2d force(agents->agents[i].forces.globalForce.x,agents->agents[i].forces.globalForce.y);
				dwa(force);
			}
		}
	}
}


/**********************/
/* CONTROL DEL ROBOT  */
/**********************/

void control(ros::NodeHandle &n) {

	// Si el robot esta en estado INIT (aun no ha recibido informacion de los sensores), no hacemos nada
	if (robot.state == INIT) {
		return;
	}
	
	// eliminamos los goals alcanzados
	while (!robot.goals.empty() && (robot.position - robot.goals.front()).norm() < 2.0 * robot.radius) {
		ROS_INFO("Goal reached (%f, %f)",robot.goals.front().getX(),robot.goals.front().getY());
		robot.goals.pop_front();
	}

	// Si no quedan goals, hemos terminado
	if (robot.goals.empty()) {
		// Si el robot se estaba moviendo, lo paramos y pasamos a waiting
		if (robot.state == MOVING) {
			geometry_msgs::Twist cmdVel;
			cmdVel.linear.x = 0;
			cmdVel.angular.z = 0;
			robot.cmdVelPublisher.publish(cmdVel);
			robot.state = WAITING;
			ROS_INFO("State: WAITING");
		}
		// En cualquier caso, terminamos la funcion de control
		return;
	}

	if (robot.state != MOVING) {
		robot.state = MOVING;
		ROS_INFO("State: MOVING");
	}

	// Quedan goals, vamos a intentar alcanzar el primero
	// Si el goal actual del robot es distinto del primer goal de la lista
	if (robot.goals.front() != robot.currentGoal) {
		// informamos al simulador TARS sobre el goal del robot, para que actualice su modelo de fuerzas
		ros::ServiceClient client = n.serviceClient<tars::RobotGoalSrv>("/tars/robot_goal");
		tars::RobotGoalSrv srv;
		srv.request.id = robot.id;
		srv.request.gx = robot.goals.front().getX();
		srv.request.gy = robot.goals.front().getY();
		if (client.call(srv)) {
			robot.currentGoal = robot.goals.front();
			robot.time = ros::Time::now();
			ROS_INFO("Navigating to local goal (%f, %f)", robot.currentGoal.getX(),robot.currentGoal.getY());
		}
	}



}


// Algoritmo Dynamic Window Approach que valora una serie de trayectorias circulares y ejecuta la mas apropiada
// de acuerdo a unos criterios. En esta implementacion, el criterio es sencillo, elige la trayectoria que mejor
// aproxime la velocidad instantanea deseada de acuerdo al vector de fuerza global
void dwa(const utils::Vector2d& globalForce) {
	// Vector de posibles trayectorias circulares
	static std::vector<geometry_msgs::Twist> commands;
	if (commands.empty()) {
		for (double lin = 0; lin<= 0.6; lin+=0.05) {
			for (double ang = -0.8; ang<=0.8; ang+=0.05) {
				geometry_msgs::Twist cmdVel;
				cmdVel.linear.x = lin;
				cmdVel.angular.z = ang;
				commands.push_back(cmdVel);
			}
		}
	}

	// Calculamos el vector de referencia de velocidad instantanea 
	utils::Vector2d velocityRef = robot.velocity + globalForce * robot.dt;

	// Si el vector de referencia excede la velocidad maxima, ajustamos
	if (velocityRef.norm() > robot.maxVel) {
		velocityRef.normalize();
		velocityRef *= robot.maxVel;
	}
	
	// Calculamos la pose que tendria el robot si pudiera seguir el vector de referencia por un tiempo dt
	utils::Vector2d posRef = robot.position + velocityRef * robot.dt;
	utils::Angle yawRef = velocityRef.angle();

	
	// Para cada trayectoria circular, calculamos la pose resultante tras un tiempo dt 
	// y comparamos con la pose de referencia
	double minDist = 999999999;
	geometry_msgs::Twist cmdVel;
	for (unsigned i=0;i<commands.size();i++) {
		double imd = commands[i].linear.x * robot.dt;
		utils::Vector2d inc(imd * std::cos(robot.yaw.toRadian() + commands[i].angular.z*robot.dt*0.5), 
			                imd * std::sin(robot.yaw.toRadian() + commands[i].angular.z*robot.dt*0.5));
		utils::Vector2d pos = robot.position + inc;
		utils::Angle yaw = robot.yaw + utils::Angle::fromRadian(commands[i].angular.z * robot.dt);	

		double dist = K1*(posRef - pos).norm() + K2 * fabs((yaw - yawRef).toRadian()); 
		if (dist < minDist) {
			minDist = dist;
			cmdVel = commands[i];
		}
	}
	robot.cmdVelPublisher.publish(cmdVel);
}


/************************/
/* VISUALIZACIONES RVIZ */
/************************/

void publishGoals(ros::Publisher& goalsPub) {
	visualization_msgs::MarkerArray markers;
	unsigned counter = 0;
	int goalId = 1;
	for (auto it = robot.goals.begin(); it!= robot.goals.end(); ++it) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
	    marker.header.stamp = ros::Time::now();
		marker.action = 0;
		marker.color.a = 1.0;	
		marker.id = counter++;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.color.r = 0;
	    marker.color.g = 1.0;
	    marker.color.b = 1.0;
	    marker.scale.x = 0.3;
		marker.scale.y = 0.3;
		marker.scale.z = 0.3;
		marker.pose.position.x = it->getX();
		marker.pose.position.y = it->getY();
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0;
		marker.pose.orientation.y = 0;		
		marker.pose.orientation.z = 0;		
		marker.pose.orientation.w = 1.0;
		marker.lifetime = ros::Duration(0.2);
		markers.markers.push_back(marker);
		marker.id = counter++;
	    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	    marker.color.r = 1.0;
	    marker.color.g = 1.0;
	    marker.color.b = 0;
	   	marker.pose.position.z = 0.2;
		marker.text = std::to_string(goalId++);
		marker.lifetime = ros::Duration(0.2);
        markers.markers.push_back(marker);
	}
	goalsPub.publish(markers);
}
