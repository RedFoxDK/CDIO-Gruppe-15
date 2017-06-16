#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <math.h>
#include <cstdlib>
#include <pthread.h>
#include <CDIO/circle_msg.h>

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate);
void increaseAltitude(ros::Publisher publisher, ros::Rate loop_rate);
void land(ros::Publisher land_pub);
void moveDrone(ros::Publisher publisher, ros::Rate loop_rate);

double calCircleDist();
void ringAltitude(ros::Publisher publisher);
void alingeWithCenter (ros::Publisher publisher);

bool wait_for_navdata = true;
int altitude = 0;
uint state;
float batteryLevel;
float vx = 0, vy = 0, vz = 0;
float rotz = 0;

double Xcenter = 0.0;
double Ycenter = 0.0;
int radius = 0;

bool fly_once = false;

 //vx, vy, vz, az = bewteen -1 and 1 (nothing- more)
geometry_msgs::Twist drone_vector(double new_vx, double new_vy, double new_vz, 
  double new_ax, double new_ay, double new_az, double K) {
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x += new_vx;
  twist_msg.linear.y += new_vy;
  twist_msg.linear.z += new_vz;
  twist_msg.angular.x += new_ax;
  twist_msg.angular.y += new_ay;
  twist_msg.angular.z += new_az;
  return twist_msg;
}

geometry_msgs::Twist reset_vector() {
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0;
  twist_msg.linear.y = 0;
  twist_msg.linear.z = 0;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = 0;
  return twist_msg;
}

void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& msg) {
  batteryLevel = msg->batteryPercent;
  vx = msg->vx;
  vy = msg->vy;
  vz = msg->vz;
  rotz = msg->rotZ;
  altitude = msg->altd;
  wait_for_navdata = false;
  if (msg->state != state) {
    ROS_INFO("State is: %u", state);
    state = msg->state;
  }
}

void circle_callback(const CDIO::circle_msg::ConstPtr& msg) {
  Xcenter = msg->centerX;
  Ycenter = msg->centerY;
  radius = msg->radius;
}

typedef struct _POSITION {
  _POSITION() : x(0.0), y(0.0) {}

  double x;
  double y;
} POSITION;

POSITION pos;

#define cosd(x) (cos(fmod((x),360) * M_PI / 180))

double calCircleDist() {
  int ring_radius = 50 //cm - from ring 1 and 2
  double forcus_l = 503.24;

  double d = (ring_radius * forcus_l) / radius;

  return d;
}

void calculatePosition(double dist)
{
  double dx = cosd(rotz) * dist;
  double dy = (rotz >= 0 ? 1 : -1) * sqrt(pow(dist, 2) - pow(dx, 2));

  std::cout << "dist: " << dist << std::endl;
  std::cout << "dx: " << dx << std::endl;
  std::cout << "dy: " << dy << std::endl;

  pos.x += dx;
  pos.y += dy;
}

void updatePosition() {
  static uint64_t first_tick = (ros::Time::now().toNSec() / 1000000); 
  static uint64_t last_tick = first_tick;

  uint64_t tick_now = (ros::Time::now().toNSec() / 1000000);  // T in milliseconds (10^3)
  uint64_t dt = (tick_now - last_tick);           // delta T

  double dist = static_cast<double>(dt / 1000.0) * vx;

  calculatePosition(dist);
  
  std::cout << "Position: " << pos.x << ", " << pos.y << std::endl;
  std::cout << "Rotation: " << rotz << std::endl;
  std::cout << "Velocities: " << vx << ", " << vy << std::endl;
  std::cout << "Since first: " << ((tick_now - first_tick) / 1000.0) << std::endl << std::endl;

  last_tick = (ros::Time::now().toNSec() / 1000000);
}

bool isTakingOff = true;
bool isLanding = false;
bool isEmergencyLanding = false;

pthread_t key_thread;
pthread_mutex_t key_mutex;

void* wait_for_input_key(void* arg) 
{
  char key = 0;

  while (!isLanding && !isEmergencyLanding) {
    std::cin >> key;

    pthread_mutex_lock(&key_mutex);

    if (key == 'X' || key == 'x')
      isEmergencyLanding = true;

    pthread_mutex_unlock(&key_mutex);
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "drone_fly_4");
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  ros::Subscriber nav_sub = n.subscribe("ardrone/navdata", 10, navdata_callback);
  ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 1);
  ros::Publisher fly_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Subscriber circle_sub = n.subscribe("CDIO/circle_finder", 1000, circle_callback);
  
  while(wait_for_navdata) {
    ros::spinOnce();
  }

  fly_pub.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  int pthread_result = pthread_create(&key_thread, NULL, wait_for_input_key, NULL);
  int pthread_mutex_result = pthread_mutex_init(&key_mutex, NULL);

  // Begin main loop
  while (ros::ok()) {
    if (batteryLevel <= 20) {
      ROS_INFO("Battery level is too low, %f", batteryLevel);
      land(land_pub);
      exit(0);
    }

    pthread_mutex_lock(&key_mutex);

    if (isTakingOff)
    {
      takeoff(takeoff_pub, loop_rate);
    }
    else if (isLanding || isEmergencyLanding)
    {
      static bool reset = false;

      if (!reset)
      {
        fly_pub.publish(drone_vector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        reset = true;
      }

      land(land_pub);
    }
    else
    {
      if (altitude > 750) 
      {
        updatePosition();


        if (Ycenter > 0 && Ycenter > 160 && Ycenter < 350)
        {
          //align
        }else 
        {
          increaseAltitude(fly_pub, loop_rate);
        }
        // Increase height until finding the center of the circle
        // Place drone accordingly, until the camera is in the center of the circle
        // Calculate distance from drone to circle
        // Fly that distance + buffer forward in order to clear the ring.
        // Land / fly to start.

        

        moveDrone(fly_pub, loop_rate);
      }
      else
      {
        increaseAltitude(fly_pub, loop_rate);
      }
    }
  
    pthread_mutex_unlock(&key_mutex);

    loop_rate.sleep();
    ros::spinOnce();
  }

  void* join_status = 0;
  pthread_join(key_thread, &join_status);
  pthread_mutex_destroy(&key_mutex);
  return 0;
}

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate) {
  static int startTakeOff = ros::Time::now().toSec();

  if (ros::Time::now().toSec() - startTakeOff >= 5) {
    std::cout << "Has finished taking off" << std::endl;
    isTakingOff = false;
  }
  
  takeoff_pub.publish(std_msgs::Empty());
}

void increaseAltitude(ros::Publisher publisher, ros::Rate loop_rate) {
  if (altitude >= 1000) {
    std::cout << "Has finsihed increasing Altitude" << std::endl;
    publisher.publish(reset_vector());
  }                            // vx,  vy,  vz,  ax,  ay,  az,   k
  
  publisher.publish(drone_vector(0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0));
}

void land(ros::Publisher land_pub) {
  if (altitude < 20) {
    std::cout << "Has finished landing" << std::endl;
    isLanding = true;
    exit(0);
  }
  
  land_pub.publish(std_msgs::Empty());
}

void moveDrone(ros::Publisher publisher, ros::Rate loop_rate)
{
  std::cout << "Is moving forward" << std::endl;
                               // vx,  vy,  vz,  ax,  ay,  az,   k
  publisher.publish(drone_vector(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

void ringAltitude(ros::Publisher publisher) {
  if (Ycenter > 160 && Ycenter < 350) {
    publisher.publish(reset_vector());

  }

  if (!fly_once) {
    std::cout << "Increasing to same heigh as the circles center" << std::endl;
    publisher.publish(drone_vector(0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0));
    fly_once = true;
  }
}

void alingeWithCenter (ros::Publisher publisher) {
  if (Xcenter >= 315 && Xcenter <= 335) {
    std::cout << "Ready to fly though" << std::endl;
    publisher.publish(reset_vector());
    ros::Duration(3).sleep();
  }

  if (Xcenter < 315 && Xcenter > 0) {
    publisher.publish(drone_vector(0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0));
  }

  if (Xcenter > 335) {
    publisher.publish(drone_vector(0.0, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0));
  }
  if (Xcenter == 0) {

  }
  ros::Duration(0.2).sleep();

  publisher.publish(reset_vector());
  ros::Duration(0.5).sleep();
}