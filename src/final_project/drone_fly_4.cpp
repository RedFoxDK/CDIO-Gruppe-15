#include <ros/ros.h>
#include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>
#include <pthread.h>

void takeoff(ros::Publisher takeoff_pub, ros::Rate loop_rate);
void increaseAltitude(ros::Publisher publisher, ros::Rate loop_rate);
void land(ros::Publisher land_pub);
void moveDrone(ros::Publisher publisher, ros::Rate loop_rate);

bool wait_for_navdata = true;
int altitude = 0;
uint state;
float batteryLevel;
float vx = 0, vy = 0, vz = 0;

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
  altitude = msg->altd;
  wait_for_navdata = false;
  if (msg->state != state) {
    ROS_INFO("State is: %u", state);
    state = msg->state;
  }
}

typedef struct _POSITION {
  int x;
  int y;
  int z;
} POSITION;

POSITION pos = { 0 };

void updatePosition() {
  static uint64_t first_tick = ros::Time::now().toNSec(); 
  static uint64_t last_tick = first_tick;

  uint64_t tick_now = ros::Time::now().toNSec();  // T in nanoseconds (10^9)
  uint64_t dt = (tick_now - last_tick);           // delta T in nanoseconds

  pos.x += static_cast<int>(static_cast<double>(dt / 1000000.0) * static_cast<double>(vx * / 1000.0));
  pos.y += static_cast<int>(static_cast<double>(dt / 1000000.0) * static_cast<double>(vy * / 1000.0));
  pos.z += static_cast<int>(static_cast<double>(dt / 1000000.0) * static_cast<double>(vz * / 1000.0));

  std::cout << "Position: " << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
  std::cout << "Velocities: " << vx << ", " << vy << ", " << vz << std::endl;
  std::cout << "Tick: " << last_tick << ", " << dt << std::endl;
  std::cout << "Since first: " << ((tick_now - first_tick) / 1000.0) << std::endl;

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
  ros::Publisher fly_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  
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
      land(land_pub);
    }
    else
    {
      if (altitude > 750) 
      {
        updatePosition();
        moveDrone(fly_pub, loop_rate);
      }
      else
      {
        increaseAltitude(fly_pub, loop_rate);
      }
    }
  
    pthread_mutex_unlock(&key_mutex);

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
  //loop_rate.sleep();
}

void increaseAltitude(ros::Publisher publisher, ros::Rate loop_rate) {
  if (altitude >= 1000) {
    std::cout << "Has finsihed increasing Altitude" << std::endl;
    publisher.publish(reset_vector());
  }                            // vx,  vy,  vz,  ax,  ay,  az,   k
  
  publisher.publish(drone_vector(0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0));
  //loop_rate.sleep();
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
  publisher.publish(drone_vector(0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}