#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/Joy.h>

using namespace std;

static inline void clear() { cout << "\033[2J\033[1;1H";}
static inline void clearLine() { cout << "\033[2K"; }
static inline void location(int x, int y) { cout << "\033[" + to_string(y) + ";" + to_string(x) + "f"; }
static inline void underline() { cout << "\033[4m"; }
static inline void clearFormatting() { cout << "\033[m"; }

class Listener
{
public:
	

private:
	
};



int main (int argc, char **argv)
{
    ros::init(argc, argv, "Interface");
	ros::NodeHandle n;
	ros::Rate loop_rate(1000);

	Listener listener;

	ros::Publisher rpm_pub = n.advertise<std_msgs::Float64>("set_RPM", 1000);
	ros::Publisher angle_pub = n.advertise<std_msgs::Float64>("set_angle", 1000);

	std_msgs::Float64 rpm_msg;
	std_msgs::Float64 angle_msg;

	while (ros::ok())
	{
        clear();
		cout << "Menu" << endl;
		cout << "1. Set RPM" << endl;
		cout << "2. Set Angle" << endl;
		cout << endl;
		cout << ">";

		string input;
		getline(cin, input);
		
		cout << endl;

		switch(stoi(input))
		{
		case 1:
			cout << "RPM: ";
			getline(cin, input);
			rpm_msg.data = stof(input);
			rpm_pub.publish(rpm_msg);
			break;

		case 2:
			cout << "Angle: ";
			getline(cin, input);
			angle_msg.data = stof(input);
			//angle_pub.publish(angle_msg);
			break;

		default:
			cout << "Error";
			getline(cin, input);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
