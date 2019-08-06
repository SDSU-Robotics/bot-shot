#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include <sensor_msgs/Joy.h>

const float HOME_ANGLE = 34.5;
const float MAX_LAUNCH_ANGLE = 70.0;

using namespace std;

static inline void clear() { cout << "\033[2J\033[1;1H";}
static inline void clearLine() { cout << "\033[2K"; }
static inline void location(int x, int y) { cout << "\033[" + to_string(y) + ";" + to_string(x) + "f"; }
static inline void underline() { cout << "\033[4m"; }
static inline void clearFormatting() { cout << "\033[m"; }

int main (int argc, char **argv)
{
    ros::init(argc, argv, "Interface");
	ros::NodeHandle n;
	ros::Rate loop_rate(1000);

	ros::Publisher rpm_pub = n.advertise<std_msgs::Float64>("set_RPM", 1000);
	ros::Publisher angle_pub = n.advertise<std_msgs::Int64>("set_angle", 1000);

	std_msgs::Float64 rpm_msg;
	std_msgs::Int64 angle_msg;

	while (ros::ok())
	{
        clear();

		cout << "X, LY: Commencement" << endl;
		cout << "B, LY: Vertical Cursor" << endl << endl;
		cout << "Menu" << endl;
		cout << "1. Set RPM" << endl;
		cout << "2. Zero RPM" << endl;
		cout << "3. Set Angle" << endl;
		cout << "4. Send Angle Home" << endl;
		cout << "5. Zero Angle Motor" << endl;
		cout << "6. Exit" << endl;
		cout << endl;
		cout << ">";

		string input;
		getline(cin, input);
		
		cout << endl;
		float angle;

		switch(stoi(input))
		{
		case 1:
			cout << "RPM: ";
			getline(cin, input);
			rpm_msg.data = stof(input);
			rpm_pub.publish(rpm_msg);
			break;

		case 2:
			cout << "RPM = 0";
			rpm_msg.data = 0;
			rpm_pub.publish(rpm_msg);
			break;

		case 3:
			cout << "Angle (deg): ";
			getline(cin, input);
			angle = stof(input);
			if (angle > MAX_LAUNCH_ANGLE || angle < HOME_ANGLE)
			{
				cout << "Invalid angle" << endl;
				break;
			}
			angle_msg.data = (stof(input) - HOME_ANGLE) * 4096.0 * 100.0 * 85.0 / 42.0 / 360.0;
			angle_pub.publish(angle_msg);
			break;
		
		case 4:
			cout << "Sending Launcher Home";
			angle = 34.5;
			if (angle > MAX_LAUNCH_ANGLE || angle < HOME_ANGLE)
			{
				cout << "Invalid angle" << endl;
				break;
			}
			angle_msg.data = (34.5 - HOME_ANGLE) * 4096.0 * 100.0 * 85.0 / 42.0 / 360.0;
			angle_pub.publish(angle_msg);
			break;

		case 5:
			cout << "Zeroed" << endl;
			angle_msg.data = -1.0;
			angle_pub.publish(angle_msg);
			angle_msg.data = 0.0;
			angle_pub.publish(angle_msg);
			break;

		case 6:
			rpm_msg.data = 0;
			rpm_pub.publish(rpm_msg);
			ros::shutdown();
			cout << "Exiting" << endl;
			exit(0);

		default:
			cout << "Error";
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}