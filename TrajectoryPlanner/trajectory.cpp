#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include<sensor_msgs/NavSatFix.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Vector3Stamped.h>
#include<std_msgs/Float32.h>
#include <math.h>

// Took wall folllowing solutions template and made our global trajectory planner node

class WallFollow
{
public:
    WallFollow():
            node_handle_(ros::NodeHandle()),
            lidar_sub_(node_handle_.subscribe("scan", 100, &WallFollow::scan_callback, this)),
            rpy_sub_(node_handle_.subscribe("imu/rpy",100,&WallFollow::rpy_callback,this)),
            gps_sub_(node_handle_.subscribe("/ublox/fix",100,&WallFollow::gps_callback,this)),
            mag_sub_(node_handle_.subscribe("imu/mag",100,&WallFollow::mag_callback,this)),
            drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/teleop", 100)),
            compass_pub_(node_handle_.advertise<std_msgs::Float32>("compass",100)),
            bearing_pub_(node_handle_.advertise<std_msgs::Float32>("bearing",100)),
            distance_pub_(node_handle_.advertise<std_msgs::Float32>("distance",100)),
            error_angle_pub_(node_handle_.advertise<std_msgs::Float32>("error_angle",100)),
            moving_average_gps_pub_(node_handle_.advertise<sensor_msgs::NavSatFix>("moving_average",100))
    {
        node_handle_.getParam("/kp", kp_);
        node_handle_.getParam("/ki", ki_);
        node_handle_.getParam("/kd", kd_);
        prev_error_ = 0.0;
        error_ = 0.0;
        integral_ = 0.0;
        node_handle_.getParam("/desired_distance_left", desired_left_wall_distance_);
        node_handle_.getParam("/lookahead_distance", lookahead_distance_);
        prev_reading_time_ = ros::Time::now().toNSec();
        current_reading_time = ros::Time::now().toNSec();
        node_handle_.getParam("/error_based_velocities", error_based_velocities_);
    }

    /// Returns the distance from obstacle at a given angle from the Laser Scan Message
    /// @param scan_msg - Laser Scan Message
    /// @param angle - Angle in Radians (0 rads -> right in front of the Car)
    /// @return
    double get_range_at_angle(const sensor_msgs::LaserScan::ConstPtr &scan_msg, const double& angle) const
    {
        const double corrected_angle = angle + M_PI;
        const double required_range_index = corrected_angle*scan_msg->ranges.size()/(2*M_PI);
        return scan_msg->ranges[static_cast<int>(required_range_index)];
    }

    /// PID controller to control the steering of the car and adjust the velocity accordingly
    void control_steering()
    {
        prev_reading_time_ = current_reading_time;
        current_reading_time = ros::Time::now().toSec();
        const auto dt = current_reading_time - prev_reading_time_;

        integral_ += error_;

        const double p_control_value = kp_*error_ + kd_*(error_ - prev_error_)/dt + ki_*(integral_);

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        sensor_msgs::NavSatFix templ;
//        int k;
//        for(k = 0; k < 20; ++k)
//        {
//            templ.latitude += avg_lata[k];
//            templ.longitude += avg_lona[k];
//        }
//        templ.latitude = templ.latitude*(180/M_PI)/20;
//        templ.longitude = templ.longitude*(180/M_PI)/20;
//        moving_average_gps_pub_.publish(templ);
//        avg_lat = templ.latitude*(M_PI/180);
//        avg_lon = templ.longitude*(M_PI/180);

//        drive_msg.drive.steering_angle = p_control_value;

//        if(abs(p_control_value) > 0.349)
//        {
//            drive_msg.drive.speed = error_based_velocities_["high"];
//        }
//        else if(abs(p_control_value) > 0.174)
//        {
//            drive_msg.drive.speed = error_based_velocities_["medium"];
//        }
//        else
//        {
//            drive_msg.drive.speed = error_based_velocities_["low"];
//        }
//        drive_pub_.publish(drive_msg);

        prev_error_ = error_;
        //Taking roll and pitch with corrected magnetic data to find compass reading for a tilted compass
        const auto x = magx*cos(Ay)+magz*sin(Ay);
        const auto y = magx*sin(Ax)*sin(Ay)+magy*cos(Ax)-magz*sin(Ax)*cos(Ay);
        const auto thet = std::atan2(fabs(y),fabs(x));
        std_msgs::Float32 temp;
        if(x > 0 && y >0)
            temp.data = thet*(180/M_PI);
        else if(x < 0 && y > 0)
            temp.data = 180-thet*(180/M_PI);
        else if(x < 0 && y < 0)
            temp.data = 180 + thet*(180/M_PI);
        else if(x > 0 && y < 0)
            temp.data = 360 - thet*(180/M_PI);

        compass_pub_.publish(temp);

        const auto xcoord = cos(latdest)*sin(londelt);
        const auto ycoord = cos(lat)*sin(latdest)-sin(lat)*cos(latdest)*cos(londelt);
        const auto b = std::atan2(fabs(xcoord), fabs(ycoord));
        std_msgs::Float32 tempy;
//        if(xcoord > 0 && ycoord >0)
//            tempy.data = b*(180/3.14);
//        else if(ycoord < 0 && xcoord > 0)
//            tempy.data = 180 - b*(180/3.14);
//        else if(ycoord < 0 && xcoord < 0)
//            tempy.data = 180 + b*(180/3.14);
//        else if(ycoord > 0 && xcoord < 0)
//            tempy.data = 360 - b*(180/3.14);
        tempy.data = b*(180/M_PI);
        if (tempy.data < 0)
        {
            tempy.data = 360 + tempy.data;
        }

        bearing_pub_.publish(tempy);
        std_msgs::Float32 tempz;
        tempz.data = temp.data - tempy.data;
        error_angle_pub_.publish(tempz);

        drive_msg.drive.steering_angle = .5*(temp.data*(M_PI/180) - tempy.data*(M_PI/180));

        const auto a = pow(sin(latdelt / 2), 2) + cos(lat) * cos(latdest) * pow(sin(londelt / 2), 2);
        const auto c = 2 * std::atan2(pow(a, 0.5), pow(1-a, 0.5));
        const auto d = 6371 * c;
        std_msgs::Float32 tempx;
        tempx.data = d;
        distance_pub_.publish(tempx);
        if( d < 5*.001)
        {
            drive_msg.drive.speed = 0;
        }
        else
        {
            drive_msg.drive.speed = 1;
        }
        drive_pub_.publish(drive_msg);

    }

    /// Returns value of Error between the required distance and the current distance
    /// @param scan_msg
    /// @param left_distance
    /// @return
    void get_error(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        const auto distance_of_a = get_range_at_angle(scan_msg, 0.7853);
        const auto distance_of_b = get_range_at_angle(scan_msg, 0.7853*2);
        constexpr auto theta = 0.7853;

        const auto alpha = std::atan2(distance_of_a*cos(theta)-distance_of_b,distance_of_a*sin(theta));

        const auto distance_t = distance_of_b*cos(alpha);
        const auto distance_tplus1 = distance_t + lookahead_distance_*sin(alpha);

        error_ = distance_tplus1 - desired_left_wall_distance_ ;
    }

    /// Scan Callback Function
    /// @param scan_msg
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        get_error(scan_msg);
        control_steering();
    }
    /// @param rpy_msg
    void rpy_callback(const geometry_msgs::Vector3Stamped::ConstPtr &rpy_msg)
    {
        geometry_msgs::Vector3 rpy = rpy_msg->vector;
        rpy.x = rpy_msg->vector.x;
        rpy.y= rpy_msg->vector.y;
        rpy.z= rpy_msg->vector.z;
        Ax = rpy.x;
        Ay = rpy.y;
        control_steering();
    }
    /// @param gps_msg
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg)
    {
        lat = gps_msg->latitude * (M_PI/180);
        lon = gps_msg->longitude * (M_PI/180);
        latdest =  34.4100954 * (M_PI/180);
        londest = -119.8610664 * (M_PI/180);
        latdelt = latdest - lat;
        londelt = londest - lon;
//        int n;
//        for(n = 1; n < 20; ++n)
//        {
//            avg_lata[n] = avg_lata[n-1];
//            avg_lona[n] = avg_lona[n-1];
//        }
//        avg_lata[0] = lat;
//        avg_lona[0] = lon;
        control_steering();
    }
    void mag_callback(const geometry_msgs::Vector3Stamped::ConstPtr &mag_msg)
    {
        geometry_msgs::Vector3 mag = mag_msg->vector;
        mag.x = mag_msg->vector.x;
        mag.y= mag_msg->vector.y;
        mag.z= mag_msg->vector.z;
        //Taking hard iron bias offsets and soft iron scaling into correcting magnetic readings
        magx = (mag.x-0.1015)*1.013;
        magy = (mag.y+0.0149)*0.978;
        magz = (mag.z-.1735)*1.0098;
        control_steering();
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber lidar_sub_;
    ros::Publisher drive_pub_;
    ros::Subscriber rpy_sub_;
    ros::Subscriber gps_sub_;
    ros::Publisher compass_pub_;
    ros::Subscriber mag_sub_;
    ros::Publisher bearing_pub_;
    ros::Publisher distance_pub_;
    ros::Publisher error_angle_pub_;
    ros::Publisher moving_average_gps_pub_;

    double kp_, ki_, kd_;
    double prev_error_, error_;
    double integral_;
    double Ax,Ay;
    double magx,magy,magz;
    double lat, lon;
    double latdest, londest;
    double latdelt, londelt;
    double avg_lata[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    double avg_lona[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    double avg_lat, avg_lon;

    double prev_reading_time_;
    double current_reading_time;

    double desired_left_wall_distance_;
    double lookahead_distance_;

    std::map<std::string, double> error_based_velocities_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "wall_follow_node");
    WallFollow wall_follower;
    ros::spin();
    return 0;
}
