#include <ros/ros.h>

class ObjectAvoidance {
public:
    ObjectAvoidance(): nh_(""), private_nh_("~")
    {
        // ROS parameters
        // start point, end point, L/R default, acceptable distance from obstacles
        private_nh_.param<double>("start_pos_x", start_pos_x, 0.0); // boat's starting position in x direction
        private_nh_.param<double>("start_pos_y", start_pos_y, 0.0); // boat's starting position in y direction
        private_nh_.param<double>("end_pos_x", end_pos_x, 0.0); // boat's goal position in x direction
        private_nh_.param<double>("end_pos_y", end_pos_y, 0.0); // boat's goal position in y direction

        private_nh_.param<bool>("avoid_left", avoid_left, true); // boolean for what is the default direction to avoid
        private_nh_.param<double>("safety_dist", safety_dist, 1.0); // minimum distance we want to be away from obstacles

        // ROS subscribers
        prop_map_ = nh_.subscribe("props", 10, &ObjectAvoidance::propMapCallback, this);
        global_pos_ = nh_.subscribe("mavros/global_position/local", 10, &ObjectAvoidance::globalPositionCallback, this);
        task_goal_position_ = nh_.subscribe("task_goal_position", 10, &ObjectAvoidance::goalPositionCallback, this);

        // ROS publishers (need any?)
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep()
        }
    }

    prop_mapper::PropArray findObstacle() {
        prop_mapper::PropArray obstacles;
        for (int i = 0; i < sizeof(props_); i++) {
            // if prop within 5 meters of the front of the boat, add it to obstacles array
            // need to filter out markers that are not obstacles

            // this might depend on testing, but maybe could figure out on my own time
        }
        return obstacles;
    }

    void avoidObstacles(prop_mapper::PropArray obstacles) {
        // store current goal, so we can set goal again when the obstacle is cleared.
        geometry_msgs::Point goal = goal_pos_.point;
        // store obstacle
        prop_mapper::Prop obstacle = obstacles[0];
        // determine the optimal direction to avoid. calculate the shortest distance to either side of the obstacle.
        // will also need to determine if there is room for the boat to travel, if a marker is near the obstacle.
        // once direction determined, set waypoint next to obstacle, then behind obstacle, then to the original waypoint.
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber prop_map_;
    ros::Subscriber global_pos_;
    ros::Subscriber goal_pos_;

    nav_msgs::Odometry current_pos_;
    prop_mapper::PropArray props_;
    task_master::TaskGoalPosition goal_pos_;

    void propMapCallback(const prop_mapper::PropArray msg) {
        props_ = msg;
    }

    void globalPositionCallback(const nav_msgs::Odometry msg) {
        current_pos_ = msg;
        // check to see if any props are in the path of the boat.
        prop_mapper::PropArray obstacles = findObstacle(); // checks for obstacle within 5 meters, returns PropArray with prop, if it exists
        if (sizeof(obstacles.props) > 0) { // if obstacle, avoid it
            avoidObstacles(obstacles);
        }
        else {
            ROS_DEBUG("No obstacles currently found.");
        }
    }

    void goalPositionCallback(const task_master::TaskGoalPosition msg) {
        goal_pos_ = msg;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_avoidance_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
        ros::console::notifyLoggerLevelsChanged();

    ObjectAvoidance object_avoidance;

    object_avoidance.spin();

    return 0;
}