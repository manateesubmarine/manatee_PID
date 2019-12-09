#include "pid.hpp"

float pressure;

void chatterCallback(const sensor_msgs::FluidPressure& msg)
{
    pressure = msg.fluid_pressure;//Add the fluid_pressure in PID object
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid");

    ros::NodeHandle n;

    float desired_value, min, max;
    std::string conf_file, of_file, file_header, subscribe;
    std::vector <std::string> pub;

    try{
        //Argument parsing
        desired_value = atof(argv[1]);
        std::cout << "Desired value is "<< desired_value << std::endl;

        min = atof(argv[2]);
        std::cout << "Minimum value is " << min << std::endl;

        max = atof(argv[3]);
        std::cout << "Maximum value is " << max << std::endl;

        conf_file = argv[4];
        std::cout << "Config file is " << conf_file << std::endl;

        of_file = argv[5];
        std::cout << "Output file is " << of_file << std::endl;

        file_header = argv[6];
        std::cout << "The header of the output file is " << file_header << std::endl;

        subscribe = argv[7];
        std::cout << "The node you are subscribing is " << subscribe << std::endl;

        for (size_t i = 8; i < argc; i++)
        {
            pub.push_back(argv[i]);
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "Problem with the parameters :" << e.what() << std::endl;
        exit(1);
    }

    PID pid = PID(n, desired_value, min, max,
            conf_file, of_file, file_header, subscribe,
            pub);

    //SUBSCRIBER
    ros::Subscriber sub = n.subscribe(pid.get_subscribe(), 1000, chatterCallback);

    ros::Rate loop_rate(10);
    float time = 0.0;

    while (ros::ok())
    {
        pid.set_actual_value(pressure);

        pid.publish_file(time);

        time += 0.01;

        //CALCULATE PID
        pid.PID_calcul();

        //MSG AND PUBLISH
        uuv_gazebo_ros_plugins_msgs::FloatStamped msg; //Creation of msg

        msg.data = pid.limit(); //If the angle is to big, reduce it

        //Publish msg
        pid.publish(msg);

        pid.info();

        ros::spinOnce();

        //loop_rate.sleep();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
