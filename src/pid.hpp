#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uuv_gazebo_ros_plugins_msgs/FloatStamped.h"
#include "sensor_msgs/FluidPressure.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <typeinfo>

class PID
{
    public:
        PID();

        template<typename ...T>
            PID(ros::NodeHandle n, float desired_value, float min, float max, std::string conf_file,
                    std::string csv_file, std::string file_header,
                    std::string subscribe, std::vector<std::string> pub)
            :desired_value_{desired_value}, min_{min}, max_{max}
                ,conf_file_{conf_file}, subscribe_{subscribe}, adv_{pub}
        {
            actual_value_ = 0.0;
            error_ = 0.0;
            error_prior_ = 0.0;
            integral_ = 0.0;
            derivative_ = 0.0;
            read_file(conf_file_, KP_, KI_, KD_);
            of_file_.open(csv_file, std::ios_base::app);
            of_file_ << file_header << std::endl;

            for (int i = 0; i < adv_.size(); ++i)
                pub_.push_back(n.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(adv_[i], 1000));

            //Display PID values
            std::cout << "KP = " << KP_;
            std::cout << ", KI = " << KI_;
            std::cout << ", and KD = " << KD_ << std::endl;
        };

        void read_file(std::string filename, float& KP, float& KI, float& KD)
        {
            std::ifstream infile(filename);
            if (!infile)
            {
                std::cerr << "Can't open the file " << filename.c_str() << std::endl;
                exit(1);
            }
            infile >> KP >> KI >> KD;
        }

        void publish_file(float time)
        {
            of_file_ << time << "," << actual_value_ << std::endl;
        }

        void PID_calcul()
        {
            //Error
            error_ = desired_value_ - actual_value_;

            //Calcul integral
            if (abs(error_) > 0.01)
                integral_ += error_ * 0.01;

            //Calcul derivative
            derivative_ = (error_ - error_prior_) / 0.01;


            //Output
            output_ = KP_ * error_ + KI_ * integral_ + KD_ * derivative_;
            error_prior_ = error_;
        }

        float limit()
        {
            if (min_ == 0 && max_ == 0)
              return output_;
            if (abs(output_) > max_)
                return output_ > 0 ? max_ : min_;
            return output_;
        }

        void set_actual_value(float value)
        {
            actual_value_ = value;
        }

        float get_actual_value()
        {
            return actual_value_;
        }

        float get_output()
        {
            return output_;
        }

        std::string get_subscribe()
        {
            return subscribe_;
        }

        void publish(auto msg)
        {
            for (int i = 0; i < adv_.size(); ++i)
                pub_[i].publish(msg);
        }

        void info()
        {
            std::cout << "Actual value is " << actual_value_;
            std::cout << " and searching to reach " << desired_value_ << std::endl;
            std::cout << "Output is " << output_ << std::endl;
            std::cout << "---------------------------------------------------" << std::endl;
        }

    private:
        //Parameters of constructor
        float KP_, KI_, KD_;
        float desired_value_;
        float min_, max_;
        std::string conf_file_;
        std::string subscribe_;

        std::ofstream of_file_;
        float actual_value_;
        float error_prior_;
        float integral_;
        float error_;
        float derivative_;
        float output_;
        std::vector<std::string> adv_;
        std::vector<ros::Publisher> pub_;
};
