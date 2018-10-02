// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>

namespace task_handler {

    class targetManager {

        private: double target_value_;
        private: double time_interval_;
        private: int seed_;
        private: double initial_time_stamp_;
        private: bool isTagetAcheived_;
        private: double band_;
        private: int targetAcheivedCount_;
        private: int targetCount_;

        public: targetManager(double target_value, double time_interval, double band, int targetCount);
        public: ~targetManager();

        public: void setTargetValue(double target_value);
        public: void setTimeInterval(double time_interval);
        public: bool targetAcheived();
        public: void targetStatus(double current_value);
        public: void setTargetCount(int count);
        public: void setBand(double value);
        public: void reset();

    };

}