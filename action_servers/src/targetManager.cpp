#include <targetManager.h>

using namespace task_handler; 

        targetManager::targetManager(double target_value, double time_interval, double band, int targetCount) {
            this->target_value_ = target_value;
            this->seed_ = 0;
            this->time_interval_ = time_interval;
            this->initial_time_stamp_ = 0;
            this->isTagetAcheived_ = false;
            this->band_ = band;
            this->targetAcheivedCount_ = 0;
            this->targetCount_ = targetCount;
        }
        targetManager::~targetManager() {}

        void targetManager::setTargetValue(double target_value) {
            this->target_value_ = target_value;
        }

        void targetManager::setTimeInterval(double time_interval) {
            this->time_interval_ = time_interval;
        }

        void targetManager::setTargetCount(int count) {
            this->targetCount_ = count;
        }

        void targetManager::setBand(double value) {
            this->band_ = value;
        }

        void targetManager::targetStatus(double current_value) {
            if (this->seed_ == 0) {
                this->initial_time_stamp_ = ros::Time::now().toSec();
                this->seed_++;
            }
            if (current_value <= this->target_value_ + this->band_ && current_value >= this->target_value_ - band_) {
                this->targetAcheivedCount_++;
            }

            if (this->targetAcheivedCount_ == this->targetCount_ && ros::Time::now().toSec() <= this->initial_time_stamp_ + this->time_interval_) {
                this->isTagetAcheived_ = true;
            }
        }

        bool targetManager::targetAcheived() {
            return this->isTagetAcheived_;
        }

        void targetManager::reset() {
            this->seed_ = 0;
            this->initial_time_stamp_ = 0;
            this->isTagetAcheived_ = false;
            this->targetAcheivedCount_ = 0;            
        }
