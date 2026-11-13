#include "global_utils/utils/pipeline.hpp"

TopicStatus::TopicStatus(uint8_t allow_threshold) {
    this->allow_threshold = allow_threshold;
    this->lost = false;
}

TopicStatus::~TopicStatus() {}

void TopicStatus::missed() {
    if(missed_count > 0)
        missed_count--;
    else lost = true;
}

bool TopicStatus::is_lost() {
    return lost;
}

void TopicStatus::reset() {
    missed_count = allow_threshold;
    lost = false;
}
