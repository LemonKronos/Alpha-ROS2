#pragma once
#include "global_utils/system_config.hpp"


class TopicStatus {
public:
    TopicStatus(uint8_t allow_threshold);
    ~TopicStatus();

    void missed();
    bool is_lost();
    void reset();

private:
    uint8_t allow_threshold;
    uint8_t missed_count;
    bool lost;
};
