#pragma once
#include <opencv2/core.hpp>
#include <array>

namespace cymbergaj { namespace robot { namespace abb {

class AbbController; // forward-declare

class RobotStrikePlanner {
public:
    explicit RobotStrikePlanner(AbbController* ctrl) : ctrl_(ctrl) {}

    // Pozycja krążka w pikselach lub mm (zależnie od Twojej konwencji)
    void strikeAt(const cv::Point2f& puck_pos);

    // Prosty mapping na pozycję TCP w mm (demo)
    static std::array<double,7> mapPuckToTCP(const cv::Point2f& p);

private:
    AbbController* ctrl_{nullptr};
};

}}} // namespace
