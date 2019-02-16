#ifndef FRAME_MAKER_H
#define FRAME_MAKER_H

#include "Frame.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include <functional>

namespace ORB_SLAM2
{

class FrameMaker
{
public:
    FrameMaker();

    FrameMaker(const string &strSettingPath, ORBVocabulary* voc, std::function<void(Frame)> transfer_func);
    
    void operator() (const cv::UMat &imGray);
    
    
protected:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft;
    
    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    cv::Mat mDistCoef;
    
    // Stereo baseline multiplied by fx.
    float mbf;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;
    
    std::function<void(Frame)> transfer_func_;
};

}

#endif
