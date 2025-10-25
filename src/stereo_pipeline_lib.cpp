#include <oakd_pcloud/stereo_pipeline.hpp>
#include <memory>
#include <vector>

#if defined(OAKD_USE_DEPTHAI) && __has_include(<depthai/depthai.hpp>)

void StereoExampe::initDepthaiDev(){
    bool withDepth = true;
    bool outputDepth = false;
    bool outputRectified = true;
    bool lrcheck  = false;
    bool extended = false;
    bool subpixel = false;

    // create Pipeline instance
    _p = std::make_shared<dai::Pipeline>();

    auto monoLeft    = _p->create<dai::node::MonoCamera>();
    auto monoRight   = _p->create<dai::node::MonoCamera>();
    auto xoutLeft    = _p->create<dai::node::XLinkOut>();
    auto xoutRight   = _p->create<dai::node::XLinkOut>();
    auto stereo      = withDepth ? _p->create<dai::node::StereoDepth>() : nullptr;
    auto xoutDepth   = _p->create<dai::node::XLinkOut>();
    auto xoutRectifL = _p->create<dai::node::XLinkOut>();
    auto xoutRectifR = _p->create<dai::node::XLinkOut>();

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if (withDepth) {
        xoutDepth  ->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    int maxDisp = 96;
    if (extended) maxDisp *= 2;
    if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

    stereo->setOutputDepth(outputDepth);
    stereo->setOutputRectified(outputRectified);
    stereo->setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->syncedLeft.link(xoutLeft->input);
    stereo->syncedRight.link(xoutRight->input);
    if(outputRectified)
    {
        stereo->rectifiedLeft.link(xoutRectifL->input);
        stereo->rectifiedRight.link(xoutRectifR->input);
    }
    stereo->depth.link(xoutDepth->input);

    auto colorCam = _p->create<dai::node::ColorCamera>();
    auto xlinkOut = _p->create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    colorCam->setPreviewSize(1280, 720);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->preview.link(xlinkOut->input);

    _dev = std::make_shared<dai::Device>(*_p);
    _dev->startPipeline();

    _opImageStreams.push_back(_dev->getOutputQueue("left", 30, false));
    _opImageStreams.push_back(_dev->getOutputQueue("right", 30, false));
    _opImageStreams.push_back(_dev->getOutputQueue("depth", 30, false));
    _opImageStreams.push_back(_dev->getOutputQueue("rectified_left", 30, false));
    _opImageStreams.push_back(_dev->getOutputQueue("rectified_right", 30, false));
    _opImageStreams.push_back(_dev->getOutputQueue("preview", 30, true));
}

std::vector<std::shared_ptr<dai::DataOutputQueue>> StereoExampe::getExposedImageStreams(){
    return _opImageStreams;
}

#else // OAKD_USE_DEPTHAI

void StereoExampe::initDepthaiDev(){
    // DepthAI not available: leave pipelines empty. Use this when building on machines without DepthAI.
}

std::vector<std::shared_ptr<dai::DataOutputQueue>> StereoExampe::getExposedImageStreams(){
    return {};
}

#endif // OAKD_USE_DEPTHAI
