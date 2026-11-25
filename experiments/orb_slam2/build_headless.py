#!/usr/bin/env python3
"""Adapt the pinned upstream checkout for headless OpenCV 4 stereo replay."""
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve()


def edit(path, before, after):
    path = root / path
    source = path.read_text()
    if after and after in source:
        return
    if before not in source:
        raise ValueError(f'Unexpected upstream source for {path}')
    path.write_text(source.replace(before, after, 1))


# Keep Eigen's ABI identical across g2o and ORB-SLAM2.
for library in ('g2o', 'DBoW2'):
    cmake = root / 'Thirdparty' / library / 'CMakeLists.txt'
    cmake.write_text(cmake.read_text().replace(' -march=native', ''))

edit('Thirdparty/DBoW2/CMakeLists.txt', 'find_package(OpenCV 3.0 QUIET)', 'find_package(OpenCV REQUIRED)')
edit('include/MapDrawer.h', '#include<pangolin/pangolin.h>', 'namespace pangolin { class OpenGlMatrix; }')
edit('include/LoopClosing.h', 'std::pair<const KeyFrame*, g2o::Sim3>', 'std::pair<KeyFrame* const, g2o::Sim3>')
edit('include/LoopClosing.h', 'bool mnFullBAIdx;', 'int mnFullBAIdx;')
edit('src/System.cc', '#include <pangolin/pangolin.h>', '#include <stdexcept>')
edit('src/System.cc', 'mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);', 'mpMapDrawer = nullptr;')
edit('src/System.cc', '''    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }''', '''    if(bUseViewer)
        throw std::invalid_argument("This build runs headless");''')
edit('src/System.cc', '''    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }''', '    // No viewer thread in this build.')
edit('src/System.cc', '''    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");''', '''    mptLocalMapping->join();
    mptLoopClosing->join();''')
edit('include/System.h', '    void Shutdown();', '    void Shutdown();\n    void SaveMapPoints(const std::string& filename);')
system = root / 'src/System.cc'
source = system.read_text()
if 'void ORB_SLAM2::System::SaveMapPoints' not in source:
    system.write_text(source + '''
void ORB_SLAM2::System::SaveMapPoints(const std::string& filename)
{
    std::ofstream output(filename.c_str());
    output << std::setprecision(10);
    for (MapPoint* point : mpMap->GetAllMapPoints())
    {
        if (point->isBad()) continue;
        cv::Mat p = point->GetWorldPos();
        output << p.at<float>(0) << " " << p.at<float>(1) << " " << p.at<float>(2) << std::endl;
    }
}
''')
tracking = root / 'src/Tracking.cc'
source = tracking.read_text()
import re
source = re.sub(r'(?m)^ +mpMapDrawer->SetCurrentCameraPose\(.*\);\n', '', source)
source = re.sub(r'(?m)^    if\(mpViewer\)\n    \{\n        mpViewer->RequestStop\(\);\n        while\(!mpViewer->isStopped\(\)\)\n        \{?\s*usleep\(3000\);\n(?:        \}\n)?    \}\n', '', source)
source = source.replace('    if(mpViewer)\n        mpViewer->Release();', '')
if 'mpViewer->' in source or 'mpMapDrawer->' in source:
    raise ValueError('Unexpected viewer use in tracking')
for code in ('RGB2GRAY', 'BGR2GRAY', 'RGBA2GRAY', 'BGRA2GRAY'):
    source = source.replace('CV_' + code, 'cv::COLOR_' + code)
tracking.write_text(source)
edit('src/FrameDrawer.cc', 'CV_GRAY2BGR', 'cv::COLOR_GRAY2BGR')
edit('src/Sim3Solver.cc', 'CV_REDUCE_SUM', 'cv::REDUCE_SUM')
edit('src/Frame.cc', '    sort(vDistIdx.begin(),vDistIdx.end());', '    if(vDistIdx.empty()) return;\n    sort(vDistIdx.begin(),vDistIdx.end());')
edit('include/PnPsolver.h', '#include <opencv2/core/core.hpp>', '#include <opencv2/core/core_c.h>')
edit('include/ORBextractor.h', '#include <opencv/cv.h>', '#include <opencv2/core.hpp>\n#include <opencv2/features2d.hpp>')

# A new stereo map cannot use a velocity or last-frame reference from a
# preceding map which was cleared by a tracking reset.
edit('src/Tracking.cc', '    mState = NO_IMAGES_YET;',
     '    mState = NO_IMAGES_YET;\n    mVelocity.release();\n    mbVO = false;')
edit('src/Tracking.cc', '    UpdateLastFrame();',
     '    if(!mLastFrame.mpReferenceKF || mlRelativeFramePoses.empty()) return false;\n    UpdateLastFrame();')
