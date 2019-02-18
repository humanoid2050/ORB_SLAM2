/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>


namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile) : 
        mbReset(false),mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false),
        wait_count(0),track_mono_dur(0),track_mono_dur_a(0),track_mono_dur_b(0),dur_grab_image_mono(0)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(MONOCULAR==MONOCULAR)
        cout << "Monocular" << endl;
    else if(MONOCULAR==STEREO)
        cout << "Stereo" << endl;
    else if(MONOCULAR==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpMap, mpKeyFrameDatabase, strSettingsFile);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetLocalMapper(mpLocalMapper);
    
    frame_maker_pool_.setFunction(FrameMaker(strSettingsFile,mpVocabulary,[this](Frame frame){tracking_thread_.dispatch(frame);}));
    frame_maker_pool_.setBlocking();
    frame_maker_pool_.setMaxQueueSize(4);
    frame_maker_pool_.start(2);
    
    tracking_thread_.setFunction(std::bind(&Tracking::Track,mpTracker,std::placeholders::_1));
    tracking_thread_.setBlocking();
    tracking_thread_.setMaxQueueSize(2);
    tracking_thread_.start(1);
}

cv::Mat System::TrackMonocular(const cv::UMat &im, const double &timestamp)
{
    const std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    if(MONOCULAR!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
                wait_count +=1;
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }
    const std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    
    
    cv::Mat Tcw;// = mpTracker->GrabImageMonocular(im,timestamp);
    
    frame_maker_pool_.dispatch(im);
    
    const std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    
    /*
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    */
    const std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
    assert(t2>t1);
    assert(t3>t2);
    assert(t4>t3);
    track_mono_dur_a += (t2 - t1);
    dur_grab_image_mono += (t3-t2);
    track_mono_dur_b += (t4 - t3);
    track_mono_dur += (t4 - t1);
    
    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    
    
    frame_maker_pool_.waitForStop();
    tracking_thread_.waitForStop();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
    
    
    cout << "time in System::TrackMonocular: " << track_mono_dur.count() << endl;
    cout << "> time in System::TrackMonocular pre Tracking::GrabImageMonocular: " << track_mono_dur_a.count() << endl;
    cout << "> time in System::TrackMonocular calling Tracking::GrabImageMonocular: " << dur_grab_image_mono.count() << endl;
    
    cout << "      - time in Tracking::Track: " << mpTracker->d2.count() << endl;
    cout << "      - time in Tracking::Relocalization: " << mpTracker->d3.count() << endl;
    cout << "> time in System::TrackMonocular post Tracking::GrabImageMonocular: " << track_mono_dur_b.count() << endl;
    
    cout << "Tracking iteration count " << mpTracker->frame_count_ << std::endl;
    
}

} //namespace ORB_SLAM
