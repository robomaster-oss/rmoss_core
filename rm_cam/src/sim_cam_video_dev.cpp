/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/

#include <thread>
#include "rm_cam/sim_cam_video_dev.hpp"

using namespace cv;
using namespace std;
using namespace rm_cam;

SimCamVideoDev::SimCamVideoDev(const std::string viedo_path){
    current_frame=0;
    is_open_=false;
    video_path_ = viedo_path;
}

SimCamVideoDev::~SimCamVideoDev(){

}

bool SimCamVideoDev::open(){
    if(cap_.open(video_path_)){
        cam_height_=cap_.get(CAP_PROP_FRAME_HEIGHT);
        cam_width_=cap_.get(CAP_PROP_FRAME_WIDTH);
        total_frames_=cap_.get(CAP_PROP_FRAME_COUNT);
        cam_fps_ = cap_.get(CAP_PROP_FPS);
        is_open_=true;
        return true;
    }else{
        return false;
    }
}


bool SimCamVideoDev::isOpened(){
    return is_open_;
}

int SimCamVideoDev::capImg(cv::Mat &img){
    if(is_open_){
        if(cap_.read(img)){
            current_frame++;
            if(current_frame>total_frames_-2){
                current_frame=0;
                cap_.set(CAP_PROP_POS_FRAMES,0);
            }
            return 0;
        }else{
            return -2;
        }
        return -1;
    }else{
        cout<<"video sim cam -error!"<<endl;
        std::this_thread::sleep_for( std::chrono::milliseconds(500));
        return -1;
    }
}

bool SimCamVideoDev::setParameter(CamParameter parameter,int value){
    switch (parameter) {
        case Fps:
            cam_fps_ = value;
            return true;
        default:
            return false;
    }
}


bool SimCamVideoDev::getParameter(CamParameter parameter, int& value) {
    switch (parameter) {
        case ResolutionWidth:
            value=cam_width_;
            return true;
        case ResolutionHeight:
            value=cam_height_;
            return true;    
        case Fps:
            value = cam_fps_;
            return true;
        default:
            return false;
    }
}
