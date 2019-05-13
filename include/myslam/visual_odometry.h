/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include <myslam/orbextractor.h>


namespace myslam 
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {
        INITIALIZING=-1,
        OK=0,
        LOST
    };
    //VO的当前状态
    VOState                 state_;
    //关键帧和场景点的集合
    Map::Ptr                map_;  
    //参考帧
    Frame::Ptr              ref_;
    //当前帧
    Frame::Ptr              curr_;
    //参考帧中的三维场景点
    vector<cv::Point3f>     pts_3d_ref_;
    //左右图像的ORB提取器
    ORBextractor           *ORBextractorLeft_, *ORBextractorRight_; 
    //当前帧左右图像的关键点
    vector<cv::KeyPoint>    keypoints_curr_left_, keypoints_curr_right_;
    //当前帧左右图像的关键点对应的描述子
    Mat                     descriptors_curr_left_, descriptors_curr_right_;
    //参考帧左图像关键点对应的描述子
    Mat                     descriptors_ref_left_;
    vector<cv::DMatch>      feature_matches_;
    //当前帧的位姿
    SE3                     T_c_r_estimated_;
    // number of inlier features in icp
    int num_inliers_; 
    // number of lost times
    int num_lost_;          
    //图像金字塔的相关变量需要在初始化vo类对象时通过ORBextractorLeft_获取这些值
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;
    
    float match_ratio_;      // ratio for selecting  good matches
    int max_num_lost_;      // max number of continuous lost times
    int min_inliers_;       // minimum inliers
    
    double key_frame_min_rot;   // minimal rotation of two key-frames
    double key_frame_min_trans; // minimal translation of two key-frames
    
    //mbf = bf(yaml)
    float mbf;
    float mb;
    
    //在右图中与左图对应的关键点
    vector<float> mvuRight;
    //左图关键点对应的场景点的深度
    vector<float> mvDepth;

    //构造函数和析构函数
    VisualOdometry();
    ~VisualOdometry();    
    //提取当前帧左图关键点并计算描述子
    void ExtractORB_left(const Mat& imleft);
    //提取当前帧右图关键点并计算描述子
    void ExtractORB_right(const Mat& imright);
    //利用双线程提取左图与右图的关键点
    void extractKeyPoints(const Mat& left, const Mat& right);
    //当前帧左右图像立体匹配
    void ComputeStereoMatches();
    //视觉里程计运行的主要函数
    bool addFrame ( Frame::Ptr frame );
    
    void setRef3DPoints();
    float findDepth(int i);
    void featureMatching();
    void poseEstimationPnP();
    bool checkEstimatedPose();
    bool checkKeyFrame();
    void addKeyFrame();
};
}

#endif // VISUALODOMETRY_H
