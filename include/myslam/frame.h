/*
 * This file is part of CNN_VO
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
#ifndef FRAME_H
#define FRAME_H

#include <myslam/common_include.h>
#include <myslam/camera.h>
#include <myslam/mappoint.h>
namespace myslam
{
//类的前置声明，因为MapPoint类相互包含  
class MapPoint;

class Frame 
{
public:
    //表示帧的智能指针
    typedef std::shared_ptr<Frame> Ptr;
    //该帧的id号
    unsigned long                  id_;
    //该帧的时间戳
    double                         time_stamp_;
    //该帧相对于世界坐标系的位姿
    SE3                            T_c_w_; 
    //用智能指针表示的相机模型
    Camera::Ptr                    camera_;
    //双目相机采集的该帧的左右图像
    Mat                            image_left_, image_right_;
    //用于判断当前帧是否为关键帧
    bool                           is_key_frame_;
public:
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, Mat color=Mat(), Mat depth=Mat() );
    ~Frame();
    
    static Frame::Ptr createFrame(); 
    
    
    // 计算相机光心相对于世界坐标系的坐标
    Vector3d getCamCenter() const;
    
    void setPose( const SE3& T_c_w );
    
    // 检查某场景点能否被当前帧观测到 
    bool isInFrame( const Vector3d& pt_world );
};
}
#endif
