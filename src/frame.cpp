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

#include <myslam/frame.h>

namespace myslam 
{
  Frame::Frame(): id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
  {
  
  }
  Frame::Frame( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat image_left, Mat image_right)
  :id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), image_left_(image_left), image_right_(image_right), is_key_frame_(false)
  {
    
  }
  Frame::~Frame()
  {

  }  
  Frame::Ptr Frame::createFrame()
  {
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++) );
  }

  void Frame::setPose ( const SE3& T_c_w )
  {
    T_c_w_ = T_c_w;
  }
  Vector3d Frame::getCamCenter() const
  {
    return T_c_w_.inverse().translation();
  }
  bool Frame::isInFrame( const Vector3d& pt_world )
  {
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    if ( p_cam(2,0)<0 ) 
        return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<image_left_.cols 
        && pixel(1,0)<image_left_.rows;
  }

}