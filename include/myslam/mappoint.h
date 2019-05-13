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

#ifndef MAPPOINT_H
#define MAPPOINT_H
#include <myslam/common_include.h>
namespace myslam
{
class MapPoint 
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    // 该场景点的id号
    unsigned long      id_; 
    // 该场景点的三维位置
    Vector3d    pos_; 
    // 
    Vector3d    norm_;      
    // 描述子
    Mat         descriptor_; 
    // 
    int         observed_times_;    
    int         matched_times_;  
    MapPoint();
    MapPoint( long id, Vector3d position, Vector3d norm );
   
    
    //static MapPoint::Ptr createMapPoint();
};
}
#endif