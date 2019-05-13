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
#ifndef CONFIG_H
#define CONFIG_H
#include <myslam/common_include.h>

namespace myslam
{
class Config 
{
public:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;
  Config() {};
  ~Config();
  static void setParameterFile( const std::string& filename );
  template< typename T >
  static T get( const std::string& key )
  {
    return T( Config::config_->file_[key] );
  }
};
  
}
#endif