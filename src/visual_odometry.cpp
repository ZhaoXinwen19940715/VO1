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
#include <myslam/visual_odometry.h>
#include <myslam/config.h>
#include <myslam/orbextractor.h>
#include <algorithm>
#include <boost/timer.hpp>
/*初始化时需要的量ORBextractor，mbf = bf，图像金字塔相关的变量*/
namespace myslam 
{
  VisualOdometry::VisualOdometry():state_ ( INITIALIZING ), ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 )
  {
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = Config::get<double> ( "keyframe_translation" );
    mbf                 = Config::get<double> ( "Camera.mbf" );
    float fx_           = Config::get<float> ("Camera.fx");
    mb                  = mbf/fx_;
    int nFeatures       = Config::get<int> ("ORBextractor.nFeatures");
    float fScaleFactor  = Config::get<float> ("ORBextractor.scaleFactor");
    int nLevels         = Config::get<int> ("ORBextractor.nLevels");
    int fIniThFAST      = Config::get<int> ("ORBextractor.iniThFAST");
    int fMinThFAST 	= Config::get<int> ("ORBextractor.minThFAST");
    ORBextractorLeft_	= new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    ORBextractorRight_  = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    mnScaleLevels       = ORBextractorLeft_->GetLevels();
    mfScaleFactor       = ORBextractorLeft_->GetScaleFactor();
    mfLogScaleFactor    = log(mfScaleFactor);
    mvScaleFactors      = ORBextractorLeft_->GetScaleFactors();
    mvInvScaleFactors   = ORBextractorLeft_->GetInverseScaleFactors();
    mvLevelSigma2       = ORBextractorLeft_->GetScaleSigmaSquares();
    mvInvLevelSigma2    = ORBextractorLeft_->GetInverseScaleSigmaSquares();
  }
  
  VisualOdometry::~VisualOdometry()
  {

  }
  
  bool VisualOdometry::addFrame ( Frame::Ptr frame )
  {
    switch ( state_ )
    {
      case INITIALIZING:
      {
	state_ = OK;
	curr_ = ref_ = frame;
	map_->insertKeyFrame ( frame );
	extractKeyPoints(curr_->image_left_, curr_->image_right_);
	ComputeStereoMatches();
	setRef3DPoints();
	break;
      }
      case OK:
      {
	curr_ = frame;
	extractKeyPoints(curr_->image_left_, curr_->image_right_);
	ComputeStereoMatches();
	featureMatching();
	poseEstimationPnP();
	if ( checkEstimatedPose() == true )
	{
	    curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  // T_c_w = T_c_r*T_r_w 
            ref_ = curr_;
            setRef3DPoints();
            num_lost_ = 0;
            if ( checkKeyFrame() == true ) // is a key-frame
            {
                addKeyFrame();
            }
	}
	else // bad estimation due to various reasons
        {
            num_lost_++;
            if ( num_lost_ > max_num_lost_ )
            {
                state_ = LOST;
            }
            return false;
        }
        break;
      }
    case LOST:
    {
        cout<<"vo has lost."<<endl;
        break;
    }
    }
        return true;
  }
    
  void VisualOdometry::ExtractORB_left(const Mat& imleft)
  {
    (*ORBextractorLeft_)(imleft,cv::Mat(),keypoints_curr_left_,descriptors_curr_left_);
  }
  
  void VisualOdometry::ExtractORB_right(const Mat& imright)
  {
    (*ORBextractorRight_)(imright,cv::Mat(),keypoints_curr_right_,descriptors_curr_right_);
  }
  
  //调用该函数时传入两个成员变量ref_->image_left_ ref_->image_right_
  void VisualOdometry::extractKeyPoints(const Mat& left, const Mat& right)
  {
    ExtractORB_left(left);
    ExtractORB_right(right);
  }
  
  void VisualOdometry::ComputeStereoMatches()
  {
    const int TH_HIGH = 100;
    const int TH_LOW = 50;
    
    mvuRight = vector<float>(keypoints_curr_left_.size(),-1.0f);
    
    mvDepth = vector<float>(keypoints_curr_left_.size(),-1.0f);

    const int thOrbDist = (TH_HIGH+TH_LOW)/2;

    const int nRows = ORBextractorLeft_->mvImagePyramid[0].rows;

    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());
    
    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);
    
    const int Nr = keypoints_curr_right_.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = keypoints_curr_right_[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[keypoints_curr_right_[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);
        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    //mb = mbf/fx
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(keypoints_curr_left_.size());
    
    for(int iL=0; iL<keypoints_curr_left_.size(); iL++)
    {
        const cv::KeyPoint &kpL = keypoints_curr_left_[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = descriptors_curr_left_.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = keypoints_curr_right_[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = descriptors_curr_right_.row(iR);

		const int dist = ORBextractor::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }
        
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = keypoints_curr_right_[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = ORBextractorLeft_->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= ORBextractorRight_->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = ORBextractorRight_->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

      for(int i=vDistIdx.size()-1;i>=0;i--)
      {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
      }
    }
  }
  
  void VisualOdometry::setRef3DPoints()
 {

    pts_3d_ref_.clear();
    descriptors_ref_left_= Mat();
    for ( size_t i=0; i<keypoints_curr_left_.size(); i++ )
    {
        double d = findDepth(i); 
        if ( d > 0)       
           {
	     Vector3d p_cam = ref_->camera_->pixel2camera(
                Vector2d(keypoints_curr_left_[i].pt.x, keypoints_curr_left_[i].pt.y), d
            );
            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
            descriptors_ref_left_.push_back(descriptors_curr_left_.row(i));	    
           }
    }
 }  

 float VisualOdometry::findDepth(int i)
 {
   return mvDepth[i];
 }

 void VisualOdometry::featureMatching()
 {
   
   vector<cv::DMatch> matches;
   cv::BFMatcher matcher ( cv::NORM_HAMMING );
   
   matcher.match ( descriptors_ref_left_, descriptors_curr_left_, matches );
   float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;
   feature_matches_.clear();
   for ( cv::DMatch& m : matches )
    {
        if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
        {
            feature_matches_.push_back(m);
        }
    }
    cout<<"good matches: "<<feature_matches_.size()<<endl;
 }
 
void VisualOdometry::poseEstimationPnP()
{
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    for ( cv::DMatch m:feature_matches_ )
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx] );
        pts2d.push_back( keypoints_curr_left_[m.trainIdx].pt );
    }
    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0,0,1
    );

    Mat rvec, tvec, inliers;
    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
    cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
}

bool VisualOdometry::checkEstimatedPose()
{
      // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
        cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if ( rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans )
        return true;
    return false;
}

void VisualOdometry::addKeyFrame()
{
    cout<<"adding a key-frame"<<endl;
    map_->insertKeyFrame ( curr_ );
}
}
