/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/



#include "PangolinDSOViewer.h"
#include "KeyFrameDisplay.h"

#include "util/settings.h"
#include "util/globalCalib.h"
#include "util/DatasetReader.h"
#include "FullSystem/HessianBlocks.h"
#include "FullSystem/FullSystem.h"
#include "FullSystem/ImmaturePoint.h"
#include <thread>

#include <iostream>
#include <fstream>
#include <string>

CameraPoseVisualization cameraposevisual(1, 0, 0, 1);

namespace dso
{
namespace IOWrap
{

PangolinDSOViewer::PangolinDSOViewer(int w, int h, ros::NodeHandle &n, bool startRunThread)
{
	this->w = w;
	this->h = h;
	running=true;


	{
		boost::unique_lock<boost::mutex> lk(openImagesMutex);
		internalVideoImg = new MinimalImageB3(w,h);
		internalKFImg = new MinimalImageB3(w,h);
		internalResImg = new MinimalImageB3(w,h);
		internalVideoImg_Right = new MinimalImageB3(w,h);
		
		videoImgChanged=kfImgChanged=resImgChanged=true;

		internalVideoImg->setBlack();
		internalVideoImg_Right->setBlack();
		internalKFImg->setBlack();
		internalResImg->setBlack();
	}


	{
		currentCam = new KeyFrameDisplay();
	}

  pub_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud", 1000);
  pub_path = n.advertise<nav_msgs::Path>("path", 1000);
  pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
  pub_key_poses = n.advertise<visualization_msgs::Marker>("key_poses", 1000);
  pub_camera_pose = n.advertise<nav_msgs::Odometry>("camera_pose", 1000);
  pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
  pub_keyframe_pose = n.advertise<nav_msgs::Odometry>("keyframe_pose", 1000);

  cameraposevisual.setScale(0.3);
  cameraposevisual.setLineWidth(0.05);

	needReset = false;

    if(startRunThread)
        runThread = boost::thread(&PangolinDSOViewer::run, this);

}


PangolinDSOViewer::~PangolinDSOViewer()
{
	close();
	runThread.join();
}


void PangolinDSOViewer::run()
{
	printf("START PANGOLIN!\n");

  settings_pointCloudMode = 1;
  settings_showKFCameras = false;
  settings_showCurrentCamera = true;
  settings_showTrajectory = true;
  settings_showFullTrajectory = false;
  settings_showActiveConstraints = true;
  settings_showAllConstraints = false;

  settings_sparsity = 1;
  settings_scaledVarTH = 0.001;
  settings_absVarTH = 0.001;
  settings_minRelBS = 0.1;

    // Default hooks for exiting (Esc) and fullscreen (tab).
  while( running && ros::ok() )
	{
    if(setting_render_display3D && keyframes.size() != 0)
		{
			boost::unique_lock<boost::mutex> lk3d(model3DMutex);
      int refreshed=0;
      int refreshed_pgl=0;
      sensor_msgs::PointCloud PC_all;
      PC_all.header.stamp.fromSec(keyframes.front()->time);
      PC_all.header.frame_id = "stereo_dso";
			for(KeyFrameDisplay* fh : keyframes)
			{
        std::vector<Eigen::Vector4f> pc;
        float blue[3] = {0,0,1};

        pc = fh->getPoint(refreshed < 10, settings_scaledVarTH, settings_absVarTH,
                          settings_pointCloudMode, settings_minRelBS, settings_sparsity);
        //std::cout<<"pc size: "<<pc.size()<<std::endl;

        if(pc.size()==0)
          refreshed++;
        else
        {
          for(int j=0; j<pc.size(); j++)
          {
            geometry_msgs::Point32 p;
            p.x = pc[j][0];
            p.y = pc[j][1];
            p.z = pc[j][2];
            PC_all.points.push_back(p);
          }
        }

      }

      pub_cloud.publish(PC_all);

			lk3d.unlock();
		}

      std::chrono::milliseconds dura(2);
      std::this_thread::sleep_for(dura);

      if(needReset) reset_internal();
  }


	printf("QUIT Pangolin thread!\n");
	printf("I'll just kill the whole process.\nSo Long, and Thanks for All the Fish!\n");

	exit(1);
}


void PangolinDSOViewer::close()
{
	running = false;
}

void PangolinDSOViewer::join()
{
	runThread.join();
	printf("JOINED Pangolin thread!\n");
}

void PangolinDSOViewer::reset()
{
	needReset = true;
}

void PangolinDSOViewer::reset_internal()
{
	model3DMutex.lock();
	for(size_t i=0; i<keyframes.size();i++) delete keyframes[i];
	keyframes.clear();
	allFramePoses.clear();
	keyframesByKFID.clear();
	connections.clear();
	model3DMutex.unlock();


	openImagesMutex.lock();
	internalVideoImg->setBlack();
	internalVideoImg_Right->setBlack();
	internalKFImg->setBlack();
	internalResImg->setBlack();
	videoImgChanged= kfImgChanged= resImgChanged=true;
	openImagesMutex.unlock();

	needReset = false;
}


void PangolinDSOViewer::publishKeyframes(
		std::vector<FrameHessian*> &frames,
		bool final,
		CalibHessian* HCalib)
{
	if(!setting_render_display3D) return;
    if(disableAllDisplay) return;

	boost::unique_lock<boost::mutex> lk(model3DMutex);
	for(FrameHessian* fh : frames)
	{
		if(keyframesByKFID.find(fh->frameID) == keyframesByKFID.end())
		{
			KeyFrameDisplay* kfd = new KeyFrameDisplay();
			keyframesByKFID[fh->frameID] = kfd;
			keyframes.push_back(kfd);
		}
		keyframesByKFID[fh->frameID]->setFromKF(fh, HCalib);

	}
}


void PangolinDSOViewer::publishCamPose(FrameShell* frame,
		CalibHessian* HCalib)
{
    if(!setting_render_display3D) return;
    if(disableAllDisplay) return;

	boost::unique_lock<boost::mutex> lk(model3DMutex);
	struct timeval time_now;
	gettimeofday(&time_now, NULL);
	lastNTrackingMs.push_back(((time_now.tv_sec-last_track.tv_sec)*1000.0f + (time_now.tv_usec-last_track.tv_usec)/1000.0f));
	if(lastNTrackingMs.size() > 10) lastNTrackingMs.pop_front();
	last_track = time_now;

	if(!setting_render_display3D) return;

	currentCam->setFromF(frame, HCalib);
  Eigen::Matrix4d Tcw = frame->camToWorld.matrix().cast<double>();
  allFramePoses.push_back(Tcw);

  nav_msgs::Odometry odometry;
//    odometry.header = ;
  odometry.header.stamp.fromSec(frame->timestamp);
  odometry.header.frame_id = "stereo_dso";
  odometry.child_frame_id = "stereo_dso";
  Eigen::Quaterniond tmp_Q;
  tmp_Q = Eigen::Quaterniond(Tcw.block<3,3>(0,0));
  odometry.pose.pose.position.x = Tcw(0,3);
  odometry.pose.pose.position.y = Tcw(1,3);
  odometry.pose.pose.position.z = Tcw(2,3);
  odometry.pose.pose.orientation.x = tmp_Q.x();
  odometry.pose.pose.orientation.y = tmp_Q.y();
  odometry.pose.pose.orientation.z = tmp_Q.z();
  odometry.pose.pose.orientation.w = tmp_Q.w();
  pub_odometry.publish(odometry);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp.fromSec(frame->timestamp);
  pose_stamped.header.frame_id = "stereo_dso";
  pose_stamped.pose = odometry.pose.pose;
  path.header.stamp.fromSec(frame->timestamp);
  path.header.frame_id = "stereo_dso";
  path.poses.push_back(pose_stamped);
  pub_path.publish(path);

  Eigen::Vector3d P = Eigen::Vector3d((double)Tcw(0,3), (double)Tcw(1,3), (double)Tcw(2,3));

  cameraposevisual.reset();
  cameraposevisual.add_pose(P, tmp_Q);
  cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);


}


void PangolinDSOViewer::publishKeyframespPose(FrameHessian* frame)
{

  key_poses.header.stamp.fromSec(frame->shell->timestamp);
  key_poses.header.frame_id = "stereo_dso";
  key_poses.ns = "key_poses";
  key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
  key_poses.action = visualization_msgs::Marker::ADD;
  key_poses.pose.orientation.w = 1.0;
  key_poses.lifetime = ros::Duration();

  //static int key_poses_id = 0;
  key_poses.id = 0; //key_poses_id++;
  key_poses.scale.x = 0.1;
  key_poses.scale.y = 0.1;
  key_poses.scale.z = 0.1;
  key_poses.color.r = 1.0;
  key_poses.color.a = 1.0;

  geometry_msgs::Point pose_marker;
  Eigen::Matrix4f Tcw = frame->shell->camToWorld.matrix().cast<float>();
  pose_marker.x = Tcw(0,3);
  pose_marker.y = Tcw(1,3);
  pose_marker.z = Tcw(2,3);
  key_poses.points.push_back(pose_marker);
  std::cout<<"key pose size: "<<key_poses.points.size()<<std::endl;
  pub_key_poses.publish(key_poses);
}


}
}
