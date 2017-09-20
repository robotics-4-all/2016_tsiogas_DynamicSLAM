/*
 This file is part of CrsmSlam.
 CrsmSlam is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 CrsmSlam is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with CrsmSlam. If not, see <http://www.gnu.org/licenses/>.

 Author : Manos Tsardoulias, etsardou@gmail.com
 Organization : AUTH, PANDORA Robotics Team
*/

#include <crsm_slam/crsm_slam.h>

namespace crsm_slam
{
  /**
    @brief Default costructor
    @param argc [int] The number of input arguments
    @param argv [char **] The input arguments
    @return void
   **/
  CrsmSlam::CrsmSlam(int argc, char **argv): Steps(15), ObjectArea(0.24)
  {
    updateParameters();
    
    robotPose.x = 0;
    robotPose.y = 0;
    robotPose.theta = 0;
    
		bestTransformation.dx = 0;
		bestTransformation.dy = 0;
		bestTransformation.dth = 0;
		
    staticMap = CrsmMap(slamParams.map_size);
    dynamicMap = CrsmMap(slamParams.map_size);
    
    expansion.expansions.insert(std::pair<CrsmDirection, int>(RIGHT, 0));
    expansion.expansions.insert(std::pair<CrsmDirection, int>(LEFT, 0));
    expansion.expansions.insert(std::pair<CrsmDirection, int>(UP, 0));
    expansion.expansions.insert(std::pair<CrsmDirection, int>(DOWN, 0));
    
    // Initialize pose publisher
    _posePublisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(slamParams.pose_publish_topic, 1);
  }
  
  /**
    @brief Reads the CRSM slam parameters from the yaml file and fills the CrsmSlamParameters structure
    @return void
   **/
  void CrsmSlam::updateParameters()
  {
    if (n.hasParam("/crsm_slam/occupancy_grid_publish_topic"))
      n.getParam("/crsm_slam/occupancy_grid_publish_topic", slamParams.occupancy_grid_publish_topic);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter occupancy_grid_publish_topic not found. Using Default");
      slamParams.occupancy_grid_publish_topic = "/crsm_slam/staticMap";
    }
    
		if (n.hasParam("/crsm_slam/dynamic_occupancy_grid_publish_topic"))
      n.getParam("/crsm_slam/dynamic_occupancy_grid_publish_topic", slamParams.dynamic_occupancy_grid_publish_topic);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter dynamic_occupancy_grid_publish_topic not found. Using Default");
      slamParams.dynamic_occupancy_grid_publish_topic = "/crsm_slam/dynamicMap";
    }
    
		if (n.hasParam("/crsm_slam/markers_publish_topic"))
      n.getParam("/crsm_slam/markers_publish_topic/", slamParams.markers_publish_topic);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter markers_publish_topic not found. Using Default");
      slamParams.markers_publish_topic = "/visualization_marker_array";
    }
		
    if (n.hasParam("/crsm_slam/robot_trajectory_publish_topic"))
      n.getParam("/crsm_slam/robot_trajectory_publish_topic", slamParams.robot_trajectory_publish_topic);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter robot_trajectory_publish_topic not found. Using Default");
      slamParams.robot_trajectory_publish_topic = "/crsm_slam/trajectory";
    }
    
    if (n.hasParam("/crsm_slam/trajectory_publisher_frame_id"))
      n.getParam("/crsm_slam/trajectory_publisher_frame_id", slamParams.trajectory_publisher_frame_id);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter trajectory_publisher_frame_id not found. Using Default");
      slamParams.trajectory_publisher_frame_id = "map";
    }
    
    if (n.hasParam("/crsm_slam/laser_subscriber_topic"))
      n.getParam("/crsm_slam/laser_subscriber_topic", slamParams.laser_subscriber_topic);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter laser_subscriber_topic not found. Using Default");
      slamParams.laser_subscriber_topic = "/crsm_slam/laser_scan";
    }
    
    if (n.hasParam("/crsm_slam/pose_publish_topic"))
      n.getParam("/crsm_slam/pose_publish_topic", slamParams.pose_publish_topic);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter pose_publish_topic not found. Using Default");
      slamParams.pose_publish_topic = "/crsm_slam/pose";
    }
    
    if (n.hasParam("/crsm_slam/publish_tf"))
      n.getParam("/crsm_slam/publish_tf", slamParams.publish_tf);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter publish_tf not found. Using Default");
      slamParams.publish_tf = true;
    }
    
    if (n.hasParam("/crsm_slam/base_footprint_frame"))
      n.getParam("/crsm_slam/base_footprint_frame", slamParams.base_footprint_frame);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter base_footprint_frame not found. Using Default");
      slamParams.base_footprint_frame = "base_footprint_link" ;
    }
    
    if (n.hasParam("/crsm_slam/base_frame"))
      n.getParam("/crsm_slam/base_frame", slamParams.base_frame);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter base_frame not found. Using Default");
      slamParams.base_frame = "base_link";
    }
    
    if (n.hasParam("/crsm_slam/map_frame"))
      n.getParam("/crsm_slam/map_frame", slamParams.map_frame);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter map_frame not found. Using Default");
      slamParams.map_frame = "map";
    }
    
    if (n.hasParam("/crsm_slam/hill_climbing_disparity"))
      n.getParam("/crsm_slam/hill_climbing_disparity", slamParams.disparity);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter hill_climbing_disparity not found. Using Default");
      slamParams.disparity = 40;
    }
    
    if (n.hasParam("/crsm_slam/slam_container_size"))
      n.getParam("/crsm_slam/slam_container_size", slamParams.map_size);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter slam_container_size not found. Using Default");
      slamParams.map_size = 500;
    }
    
    if (n.hasParam("/crsm_slam/slam_occupancy_grid_dimentionality"))
      n.getParam("/crsm_slam/slam_occupancy_grid_dimentionality", slamParams.ocgd);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter slam_occupancy_grid_dimentionality not found. Using Default");
      slamParams.ocgd = 0.02;
    }
    
    if (n.hasParam("/crsm_slam/map_update_density"))
      n.getParam("/crsm_slam/map_update_density", slamParams.density);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter map_update_density not found. Using Default");
      slamParams.density = 40.0;
    }
    
    if (n.hasParam("/crsm_slam/map_update_obstacle_density"))
      n.getParam("/crsm_slam/map_update_obstacle_density", slamParams.obstacle_density);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter map_update_obstacle_density not found. Using Default");
      slamParams.obstacle_density = 3.0;
    }
    
    if (n.hasParam("/crsm_slam/scan_density_lower_boundary"))
      n.getParam("/crsm_slam/scan_density_lower_boundary", slamParams.scan_selection_meters);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter scan_density_lower_boundary not found. Using Default");
      slamParams.scan_selection_meters = 0.3;
    }
    
    if (n.hasParam("/crsm_slam/max_hill_climbing_iterations"))
      n.getParam("/crsm_slam/max_hill_climbing_iterations", slamParams.max_hill_climbing_iterations);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter max_hill_climbing_iterations not found. Using Default");
      slamParams.max_hill_climbing_iterations = 40000;
    }
    
    if (n.hasParam("/crsm_slam/desired_number_of_picked_rays"))
      n.getParam("/crsm_slam/desired_number_of_picked_rays", slamParams.desired_number_of_picked_rays);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter desired_number_of_picked_rays not found. Using Default");
      slamParams.desired_number_of_picked_rays = 40;
    }
    
    if (n.hasParam("/crsm_slam/occupancy_grid_map_freq"))
      n.getParam("/crsm_slam/occupancy_grid_map_freq", slamParams.occupancy_grid_map_freq);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter occupancy_grid_map_freq not found. Using Default");
      slamParams.occupancy_grid_map_freq = 1.0;
    }
    
    if (n.hasParam("/crsm_slam/robot_pose_tf_freq"))
      n.getParam("/crsm_slam/robot_pose_tf_freq", slamParams.robot_pose_tf_freq);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter robot_pose_tf_freq not found. Using Default");
      slamParams.robot_pose_tf_freq = 5.0;
    }
    
    if (n.hasParam("/crsm_slam/trajectory_freq"))
      n.getParam("/crsm_slam/trajectory_freq", slamParams.trajectory_freq);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter trajectory_freq not found. Using Default");
      slamParams.trajectory_freq = 1.0;
    }
    
    if (n.hasParam("/crsm_slam/robot_width"))
      n.getParam("/crsm_slam/robot_width", slamParams.robot_width);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter robot_width not found. Using Default");
      slamParams.robot_width = 0.6;
    }
    
    if (n.hasParam("/crsm_slam/robot_length"))
      n.getParam("/crsm_slam/robot_length", slamParams.robot_length);
    else
    {
      ROS_WARN("[CrsmSlam] : Parameter robot_length not found. Using Default");
      slamParams.robot_length = 0.75;
    }
  }
  
	/**
    @brief Serves the laser scan messages
    @param msg [sensor_msgs::LaserScanConstPtr&] : The laser rays distances
    @return void
   **/
  void CrsmSlam::fixNewScans(sensor_msgs::LaserScanConstPtr const &msg)
  {
    if(!laser.initialized)
    {
      laser.initialize(msg);
      
      // Try to find distance between laser and robot center, else initialize to zero
      tf::StampedTransform tfTransform;
      
      try
      {
        _listener.waitForTransform(slamParams.base_frame, msg->header.frame_id, ros::Time(0), ros::Duration(1.0));
        _listener.lookupTransform(slamParams.base_frame, msg->header.frame_id, ros::Time(0), tfTransform);
        
        tf::Vector3 origin = tfTransform.getOrigin();
        slamParams.dx_laser_robotCenter = origin[0];
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("[CrsmSlam] Error in tf : %s", ex.what());
        slamParams.dx_laser_robotCenter = 0.0;
      }
      
      //drawInitialPatch();
    }
    
    for(unsigned int i = 0; i < laser.info.laserRays; i++)
    {
      laser.scan.distance[i] = msg->ranges[i];
      
      // Check for invalid measurements
      if( laser.scan.distance[i] < laser.info.laserMin or laser.scan.distance[i] > laser.info.laserMax )
        laser.scan.distance[i] = 0;
        
      laser.scan.p[i].theta = msg->angle_min + ( i * msg->angle_increment );
      laser.scan.p[i].x = laser.scan.distance[i] / slamParams.ocgd * cos(laser.scan.p[i].theta);
      laser.scan.p[i].y = laser.scan.distance[i] / slamParams.ocgd * sin(laser.scan.p[i].theta);
    }
    
    calculateCriticalRays();
    
    for(std::set<int>::const_iterator it = scanSelections.begin(); it != scanSelections.end();)
    {
      if(laser.scan.distance[*it] == 0)
				scanSelections.erase(it);
			else
				++it;
    }
    
    findTransformation();
    
    robotPose.x += bestTransformation.dx;
    robotPose.y += bestTransformation.dy;
    robotPose.theta += bestTransformation.dth;
    
    if(robotPose.theta > pi)
      robotPose.theta -= pi_double;
      
    if(robotPose.theta < -pi)
      robotPose.theta += pi_double;
      
    static int counter = 0;
    
    if(counter++ < 40)
    {
      robotPose.x = 0;
      robotPose.y = 0;
      robotPose.theta = -(laser.info.laserAngleBegin + laser.info.laserAngleEnd) / 2.0;
    }
    
    //if(counter < 10)
    //{
			//meanDensity = 0.5;
		//}
    
    updateMapProbabilities();
    
    publishOGM(msg->header.stamp);
    //publishDOGM(msg->header.stamp);
    
    // Publish tf before trajectory in order to update it
    publishRobotPoseTf(msg->header.stamp);
    publishTrajectory(msg->header.stamp);
  }
  
  /**
    @brief Chooses important rays for RRHC
    @return void
   **/
  void CrsmSlam::calculateCriticalRays()
  {
    meanDensity = 0;
    double maxRay = 0;
    
    for(unsigned int i = 0; i < laser.info.laserRays - 1; i++)
    {
      if(laser.scan.distance[i] > maxRay)
        maxRay = laser.scan.distance[i];
        
      laser.scan.density[i] = fabs(laser.scan.distance[i] - laser.scan.distance[i + 1]);
      meanDensity += laser.scan.density[i];
    }
    
    if(laser.scan.distance[laser.info.laserRays - 1] > maxRay)
      maxRay = laser.scan.distance[laser.info.laserRays - 1];
      
    meanDensity /= (laser.info.laserRays - 1);
    
    scanSelections.clear();
    bigChanges.clear();
    
    for(unsigned int i = 0; i < laser.info.laserRays; i++)
    {
      if(laser.scan.distance[i] < (laser.info.laserMax - 0.05))
      {
        bigChanges.insert(i);
        break;
      }
    }
    
    unsigned int counterLaserRays = 0;
    
    while(counterLaserRays < laser.info.laserRays)
    {
      unsigned int a = 0, b = 0;
      
      if(laser.scan.density[counterLaserRays] > slamParams.scan_selection_meters)
      {
        if(laser.scan.distance[counterLaserRays] < (laser.info.laserMax - 0.05))
        {
          bigChanges.insert(counterLaserRays);
          a = 1;
        }
        
        if(laser.scan.distance[counterLaserRays + 1] < (laser.info.laserMax - 0.05))
        {	
          bigChanges.insert(counterLaserRays + 1);
          b = 1;
        }
      }
      
      if((a + b) == 0)
        counterLaserRays++;
      else
        counterLaserRays += a + b;
    }
    
    for(unsigned int i = laser.info.laserRays - 1; i > 0; i--)
    {
      if(laser.scan.distance[i] < (laser.info.laserMax - 0.05))
      {
        bigChanges.insert(i);
        break;
      }
    }
    
    scanSelections = bigChanges;
    static double parameter = 100;
    unsigned int start, end;
    int count = 0;
    
    for(std::set<int>::const_iterator it = bigChanges.begin(); it != bigChanges.end();)
    {
      double sumDensity = 0;
      start = *it;
      it++;
      
      if(it == bigChanges.end())
				break;
				
      end = *it;
      
      if(laser.scan.distance[start + 1] > (laser.info.laserMax) - 0.5)
				continue;
				
      for(unsigned int i = start; i < end - 1; i++)
        sumDensity += laser.scan.density[i];
        
      double step = sumDensity / (end - start);
      double localPos = 0;
      
      for(unsigned int i = start; i < end; i++)
      {
        localPos += laser.scan.density[i];
        
        if( localPos > (parameter * step / exp(laser.scan.distance[i] / sqrt(maxRay) * 3.0)) )
        {
          scanSelections.insert(i);
          count++;
          localPos = 0;
        }
      }
    }
    
    if(count > slamParams.desired_number_of_picked_rays * 1.25)
      parameter += (count - slamParams.desired_number_of_picked_rays);
    else if(count < slamParams.desired_number_of_picked_rays * 0.75)
      parameter -= (slamParams.desired_number_of_picked_rays - count);
      
    bool isFinished = false;
    
    while(!isFinished)
    {
      isFinished = true;
      std::vector<int> tete;
      
      for(std::set<int>::const_iterator it = scanSelections.begin(); it != scanSelections.end(); it++)
      {
        tete.push_back(*it);
      }
      
      for(unsigned int i = 0; i < tete.size() - 1; i++)
      {
        if((tete[i + 1] - tete[i]) > 25)
        {
          isFinished = false;
          scanSelections.insert((tete[i + 1] + tete[i]) / 2);
        }
      }
    }
  }
  
  /**
    @brief Calculates the transformation (translation & rotation) with RRHC
    @return void
   **/
  void CrsmSlam::findTransformation()
  {
    bestFitness = 0;
    bestTransformation.dx = bestTransformation.dy = bestTransformation.dth = 0;
    
    int tempx, tempy;
    double sinth, costh, tttx, ttty;
    CrsmTransformation temp;
    CrsmHillClimbingPerson trier;
    unsigned int counter = 0;
    
    trier.fitness = 0;
    trier.t.dx = 0;
    trier.t.dy = 0;
    trier.t.dth = 0;
    
    while(true)
    {
      temp.dx = robotPose.x + trier.t.dx;
      temp.dy = robotPose.y + trier.t.dy;
      temp.dth = robotPose.theta + trier.t.dth;
      trier.fitness = 0;
      double tempFitness = 0;
      
      for(std::set<int>::iterator j = scanSelections.begin(); j != scanSelections.end(); j++)
      {
        tempx = laser.scan.p[*j].x;
        tempy = laser.scan.p[*j].y;
        
        sinth = sin(temp.dth);
        costh = cos(temp.dth);
        
        tttx = tempx * costh - tempy * sinth + temp.dx + staticMap.info.originx;
        ttty = tempx * sinth + tempy * costh + temp.dy + staticMap.info.originy;
        
        if( checkExpansion((unsigned int)(tttx - 1), (unsigned int)(ttty - 1), false) or
						checkExpansion((unsigned int)(tttx + 1), (unsigned int)(ttty + 1), false) or
						staticMap.p[(unsigned int)tttx][(unsigned int)ttty] == 127 )
						continue;
						
        tempFitness += ( 10 * ( 255 - staticMap.p[(unsigned int) tttx][(unsigned int) ttty] ) +
															( 255 - staticMap.p[(unsigned int) tttx][(unsigned int) ttty - 1] ) +
															( 255 - staticMap.p[(unsigned int) tttx][(unsigned int) ttty + 1] ) +
															( 255 - staticMap.p[(unsigned int) tttx - 1][(unsigned int) ttty] ) +
															( 255 - staticMap.p[(unsigned int) tttx + 1][(unsigned int) ttty] ) ) / 255.0;
      }
      
      tempFitness /= (14.0 * scanSelections.size());
      trier.fitness = tempFitness;
      
      if(trier.fitness > bestFitness)
      {
        bestFitness = trier.fitness;
        bestTransformation = trier.t;
        
        trier.t.dx += rand() % slamParams.disparity / 4 - slamParams.disparity / 8;
        trier.t.dy += rand() % slamParams.disparity / 4 - slamParams.disparity / 8;
        trier.t.dth += ( rand() % slamParams.disparity - slamParams.disparity / 2.0 ) / 90.0;
      }
      else
      {
        trier.t.dx = rand() % slamParams.disparity / 2 - slamParams.disparity / 4;
        trier.t.dy = rand() % slamParams.disparity / 2 - slamParams.disparity / 4;
        trier.t.dth = (rand() % slamParams.disparity - slamParams.disparity / 2.0) / 45.0;
      }
      
      if(counter++ > slamParams.max_hill_climbing_iterations)
        break;
    }
  }
  
  /**
    @brief Updates map after finding the new robot pose
    @return void
   **/
  void CrsmSlam::updateMapProbabilities()
  {		
    expansion.expansions[RIGHT] = 0;
    expansion.expansions[LEFT] = 0;
    expansion.expansions[UP] = 0;
    expansion.expansions[DOWN] = 0;
    
    // Fix map size according to the laser scans
    for(int i = 0; i < laser.info.laserRays; i++)
    {
      double const xPoint = laser.scan.distance[i] / slamParams.ocgd *
									 cos(robotPose.theta + laser.angles[i]) + robotPose.x + staticMap.info.originx + 3;
									 
      double const yPoint = laser.scan.distance[i] / slamParams.ocgd *
									 sin(robotPose.theta + laser.angles[i]) + robotPose.y + staticMap.info.originy + 3;
									 
      checkExpansion(xPoint, yPoint, true);
    }
    // +3 since we are checking a bit more than the actual measure below
    
    expandMap();
    
    int R = 0;
    std::vector<CrsmPoint> dynamicPoint;
    
    // Set the map's colorization
    for(int measid = 0; measid < laser.info.laserRays; measid++)
    {
      if(laser.scan.distance[measid] == 0 or laser.scan.distance[measid] == laser.info.laserMax)
        continue;
        
			int const dMeasure = laser.scan.distance[measid] / slamParams.ocgd; // Ray distance in pixels
			
			while(R < dMeasure + 1)
			{
				unsigned const xPoint = R * cos(robotPose.theta + laser.angles[measid]) + robotPose.x + staticMap.info.originx;
				unsigned const yPoint = R * sin(robotPose.theta + laser.angles[measid]) + robotPose.y + staticMap.info.originy;
				
				if(checkExpansion(xPoint, yPoint, false))
					break;
					
				if(R < dMeasure)
				{
					int const wr = 1; // Whitening rate
					
					staticMap.p[xPoint][yPoint] =	(staticMap.p[xPoint][yPoint] <= 255 - wr) ? staticMap.p[xPoint][yPoint] + wr : 255;
					
					dynamicMap.p[xPoint][yPoint] = 127;
				}
				
				CrsmPoint const Point(xPoint, yPoint);
				
				// Populate this many cells as obstacle
				if(R > dMeasure - 2 and R < dMeasure + 1)
				{
					int const br = ceil( staticMap.p[xPoint][yPoint] * (255 - staticMap.p[xPoint][yPoint]) / 254.0 ); // Blackening rate
					
					staticMap.p[xPoint][yPoint] -= br;
					
					if( not closeToStatic(Point) )
					{
						drawFreeSpace(Point);
						
						dynamicMap.p[xPoint][yPoint] = 0;
						dynamicPoint.push_back(Point);
					}
				}
				
				R++;
			}
			
      R = 1;
    }
    
    k = object.empty() ? 1 : object.size();
		std::vector<CrsmPoint> centroidsScanned;
		
		while(k > 0 and k <= dynamicPoint.size())
			centroidsScanned = findCentroids(dynamicPoint);
			
		static double pastTime = ros::Time::now().toSec();
		double presentTime = ros::Time::now().toSec();
		
		updateObjects(centroidsScanned, presentTime - pastTime);
		publishMarkers();
		pastTime = presentTime;
  }
  
  void CrsmSlam::drawFreeSpace(CrsmPoint const &Point)
  {
		int const span = 5;
		
		for(int i = -span; i <= span; i++)
		{
			for(int j = -span; j <= span; j++)
			{
				if(checkExpansion(Point.x + i, Point.y + j, false))
					continue;
					
				staticMap.p[ Point.x + i ][ Point.y + j ] = 255;
			}
		}
	}
  
  bool CrsmSlam::closeToStatic(CrsmPoint const &Point)
  {
		int const span = ObjectArea / slamParams.ocgd;
		
		for(int i = -span; i <= span; i++)
		{
			for(int j = -span; j <= span; j++)
			{
				if(checkExpansion(Point.x + i, Point.y + j, false))
					continue;
					
				if( staticMap.p[ Point.x + i ][ Point.y + j ] < 127 )
					return true;
			}
		}
		
		return false;
	}
	
	double CrsmSlam::Distance(CrsmPoint const &Point1, CrsmPoint const &Point2)
	{
		return sqrt( (Point2.x - Point1.x) * (Point2.x - Point1.x) + (Point2.y - Point1.y) * (Point2.y - Point1.y) );
	}
	
  std::vector<CrsmPoint> CrsmSlam::findCentroids(std::vector<CrsmPoint> const &dynamicPoint)
  {
		bool emptyCluster = true;
		double *distanceFromCentroid = new double [k];
		std::vector<CrsmPoint> *cluster = new std::vector<CrsmPoint> [k];
		std::vector<CrsmPoint> currentCentroids, previousCentroids;
		
		while(true)
		{
			currentCentroids.clear();
			
			// Calculate K centroids
			for(int i = 0; i < k; i++)
			{
				if(emptyCluster)
					currentCentroids.push_back( dynamicPoint[rand() % dynamicPoint.size()] );
				else
				{
					int sumX = 0, sumY = 0;
					
					for(std::vector<CrsmPoint>::const_iterator it = cluster[i].begin(); it != cluster[i].end(); it++)
					{
						sumX += (*it).x;
						sumY += (*it).y;
					}
					
					currentCentroids.push_back( CrsmPoint ( sumX / cluster[i].size(), sumY / cluster[i].size() ) );
				}
				
				cluster[i].clear();
			}
			
			emptyCluster = false;
			
			std::set<int> clusterIndex;
			
			// Classify dynamic points in K clusters
			for(int i = 0; i < dynamicPoint.size(); i++)
			{
				for(int j = 0; j < k; j++)
				{
					distanceFromCentroid[j] = Distance( dynamicPoint[i], currentCentroids[j] );
				}
				
				int const indexOfMin =
					std::distance( distanceFromCentroid, std::min_element(distanceFromCentroid, distanceFromCentroid + k) );
					
				clusterIndex.insert(indexOfMin);
				
				cluster[indexOfMin].push_back(dynamicPoint[i]);
			}
			
			if(clusterIndex.size() < k)
				emptyCluster = true;
			else if(currentCentroids == previousCentroids)
				break;
				
			previousCentroids = currentCentroids;
		}
		
		if( centroidsTooClose(currentCentroids) )
		  k--;
		else if( pointTooFar(currentCentroids, cluster) )
			k++;
		else
			k = 0;
			
		delete [] distanceFromCentroid;
		delete [] cluster;
		
		return currentCentroids;
	}
	
	bool CrsmSlam::centroidsTooClose(std::vector<CrsmPoint> const &currentCentroids)
	{
		for(std::vector<CrsmPoint>::const_iterator it1 = currentCentroids.begin(); it1 != currentCentroids.end() - 1; it1++)
		{
			for(std::vector<CrsmPoint>::const_iterator it2 = it1 + 1; it2 != currentCentroids.end(); it2++)
			{
				if( ( Distance(*it1, *it2) * slamParams.ocgd ) <= ObjectArea )
					return true;
			}
		}
		
		return false;
	}
	
	bool CrsmSlam::pointTooFar(std::vector<CrsmPoint> const &currentCentroids, std::vector<CrsmPoint> const *const &cluster)
	{
		for(int i = 0; i < k; i++)
		{
			for(std::vector<CrsmPoint>::const_iterator it = cluster[i].begin(); it != cluster[i].end(); it++)
			{
				if( ( Distance( currentCentroids[i], (*it) ) * slamParams.ocgd ) > ObjectArea )
					return true;
			}
		}
		
		return false;
	}
	
	void CrsmSlam::updateObjects(std::vector<CrsmPoint> &centroidsScanned, double const currentExecutionTime)
	{
		// Match predictions of centroids with actual centroids scanned by the laser
		for(std::vector<CrsmDynamicObject>::iterator it = object.begin(); it != object.end(); it++)
		{
			(*it).predict(currentExecutionTime);
			
			if(centroidsScanned.empty())
				continue;
				
			std::vector<CrsmPoint>::iterator best = centroidsScanned.begin();
			double minDistance = Distance( (*it).getCentroid(), *best );
			
			for(std::vector<CrsmPoint>::iterator it2 = centroidsScanned.begin() + 1; it2 != centroidsScanned.end(); it2++)
			{
				double distance = Distance( (*it).getCentroid(), *it2 );
				
				if(distance < minDistance)
				{
					minDistance = distance;
					best = it2;
				}
			}
			
			if( (minDistance * slamParams.ocgd) <= ObjectArea )
			{
				(*it).update(Steps, *best);
				centroidsScanned.erase(best);
			}
		}
		
		for(std::vector<CrsmDynamicObject>::iterator it = object.begin(); it != object.end();)
		{
			if( (*it).timeUnscanned() > (Steps * currentExecutionTime) )
			{
				deleted.push_back( (*it).getID() );
				deleted.push_back( (*it).getID() + 1);
				
				object.erase(it);
			}
			else
				++it;
		}
		
		// Assign remaining centroids to new dynamic objects
		for(std::vector<CrsmPoint>::const_iterator it = centroidsScanned.begin(); it != centroidsScanned.end(); it++)
		{
			object.push_back( CrsmDynamicObject(*it, id += 2) );
		}
	}
	
	void CrsmSlam::startMarkersPublisher()
	{
    _markersPublisher = n.advertise<visualization_msgs::MarkerArray>(slamParams.markers_publish_topic.c_str(), 1);
	}
	
	void CrsmSlam::publishMarkers()
	{
		visualization_msgs::MarkerArray markerArray;
		
		for(std::vector<int>::const_iterator it = deleted.begin(); it != deleted.end(); it++)
		{
			visualization_msgs::Marker marker;
			
			marker.ns = "marker_ns";
			marker.id = *it;
			marker.action = visualization_msgs::Marker::DELETE;
			
			markerArray.markers.push_back(marker);
		}
		
		deleted.clear();
		
		for(int i = 0; i < ( 2 * object.size() ); i++)
		{
			visualization_msgs::Marker marker;
			
			marker.header.frame_id = slamParams.map_frame;
			marker.header.stamp = ros::Time::now();
			
			marker.ns = "marker_ns";
			marker.id = (i % 2 == 0) ? object[i / 2].getID() : object[i / 2].getID() + 1;
			
			uint32_t shape;
			double scaleX, scaleY, scaleZ;
			
			marker.pose.position.x = object[i / 2].getCentroid().x * slamParams.ocgd - staticMap.info.originx * slamParams.ocgd;
			marker.pose.position.y = object[i / 2].getCentroid().y * slamParams.ocgd - staticMap.info.originy * slamParams.ocgd;
			marker.pose.position.z = 0.0;
			
			if(i % 2 == 0)
			{
				shape = visualization_msgs::Marker::SPHERE;
				
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				
				scaleX = scaleY = scaleZ = 0.4;
			}
			else
			{
				shape = visualization_msgs::Marker::ARROW;
				
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = sin(object[i / 2].getYawAngle() / 2);
				marker.pose.orientation.w = cos(object[i / 2].getYawAngle() / 2);
				
				scaleX = object[i / 2].still() ? 0.0 : 0.5;
				scaleY = object[i / 2].still() ? 0.0 : 0.1;
				scaleZ = 0.0;
			}
			
			marker.type = shape;
			marker.action = visualization_msgs::Marker::ADD;
			
			marker.scale.x = scaleX;
			marker.scale.y = scaleY;
			marker.scale.z = scaleZ;
			
			marker.color.r = (float) (i % 2 == 0) * object[i / 2].getR();
			marker.color.g = (float) (i % 2 == 0) ? object[i / 2].getG() : 1;
			marker.color.b = (float) (i % 2 == 0) * object[i / 2].getB();
			marker.color.a = 1.0f;
			
			markerArray.markers.push_back(marker);
		}
		
		_markersPublisher.publish(markerArray);
	}
	
  bool CrsmSlam::checkExpansion(double x, double y, bool update)
  {
    // +-50 pixels for more strict resizing
    int extra_pix = 50;
    int plus_pix = 100;
    bool changed = false;
    
    if(x < extra_pix)
    {
      if(update and (fabs(x)) > expansion.expansions[LEFT])
      {
        expansion.expansions[LEFT] = fabs(x) + plus_pix;
      }
      
      changed = true;
    }
    
    if(x >= (int) staticMap.info.width - extra_pix)
    {
      if(update and (fabs(x - staticMap.info.width)) > expansion.expansions[RIGHT])
      {
        expansion.expansions[RIGHT] = fabs(x - staticMap.info.width) + plus_pix;
      }
      
      changed = true;
    }
    
    if(y < extra_pix)
    {
      if(update and (fabs(y)) > expansion.expansions[UP])
      {
        expansion.expansions[UP] = fabs(y) + plus_pix;
      }
      
      changed = true;
    }
    
    if(y >= (int) staticMap.info.height - extra_pix)
    {
      if(update and (fabs(y - staticMap.info.height)) > expansion.expansions[DOWN])
      {
        expansion.expansions[DOWN] = fabs(y - staticMap.info.height) + plus_pix;
      }
      
      changed = true;
    }
    
    return changed;
  }
  
  void CrsmSlam::expandMap()
  {
    if( expansion.expansions[LEFT] == 0 and
				expansion.expansions[RIGHT] == 0 and
				expansion.expansions[UP] == 0 and
				expansion.expansions[DOWN] == 0 )
		{
      return;
    }
    
    staticMap.expandMap(expansion);
    dynamicMap.expandMap(expansion);
  }
	
  /**
    @brief Starts the laser subscriber, listening to laser_subscriber_topic 
      from parameters
    @return void
   **/
  void CrsmSlam::startLaserSubscriber()
  {
    clientLaserValues = n.subscribe( slamParams.laser_subscriber_topic.c_str(), 1, &CrsmSlam::fixNewScans, this );
  }

  /**
    @brief Stops the laser subscriber
    @return void
   **/
  void CrsmSlam::stopLaserSubscriber()
  {
    clientLaserValues.shutdown();
  }

  /**
    @brief Starts the OccupancyGrid publisher, posting to 
      occupancy_grid_publish_topic from parameters
    @return void
   **/
  void CrsmSlam::startOGMPublisher()
  {
    _occupancyGridPublisher = n.advertise<nav_msgs::OccupancyGrid>(slamParams.occupancy_grid_publish_topic.c_str(), 1);
  }

  /**
    @brief Stops the OccupancyGrid publisher
    @return void
   **/
  void CrsmSlam::stopOGMPublisher()
  {
    _occupancyGridPublisher.shutdown();
  }

  /**
    @brief Publishes the OccupancyGrid map as nav_msgs::OccupancyGrid, posting 
      with occupancy_grid_map_freq Hz from parameters
    @param e [const ros::TimerEvent&] The timer event
    @return void
   **/
  void CrsmSlam::publishOGM(ros::Time timestamp)
  {
    int width = staticMap.info.width;
    int height = staticMap.info.height;
    
    nav_msgs::OccupancyGrid grid;
    
    grid.header.stamp = timestamp;
    grid.header.frame_id = slamParams.map_frame;
    
    grid.info.resolution = slamParams.ocgd;
    grid.info.width = width;
    grid.info.height = height;
    
    grid.info.origin.position.x = -(staticMap.info.originx * slamParams.ocgd);
    grid.info.origin.position.y = -(staticMap.info.originy * slamParams.ocgd);
    grid.info.origin.orientation.w = 1;
    
    grid.data.resize(width * height);
    
    for(int i = 0; i < width; i++)
    {
      for(int j = 0; j < height; j++)
      {
        if (staticMap.p[i][j] == 127)
          grid.data[j * width + i] = -1;
        else
          grid.data[j * width + i] = 100.0 - (int) (staticMap.p[i][j] * 100.0 / 255.0);
      }
    }
    
    _occupancyGridPublisher.publish(grid);
  }
  
  /**
    @brief Starts the dynamic OccupancyGrid publisher, posting to 
      dynamic_occupancy_grid_publish_topic from parameters
    @return void
   **/
  void CrsmSlam::startDOGMPublisher()
  {
    _dynamicOccupancyGridPublisher = n.advertise<nav_msgs::OccupancyGrid>(slamParams.dynamic_occupancy_grid_publish_topic.c_str(), 1);
  }
  
  /**
    @brief Publishes the dynamic OccupancyGrid map as nav_msgs::OccupancyGrid, posting 
      with occupancy_grid_map_freq Hz from parameters
    @param e [const ros::TimerEvent&] The timer event
    @return void
   **/
  void CrsmSlam::publishDOGM(ros::Time timestamp)
  {
    int width = dynamicMap.info.width;
    int height = dynamicMap.info.height;
    
    nav_msgs::OccupancyGrid grid;
    
    grid.header.stamp = timestamp;
    grid.header.frame_id = slamParams.map_frame;
    
    grid.info.resolution = slamParams.ocgd;
    grid.info.width = width;
    grid.info.height = height;
    
    grid.info.origin.position.x = -(dynamicMap.info.originx * slamParams.ocgd);
    grid.info.origin.position.y = -(dynamicMap.info.originy * slamParams.ocgd);
    grid.info.origin.orientation.w = 1;
    
    grid.data.resize(width * height);
    
    for(int i = 0; i < width; i++)
    {
      for(int j = 0; j < height; j++)
      {
        if (dynamicMap.p[i][j] == 127)
          grid.data[j * width + i] = -1;
        else
          grid.data[j * width + i] = 100.0 - (int) (dynamicMap.p[i][j] * 100.0 / 255.0);
      }
    }
    
    _dynamicOccupancyGridPublisher.publish(grid);
  }
  
  /**
    @brief Returns the map occupancy probability of coordinates (x,y) 
      ranging from 0-255 (0 is occupied, 255 is free)
    @param x [int] : The x coordinate
    @param y [int] : The y coordinate
    @return char probability
   **/
  char CrsmSlam::getMapProbability(int x, int y)
  {
    return staticMap.p[x][y];
  }

  /**
    @brief Returns the map info in a CrsmMapInfo structure
    @return CrsmMapInfo
   **/
  CrsmMapInfo CrsmSlam::getMapInfo()
  {
    return staticMap.info;
  }

  /**
    @brief Returns the robot pose in a CrsmPose structure
    @return CrsmPose
   **/
  CrsmPose CrsmSlam::getRobotPose()
  {
    return robotPose;
  }

  /**
    @brief Returns the laser info in a CrsmLaserInfo structure
    @return CrsmLaserInfo
   **/
  CrsmLaserInfo CrsmSlam::getLaserInfo()
  {
    return laser.info;
  }

  /**
    @brief Returns the robot trajectory in a vector of CrsmPose structures
    @return std::vector<CrsmPose>
   **/
  std::vector<CrsmPose> CrsmSlam::getTrajectory()
  {
    return robotTrajectory;
  }

  /**
    @brief Publishes the Tf robot pose, posting with robot_pose_tf_freq Hz 
      from parameters
    @param e [const ros::TimerEvent&] The timer event
    @return void
   **/
  void CrsmSlam::publishRobotPoseTf(ros::Time timestamp)
  {
    double rx = (robotPose.x - cos(robotPose.theta) * (slamParams.dx_laser_robotCenter / slamParams.ocgd)) * slamParams.ocgd;
    double ry = (robotPose.y - sin(robotPose.theta) * (slamParams.dx_laser_robotCenter / slamParams.ocgd)) * slamParams.ocgd;
    double rth = robotPose.theta;
    
    tf::Vector3 translation(rx, ry, 0);
    tf::Quaternion rotation;
    rotation.setRPY(0, 0, rth);
    
    if(slamParams.publish_tf)
    {
      tf::Transform transform(rotation, translation);
      _slamFrameBroadcaster.sendTransform( tf::StampedTransform( transform, timestamp, slamParams.map_frame, slamParams.base_footprint_frame ) );
    }
    
    // Publish pose on pose topic
    geometry_msgs::PoseWithCovarianceStamped poseOut;
    poseOut.header.stamp = timestamp;
    poseOut.header.frame_id = slamParams.map_frame;
    poseOut.pose.pose.position.x = rx;
    poseOut.pose.pose.position.y = ry;
    tf::quaternionTFToMsg(rotation, poseOut.pose.pose.orientation);
    _posePublisher.publish(poseOut);
    
    // Update trajectory
    geometry_msgs::PoseStamped pathPoint;
    pathPoint.header.stamp = timestamp;
    pathPoint.pose.position.x = rx;
    pathPoint.pose.position.y = ry;
    tf::Quaternion q;
    q.setRPY(0, 0, rth);
    geometry_msgs::Quaternion gq;
    tf::quaternionTFToMsg(q, gq);
    pathPoint.pose.orientation = gq;
    trajectory.poses.push_back(pathPoint);
  }

  /**
    @brief Starts the Trajectory publisher, posting to 
      robot_trajectory_publish_topic from parameters, with 
      trajectory_publisher_frame_id as frame ID.
    @return void
   **/
  void CrsmSlam::startTrajectoryPublisher()
  {
    _pathPublisher = n.advertise<nav_msgs::Path>(slamParams.robot_trajectory_publish_topic.c_str(), 1);
  }

  /**
    @brief Stops the Trajectory publisher.
    @return void
   **/
  void CrsmSlam::stopTrajectoryPublisher()
  {
    _pathPublisher.shutdown();
  }

  /**
    @brief Publishes the robot trajectory as nav_msgs::Path, posting with 
      trajectory_freq Hz from parameters
    @param e [const ros::TimerEvent&] The timer event
    @return void
   **/
  void CrsmSlam::publishTrajectory(ros::Time timestamp)
  {
    trajectory.header.stamp = timestamp;
    trajectory.header.frame_id = slamParams.trajectory_publisher_frame_id;
    _pathPublisher.publish(trajectory);
  }

  //---------------------- Setters for slamParameters ----------------------------//
  /**
    @brief Sets the disparity of CRSM_SlamParameters
    @param disparity [int] Disparity of mutation in pixels at hill climbing
    @return void
   **/
  void CrsmSlam::setDisparity(int disparity)
  {
    slamParams.disparity = disparity;
  }

  /**
    @brief Sets the map_size of CRSM_SlamParameters
    @param size [int] Map size of initial allocated map
    @return void
   **/
  void CrsmSlam::setInitialMapSize(int size)
  {
    slamParams.map_size = size;
  }

  /**
    @brief Sets the ocgd of CRSM_SlamParameters
    @param ocgd [double] [OC]cupancy [G]rid [D]imentionality - the width and 
      height in meters of a pixel
    @return void
   **/
  void CrsmSlam::setOcgd(double ocgd)
  {
    slamParams.ocgd = ocgd;
  }

  /**
    @brief Sets the density of CRSM_SlamParameters
    @param density [double] Map update density (0-127)
    @return void
   **/
  void CrsmSlam::setDensity(double density)
  {
    slamParams.density = density;
  }

  /**
    @brief Sets the obstacle_density of CRSM_SlamParameters
    @param ob_density [double] Coefficient for obstacle update density (0+)
    @return void
   **/
  void CrsmSlam::setObstacleDensity(double ob_density)
  {
    slamParams.obstacle_density = ob_density;
  }

  /**
    @brief Sets the scan_selection_meters of CRSM_SlamParameters
    @param scan_selection_meters [double] Scan density lower boundary for a 
      scan-part identification
    @return void
   **/
  void CrsmSlam::setScanSelectionMeters(double scan_selection_meters)
  {
    slamParams.scan_selection_meters = scan_selection_meters;
  }

  /**
    @brief Sets the max_hill_climbing_iterations of CRSM_SlamParameters
    @param iterations [int] Maximum RRHC iterations
    @return void
   **/
  void CrsmSlam::setMaxHillClimbingIterations(int iterations)
  {
    slamParams.max_hill_climbing_iterations = iterations;
  }

  /**
    @brief Sets the dx_laser_robotCenter of CRSM_SlamParameters
    @param dx [double] Translation in x axis of laser in comparison to robot 
      center
    @return void
   **/
  void CrsmSlam::setDxLaserRobotCenter(double dx)
  {
    slamParams.dx_laser_robotCenter = dx;
  }

  /**
    @brief Sets the occupancy_grid_map_freq of CRSM_SlamParameters
    @param freq [double] The occupancy grid map publishing frequency
    @return void
   **/
  void CrsmSlam::setOccupancyGridMapFreq(double freq)
  {
    slamParams.occupancy_grid_map_freq = freq;
  }

  /**
    @brief Sets the robot_pose_tf_freq of CRSM_SlamParameters
    @param freq [double] The robot pose publishing frequency
    @return void
   **/
  void CrsmSlam::setRobotPoseTfFreq(double freq)
  {
    slamParams.robot_pose_tf_freq = freq;
  }

  /**
    @brief Sets the trajectory_freq of CRSM_SlamParameters
    @param freq [double] The trajectory publishing frequency
    @return void
   **/
  void CrsmSlam::setTrajectoryFreq(double freq)
  {
    slamParams.trajectory_freq = freq;
  }

  /**
    @brief Sets the desired_number_of_picked_rays of CRSM_SlamParameters
    @param rays [int] The desired number of picked rays [algorithm specific]
    @return void
   **/
  void CrsmSlam::setDesiredNumberOfPickedRays(int rays)
  {
    slamParams.desired_number_of_picked_rays = rays;
  }

  /**
    @brief Sets the robot_width of CRSM_SlamParameters
    @param width [double] The robot width
    @return void
   **/
  void CrsmSlam::setRobotWidth(double width)
  {
    slamParams.robot_width = width;
  }

  /**
    @brief Sets the robot_length of CRSM_SlamParameters
    @param length [double] The robot length
    @return void
   **/
  void CrsmSlam::setRobotLength(double length)
  {
    slamParams.robot_length = length;
  }

  /**
    @brief Sets the occupancy_grid_publish_topic of CRSM_SlamParameters
    @param topic [std::string] The occupancy grid publishing topic
    @return void
   **/
  void CrsmSlam::setOccupancyGridPublishTopic(std::string topic)
  {
    slamParams.occupancy_grid_publish_topic = topic;
  }

  /**
    @brief Sets the robot_trajectory_publish_topic of CRSM_SlamParameters
    @param topic [std::string] The trajectory publishing topic
    @return void
   **/
  void CrsmSlam::setRobotTrajectoryPublishTopic(std::string topic)
  {
    slamParams.robot_trajectory_publish_topic = topic;
  }

  /**
    @brief Sets the trajectory_publisher_frame_id of CRSM_SlamParameters
    @param frame_id [std::string] The trajectory frame ID
    @return void
   **/
  void CrsmSlam::setTrajectoryPublisherFrameId(std::string frame_id)
  {
    slamParams.trajectory_publisher_frame_id = frame_id;
  }

  /**
    @brief Sets the laser_subscriber_topic of CRSM_SlamParameters
    @param topic [std::string] The laser subscriber topic
    @return void
   **/
  void CrsmSlam::setLaserSubscriberTopic(std::string topic)
  {
    slamParams.laser_subscriber_topic = topic;
  }

  /**
    @brief Sets the base_footprint_frame of CRSM_SlamParameters
    @param frame [std::string] Holds the base footprint frame - (x,y,yaw)
    @return void
   **/
  void CrsmSlam::setBaseFootprintFrame(std::string frame)
  {
    slamParams.base_footprint_frame = frame;
  }

  /**
    @brief Sets the base_frame of CRSM_SlamParameters
    @param frame [std::string] Holds the base frame
    @return void
   **/
  void CrsmSlam::setBaseFrame(std::string frame)
  {
    slamParams.base_frame = frame;
  }

  /**
    @brief Sets the map_frame of CRSM_SlamParameters
    @param frame [std::string] Holds the map frame
    @return void
   **/
  void CrsmSlam::setMapFrame(std::string frame)
  {
    slamParams.map_frame = frame;
  }

  //------------------- Getters for slamParameters ----------------------//

  /**
    @brief Gets the disparity of CRSM_SlamParameters
    @return int Disparity of mutation in pixels at hill climbing
   **/
  int CrsmSlam::getDisparity()
  {
    return slamParams.disparity;
  }

  /**
    @brief Gets the map_size of CRSM_SlamParameters
    @return int Map size of initial allocated map
   **/
  int CrsmSlam::getInitialMapSize()
  {
    return slamParams.map_size;
  }

  /**
    @brief Gets the ocgd of CRSM_SlamParameters
    @return double [OC]cupancy [G]rid [D]imentionality - 
      the width and height in meters of a pixel
   **/
  double CrsmSlam::getOcgd()
  {
    return slamParams.ocgd;
  }

  /**
    @brief Gets the density of CRSM_SlamParameters
    @return double Map update density (0-127)
   **/
  double CrsmSlam::getDensity()
  {
    return slamParams.density;
  }

  /**
    @brief Gets the obstacle_density of CRSM_SlamParameters
    @return double Coefficient for obstacle update density (0+)
   **/
  double CrsmSlam::getObstacleDensity()
  {
    return slamParams.obstacle_density;
  }

  /**
    @brief Gets the scan_selection_meters of CRSM_SlamParameters
    @return double Scan density lower boundary for a scan-part identification
   **/
  double CrsmSlam::getScanSelectionMeters()
  {
    return slamParams.scan_selection_meters;
  }

  /**
    @brief Gets the max_hill_climbing_iterations of CRSM_SlamParameters
    @return int Maximum RRHC iterations
   **/
  int CrsmSlam::getMaxHillClimbingIterations()
  {
    return slamParams.max_hill_climbing_iterations;
  }

  /**
    @brief Gets the dx_laser_robotCenter of CRSM_SlamParameters
    @return double Translation in x axis of laser in comparison to robot center
   **/
  double CrsmSlam::getDxLaserRobotCenter()
  {
    return slamParams.dx_laser_robotCenter;
  }

  /**
    @brief Gets the occupancy_grid_map_freq of CRSM_SlamParameters
    @return double The occupancy grid map publishing frequency
   **/
  double CrsmSlam::getOccupancyGridMapFreq()
  {
    return slamParams.occupancy_grid_map_freq;
  }

  /**
    @brief Gets the robot_pose_tf_freq of CRSM_SlamParameters
    @return double The robot pose publishing frequency
   **/
  double CrsmSlam::getRobotPoseTfFreq()
  {
    return slamParams.robot_pose_tf_freq;
  }

  /**
    @brief Gets the trajectory_freq of CRSM_SlamParameters
    @return double The trajectory publishing frequency
   **/
  double CrsmSlam::getTrajectoryFreq()
  {
    return slamParams.trajectory_freq;
  }

  /**
    @brief Gets the desired_number_of_picked_rays of CRSM_SlamParameters
    @return int The desired number of picked rays [algorithm specific]
   **/
  int CrsmSlam::getDesiredNumberOfPickedRays()
  {
    return slamParams.desired_number_of_picked_rays;
  }

  /**
    @brief Gets the robot_width of CRSM_SlamParameters
    @return double The robot width
   **/
  double CrsmSlam::getRobotWidth()
  {
    return slamParams.robot_width;
  }

  /**
    @brief Gets the robot_length of CRSM_SlamParameters
    @return double The robot length
   **/
  double CrsmSlam::getRobotLength()
  {
    return slamParams.robot_length;
  }

  /**
    @brief Gets the occupancy_grid_publish_topic of CRSM_SlamParameters
    @return std::string The occupancy grid publishing topic
   **/
  std::string CrsmSlam::getOccupancyGridPublishTopic()
  {
    return slamParams.occupancy_grid_publish_topic;
  }

  /**
    @brief Gets the robot_trajectory_publish_topic of CRSM_SlamParameters
    @return std::string The trajectory publishing topic
   **/
  std::string CrsmSlam::getRobotTrajectoryPublishTopic()
  {
    return slamParams.robot_trajectory_publish_topic;
  }

  /**
    @brief Gets the trajectory_publisher_frame_id of CRSM_SlamParameters
    @return std::string The trajectory frame ID
   **/
  std::string CrsmSlam::getTrajectoryPublisherFrameId()
  {
    return slamParams.trajectory_publisher_frame_id;
  }

  /**
    @brief Gets the laser_subscriber_topic of CRSM_SlamParameters
    @return std::string The laser subscriber topic
   **/
  std::string CrsmSlam::getLaserSubscriberTopic()
  {
    return slamParams.laser_subscriber_topic;
  }

  /**
    @brief Gets the base_footprint_frame of CRSM_SlamParameters
    @return std::string Holds the base footprint frame - (x,y,yaw)
   **/
  std::string CrsmSlam::getBaseFootprintFrame()
  {
    return slamParams.base_footprint_frame;
  }

  /**
    @brief Gets the base_frame of CRSM_SlamParameters
    @return std::string Holds the base frame
   **/
  std::string CrsmSlam::getBaseFrame()
  {
    return slamParams.base_frame;
  }

  /**
    @brief Gets the map_frame of CRSM_SlamParameters
    @return std::string Holds the map frame
   **/
  std::string CrsmSlam::getMapFrame()
  {
    return slamParams.map_frame;
  }
  
  //void CrsmSlam::drawInitialPatch()
  //{
    //int initialPatchWidth = slamParams.robot_width / slamParams.ocgd / 2;
    //int initialPatchLength = slamParams.robot_length / slamParams.ocgd / 2;
    //
    //for(int i = -initialPatchLength; i <= initialPatchLength; i++)
    //{
      //for(int j = -initialPatchWidth; j <= initialPatchWidth; j++)
      //{
        //staticMap.p[i + staticMap.info.originx - (int)(slamParams.dx_laser_robotCenter / slamParams.ocgd)][j + staticMap.info.originy] = 0;
      //}
    //}
  //}
}
