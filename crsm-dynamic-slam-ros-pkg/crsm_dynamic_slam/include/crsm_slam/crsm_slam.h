/*
 This file is part of CrsmSlam.
 CrsmSlam is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 CrsmSlam is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with CrsmSlam.  If not, see <http://www.gnu.org/licenses/>.
 
 Author : Manos Tsardoulias, etsardou@gmail.com
 Organization : AUTH, PANDORA Robotics Team
 */

#ifndef CRSM_SLAM_HEADER
#define CRSM_SLAM_HEADER

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <crsm_slam/crsm_laser.h>
#include <crsm_slam/crsm_hillClimbing.h>
#include <crsm_slam/crsm_pose.h>
#include <crsm_slam/crsm_map.h>
#include <crsm_slam/crsm_slamParameters.h>
#include <crsm_slam/crsm_dynamicObject.h>

namespace crsm_slam
{
  /**
    @class CrsmSlam
    @brief The main slam class. Contains the main functionalities of CRSM slam.
   **/
  class CrsmSlam
  {
    private:
			
      ros::Subscriber clientLaserValues; //!< The laser subscriber
      ros::Publisher _occupancyGridPublisher; //!< The occupancy grid map publisher
      ros::Publisher _markersPublisher; //!< The markers publisher
      ros::Publisher _pathPublisher; //!< The robot trajectory publisher
      ros::Publisher _posePublisher; //!< The robot pose publisher
      tf::TransformBroadcaster _slamFrameBroadcaster; //!< The tf robot pose broadcaster
      tf::TransformListener _listener; //!< Tf listener to acquire the transformation between the laser and the robot center
      
      CrsmExpansion expansion;
      CrsmMap staticMap; //!< The OGM container (static part)
      CrsmMap dynamicMap; //!< The OGM container (dynamic part)
      CrsmLaser laser; //!< The laser container
      
      int argc; //!< Number of input arguments
      char **argv; //!< The input arguments
      
      double bestFitness; //!< The best RRHC fitness for a specific iteration
      double meanDensity; //!< The mean laser scan density for a specific iteration
      
      CrsmTransformation bestTransformation; //!< The best RRHC transformation for a specific iteration
      CrsmPose robotPose; //!< The robot pose
      CrsmSlamParameters slamParams; //!< The slam parameters
      
      std::vector<CrsmPose> robotTrajectory; //!< Container for the robot trajectory
      std::set<int> scanSelections; //!< Holds the critical rays, on which the scan matching is performed
      std::set<int> bigChanges; //!< Holds the irregularities of a specific scan in terms of distance
      
      /**
        @brief Reads the CRSM slam parameters from the yaml file and fills the CrsmSlamParameters structure
        @return void
       **/
      void updateParameters();
      
      /**
       * @brief Draws a patch underneath the robot's footprint
       * @return void
       **/
      //void drawInitialPatch();
      
      void expandMap();
      
      bool checkExpansion(double x, double y, bool update);
      
      nav_msgs::Path trajectory;
      
      int k; //!< Number of clusters for use in findCentroids function
      int id; //!< Ids assigned to dynamic objects
      double const ObjectArea; //!< Area [meters] that defines the minimum space occupied by a dynamic object
      
      std::vector<CrsmDynamicObject> dynamicObject; //!< Vector that contains the dynamic objects scanned by the laser
      std::vector<int> deleted;
      
    public:
    
      ros::NodeHandle n; //!< The ROS node handle
      
      /**
        @brief Default costructor
        @param argc [int] The number of input arguments
        @param argv [char **] The input arguments
        @return void
       **/
      CrsmSlam(int argc, char **argv);
      
      /**
        @brief Destructor
        @return void
       **/
      ~CrsmSlam() {}
      
      /**
        @brief Serves the laser scan messages
        @param msg [sensor_msgs::LaserScanConstPtr&] : The laser rays distances
        @return void
       **/
      void fixNewScans(sensor_msgs::LaserScanConstPtr const &msg);
      
      /**
        @brief Chooses important rays for RRHC
        @return void
       **/
      void calculateCriticalRays();
      
      /**
        @brief Calculates the transformation (translation & rotation) with RRHC
        @return void
       **/
      void findTransformation();
      
      /**
        @brief Updates map after finding the new robot pose
        @return void
       **/
      void updateMapProbabilities();
      
     	/**
				@author Efthimis Tsiogas, efthimis91@gmail.com
				@brief Draws a bit of free space in static map around a dynamic point's position
				@param Point [const CrsmPoint&] Central point
				@return void
			 **/
      void drawFreeSpace(CrsmPoint const &Point);
      
     	/**
				@author Efthimis Tsiogas, efthimis91@gmail.com
				@brief Returns true if a map point is too close to a potentially static point
				@param Point [const CrsmPoint&] Point to be checked
				@return bool
			 **/
      bool closeToStatic(CrsmPoint const &Point);
      
			/**
				@author Efthimis Tsiogas, efthimis91@gmail.com
				@brief Calculates the Euclidean distance between two map points [pixels]
				@param Point1 [const CrsmPoint&] First map point
				@param Point2 [const CrsmPoint&] Second map point
				@return The distance between the two CrsmPoints [pixels ^ 2]
			 **/
			double Distance(CrsmPoint const &Point1, CrsmPoint const &Point2);
			
      /**
				@author Efthimis Tsiogas, efthimis91@gmail.com
        @brief Finds dynamic objects by applying k-means clustering to the dynamic points
        @param dynamicPoint [const std::vector<CrsmPoint>&] Dynamic points scanned by the laser
        @return Centroids of the resulting clusters
       **/
      std::vector<CrsmPoint> findCentroids(std::vector<CrsmPoint> const &dynamicPoint);
      
      /**
				@author Efthimis Tsiogas, efthimis91@gmail.com
        @brief Checks if any two centroids are too close to each other and returns true if that's the case
        @param currentCentroids [const std::vector<CrsmPoint>&] Centroids to check
        @return bool
       **/
      bool centroidsTooClose(std::vector<CrsmPoint> const &currentCentroids);
			
		  /**
				@author Efthimis Tsiogas, efthimis91@gmail.com
        @brief Checks whether a point's farest distance from the respective cluster's centroid is greater than ObjectArea
        @param cluster [const std::vector<CrsmPoint>* const] Clusters formed by k-means clustering
        @param currentCentroids [const std::vector<CrsmPoint>&] Centroids of clusters
        @return true if any such distance is greater than ObjectArea, false otherwise
       **/
			bool pointTooFar(std::vector<CrsmPoint> const &currentCentroids, std::vector<CrsmPoint> const *const &cluster);
			
			/**
				@author Efthimis Tsiogas, efthimis91@gmail.com
        @brief Matches the centroids of existing dynamic objects to the centroids scanned by the laser
        @param centroidsScanned [std::vector<CrsmPoint>*] Centroids found by k-means' clusters
        @param currentExecutionTime [const double] Execution time of CRSM SLAM
        @return void
       **/
      void updateObjects(std::vector<CrsmPoint> *centroidsScanned, double const currentExecutionTime);
      
      /**
				@author Efthimis Tsiogas, efthimis91@gmail.com
        @brief Starts the markers publisher, posting to markers_publish_topic from parameters
        @return void
       **/
			void startMarkersPublisher();
			
      /**
				@author Efthimis Tsiogas, efthimis91@gmail.com
        @brief Publishes the markers as visualization_msgs::MarkerArray
        @return void
       **/
			void publishMarkers();
      
      /**
        @brief Starts the laser subscriber, listening to laser_subscriber_topic from parameters
        @return void
       **/
      void startLaserSubscriber();
      
      /**
        @brief Stops the laser subscriber
        @return void
       **/
      void stopLaserSubscriber();
      
      /**
        @brief Starts the OccupancyGrid publisher, posting to occupancy_grid_publish_topic from parameters
        @return void
       **/
      void startOGMPublisher();
      
      /**
        @brief Stops the OccupancyGrid publisher
        @return void
       **/
      void stopOGMPublisher();
      
      /**
        @brief Publishes the OccupancyGrid map as nav_msgs::OccupancyGrid, posting with occupancy_grid_map_freq Hz from parameters
        @param e [const ros::TimerEvent&] The timer event
        @return void
       **/
      void publishOGM(ros::Time timestamp);
      
      /**
        @brief Publishes the Tf robot pose, posting with robot_pose_tf_freq Hz from parameters
        @param e [const ros::TimerEvent&] The timer event
        @return void
       **/
      void publishRobotPoseTf(ros::Time timestamp);
      
      /**
        @brief Starts the Trajectory publisher, posting to robot_trajectory_publish_topic from parameters, with trajectory_publisher_frame_id as frame ID.
        @return void
       **/
      void startTrajectoryPublisher();
      
      /**
        @brief Stops the Trajectory publisher.
        @return void
       **/
      void stopTrajectoryPublisher();
      
      /**
        @brief Publishes the robot trajectory as nav_msgs::Path, posting with trajectory_freq Hz from parameters
        @param e [const ros::TimerEvent&] The timer event
        @return void
       **/
      void publishTrajectory(ros::Time timestamp);
      
      /**
        @brief Returns the map info in a CrsmMapInfo structure
        @return CrsmMapInfo
       **/
      CrsmMapInfo getMapInfo();
      
      /**
        @brief Returns the map occupancy probability of coordinates (x,y) ranging from 0-255 (0 is occupied, 255 is free)
        @param x [int] : The x coordinate
        @param y [int] : The y coordinate
        @return char probability
       **/
      char getMapProbability(int x, int y);
      
      /**
        @brief Returns the robot pose in a CrsmPose structure
        @return CrsmPose
       **/
      CrsmPose getRobotPose();
      
      /**
        @brief Returns the laser info in a CrsmLaserInfo structure
        @return CrsmLaserInfo
       **/
      CrsmLaserInfo getLaserInfo();
      
      /**
        @brief Returns the robot trajectory in a vector of CrsmPose structures
        @return std::vector<CrsmPose>
       **/
      std::vector<CrsmPose> getTrajectory();
      
      //---------------------- Setters for slamParameters --------------------//
      
      /**
        @brief Sets the disparity of CRSM_SlamParameters
        @param disparity [int] Disparity of mutation in pixels at hill climbing
        @return void
       **/
      void setDisparity(int disparity);

      /**
        @brief Sets the map_size of CRSM_SlamParameters
        @param size [int] Map size of initial allocated map
        @return void
       **/
      void setInitialMapSize(int size);

      /**
        @brief Sets the ocgd of CRSM_SlamParameters
        @param ocgd [double] [OC]cupancy [G]rid [D]imentionality - the width and height in meters of a pixel
        @return void
       **/
      void setOcgd(double ocgd);

      /**
        @brief Sets the density of CRSM_SlamParameters
        @param density [double] Map update density (0-127)
        @return void
       **/
      void setDensity(double density);

      /**
        @brief Sets the obstacle_density of CRSM_SlamParameters
        @param ob_density [double] Coefficient for obstacle update density (0+)
        @return void
       **/
      void setObstacleDensity(double ob_density);

      /**
        @brief Sets the scan_selection_meters of CRSM_SlamParameters
        @param scan_selection_meters [double] Scan density lower boundary for a scan-part identification
        @return void
       **/
      void setScanSelectionMeters(double scan_selection_meters);

      /**
        @brief Sets the max_hill_climbing_iterations of CRSM_SlamParameters
        @param iterations [int] Maximum RRHC iterations
        @return void
       **/
      void setMaxHillClimbingIterations(int iterations);

      /**
        @brief Sets the dx_laser_robotCenter of CRSM_SlamParameters
        @param dx [double] Translation in x axis of laser in comparison to robot center
        @return void
       **/
      void setDxLaserRobotCenter(double dx);

      /**
        @brief Sets the occupancy_grid_map_freq of CRSM_SlamParameters
        @param freq [double] The occupancy grid map publishing frequency
        @return void
       **/
      void setOccupancyGridMapFreq(double freq);

      /**
        @brief Sets the robot_pose_tf_freq of CRSM_SlamParameters
        @param freq [double] The robot pose publishing frequency
        @return void
       **/
      void setRobotPoseTfFreq(double freq);

      /**
        @brief Sets the trajectory_freq of CRSM_SlamParameters
        @param freq [double] The trajectory publishing frequency
        @return void
       **/
      void setTrajectoryFreq(double freq);

      /**
        @brief Sets the desired_number_of_picked_rays of CRSM_SlamParameters
        @param rays [int] The desired number of picked rays [algorithm specific]
        @return void
       **/
      void setDesiredNumberOfPickedRays(int rays);

      /**
        @brief Sets the robot_width of CRSM_SlamParameters
        @param width [double] The robot width
        @return void
       **/
      void setRobotWidth(double width);

      /**
        @brief Sets the robot_length of CRSM_SlamParameters
        @param length [double] The robot length
        @return void
       **/
      void setRobotLength(double length);

      /**
        @brief Sets the occupancy_grid_publish_topic of CRSM_SlamParameters
        @param topic [std::string] The occupancy grid publishing topic
        @return void
       **/
      void setOccupancyGridPublishTopic(std::string topic);

      /**
        @brief Sets the robot_trajectory_publish_topic of CRSM_SlamParameters
        @param topic [std::string] The trajectory publishing topic
        @return void
       **/
      void setRobotTrajectoryPublishTopic(std::string topic);

      /**
        @brief Sets the trajectory_publisher_frame_id of CRSM_SlamParameters
        @param frame_id [std::string] The trajectory frame ID
        @return void
       **/
      void setTrajectoryPublisherFrameId(std::string frame_id);

      /**
        @brief Sets the laser_subscriber_topic of CRSM_SlamParameters
        @param topic [std::string] The laser subscriber topic
        @return void
       **/
      void setLaserSubscriberTopic(std::string topic);

      /**
        @brief Sets the base_footprint_frame of CRSM_SlamParameters
        @param frame [std::string] Holds the base footprint frame - (x,y,yaw)
        @return void
       **/
      void setBaseFootprintFrame(std::string frame);

      /**
        @brief Sets the base_frame of CRSM_SlamParameters
        @param frame [std::string] Holds the base frame
        @return void
       **/
      void setBaseFrame(std::string frame);

      /**
        @brief Sets the map_frame of CRSM_SlamParameters
        @param frame [std::string] Holds the map frame
        @return void
       **/
      void setMapFrame(std::string frame);

      //------------------- Getters for slamParameters ----------------------//

      /**
        @brief Gets the disparity of CRSM_SlamParameters
        @return int Disparity of mutation in pixels at hill climbing
       **/
      int getDisparity();

      /**
        @brief Gets the map_size of CRSM_SlamParameters
        @return int Map size of initial allocated map
       **/
      int getInitialMapSize();

      /**
        @brief Gets the ocgd of CRSM_SlamParameters
        @return double [OC]cupancy [G]rid [D]imentionality - the width and height in meters of a pixel
       **/
      double getOcgd();

      /**
        @brief Gets the density of CRSM_SlamParameters
        @return double Map update density (0-127)
       **/
      double getDensity();

      /**
        @brief Gets the obstacle_density of CRSM_SlamParameters
        @return double Coefficient for obstacle update density (0+)
       **/
      double getObstacleDensity();

      /**
        @brief Gets the scan_selection_meters of CRSM_SlamParameters
        @return double Scan density lower boundary for a scan-part 
          identification
       **/
      double getScanSelectionMeters();

      /**
        @brief Gets the max_hill_climbing_iterations of CRSM_SlamParameters
        @return int Maximum RRHC iterations
       **/
      int getMaxHillClimbingIterations();

      /**
        @brief Gets the dx_laser_robotCenter of CRSM_SlamParameters
        @return double Translation in x axis of laser in comparison to robot center
       **/
      double getDxLaserRobotCenter();

      /**
        @brief Gets the occupancy_grid_map_freq of CRSM_SlamParameters
        @return double The occupancy grid map publishing frequency
       **/
      double getOccupancyGridMapFreq();

      /**
        @brief Gets the robot_pose_tf_freq of CRSM_SlamParameters
        @return double The robot pose publishing frequency
       **/
      double getRobotPoseTfFreq();

      /**
        @brief Gets the trajectory_freq of CRSM_SlamParameters
        @return double The trajectory publishing frequency
       **/
      double getTrajectoryFreq();

      /**
        @brief Gets the desired_number_of_picked_rays of CRSM_SlamParameters
        @return int The desired number of picked rays [algorithm specific]
       **/
      int getDesiredNumberOfPickedRays();

      /**
        @brief Gets the robot_width of CRSM_SlamParameters
        @return double The robot width
       **/
      double getRobotWidth();

      /**
        @brief Gets the robot_length of CRSM_SlamParameters
        @return double The robot length
       **/
      double getRobotLength();

      /**
        @brief Gets the occupancy_grid_publish_topic of CRSM_SlamParameters
        @return std::string The occupancy grid publishing topic
       **/
      std::string getOccupancyGridPublishTopic();

      /**
        @brief Gets the robot_trajectory_publish_topic of CRSM_SlamParameters
        @return std::string The trajectory publishing topic
       **/
      std::string getRobotTrajectoryPublishTopic();

      /**
        @brief Gets the trajectory_publisher_frame_id of CRSM_SlamParameters
        @return std::string The trajectory frame ID
       **/
      std::string getTrajectoryPublisherFrameId();

      /**
        @brief Gets the laser_subscriber_topic of CRSM_SlamParameters
        @return std::string The laser subscriber topic
       **/
      std::string getLaserSubscriberTopic();

      /**
        @brief Gets the base_footprint_frame of CRSM_SlamParameters
        @return std::string Holds the base footprint frame - (x,y,yaw)
       **/
      std::string getBaseFootprintFrame();

      /**
        @brief Gets the base_frame of CRSM_SlamParameters
        @return std::string Holds the base frame
       **/
      std::string getBaseFrame();

      /**
        @brief Gets the map_frame of CRSM_SlamParameters
        @return std::string Holds the map frame
       **/
      std::string getMapFrame();
  };
}

#endif
