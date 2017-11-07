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

 Author : Efthimis Tsiogas, efthimis91@gmail.com
 Organization : AUTH
*/

#ifndef CRSM_DYNAMICOBJECT
#define CRSM_DYNAMICOBJECT

#include <ros/ros.h>

#include <crsm_slam/crsm_point.h>
#include <queue>

namespace crsm_slam
{
	/**
	  @class CrsmDynamicObject
	  @brief Any object that moves during CrsmSlam
	 **/
  class CrsmDynamicObject
  {
		int id; //!< Unique id for the ROS-marker of dynamic object
		int timesStill; //!< Amount of continuous repetitions that dynamic object remains still
		double dt; //!< Current execution time of CRSM SLAM program
		double dxSum,	dySum; //!< Sums of x and y changes of dynamic object's centroid [pixels]
		double xVelocity, yVelocity; //!< Velocities [pixels / second] in x-direction and y-direction of dynamic object
		double yawAngle; //!< Yaw angle of dynamic object [radians]
		double r, g, b; //!< Red, green and blue components for dynamic object's marker colorization
		
		CrsmPoint centroid; //!< Current centroid of dynamic object
		CrsmPoint lastCentroid; //!< Last scanned centroid assigned to dynamic object
		std::queue<double> dx, dy; //!< Changes in x and y [pixels] of dynamic object's centroid
		
		public:
			
			/**
				@brief Default constructor
			**/
			CrsmDynamicObject() {}
			
			/**
				@brief Initialises a CrsmDynamicObject
				@param first [const CrsmPoint&] Current centroid of dynamic object as CrsmPoint
				@param _id [const int] Unique id of dynamic object as int
			**/
			CrsmDynamicObject(CrsmPoint const &first, int const _id);
			
			/**
				@brief Predict current centroid of dynamic object
				@param executionTime [const double] Execution time of CRSM SLAM program [seconds]
				@return void
			**/
			void predict(double const executionTime);
			
			/**
				@brief Update centroid, velocity and yaw angle of dynamic object
				@param newCentroid [const CrsmPoint&] Scanned centroid to be assigned to dynamic object
				@return void
			**/
			void update(CrsmPoint const &newCentroid);
			
			bool still() const;
			int getID() const;
			double timeUnscanned() const;
			double getYawAngle() const;
			double getR() const;
			double getG() const;
			double getB() const;
			CrsmPoint const& getCentroid() const;
  };
}

#endif
