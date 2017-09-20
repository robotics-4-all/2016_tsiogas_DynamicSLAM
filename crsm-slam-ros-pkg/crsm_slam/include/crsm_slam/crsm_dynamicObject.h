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
#include <numeric>
#include <queue>

namespace crsm_slam
{
	/**
	  @class CrsmDynamicObject
	  @brief Any object that moves during CrsmSlam
	 **/
  class CrsmDynamicObject
  {
		int id, timesStill;
		double dt, dxSum, dySum, xVelocity, yVelocity, yawAngle, r, g, b;
		
		CrsmPoint centroid, lastCentroid;
		std::queue<double> dx, dy;
		
		public:
		
			CrsmDynamicObject() {}
			CrsmDynamicObject(CrsmPoint const &first, int const _id);
			void predict(double const executionTime);
			void update(int const Steps, CrsmPoint const &newCentroid);
			int getID() const;
			bool still() const;
			double getR() const;
			double getG() const;
			double getB() const;
			double getYawAngle() const;
			double timeUnscanned() const;
			CrsmPoint const& getCentroid() const;
  };
}

#endif
