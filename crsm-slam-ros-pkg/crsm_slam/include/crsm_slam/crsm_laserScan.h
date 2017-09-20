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

#ifndef CRSM_LASERSCAN_HEADER
#define CRSM_LASERSCAN_HEADER

#include <crsm_slam/crsm_point.h>

namespace crsm_slam
{
  /**
    @struct CrsmLaserScan
    @brief The container of a laser range finder scan
   **/
  struct CrsmLaserScan
  {
    double *distance; //!< Array with ray distances [meters]
    double *density; //!< Density of scans
    CrsmPoint *p; //!< Array with ray points [pixels]
		
		/**
			@brief Default constructor
		 **/
		CrsmLaserScan() {}
		
    /**
      @brief Constructor. Allocates the neccessary structures for the scan to be stored in
     **/
    CrsmLaserScan(int nRays)
    {
      distance = new double[nRays];
      density = new double[nRays - 1];
      p = new CrsmPoint[nRays];
    }
  };
}

#endif
