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

 Author : Efthimis Tsiogas, efthimis91@gmail.com
 Organization : AUTH
*/

#include <crsm_slam/crsm_dynamicObject.h>

namespace crsm_slam
{
	CrsmDynamicObject::CrsmDynamicObject(CrsmPoint const &first, int const _id)
	{
		centroid = lastCentroid = first;
		id = _id;
		
		timesStill = 3;
		dt = dxSum = dySum = xVelocity = yVelocity = yawAngle = 0;
		
		r = (double) rand() / RAND_MAX;
		g = (double) rand() / RAND_MAX;
		b = (double) rand() / RAND_MAX;
	}
	
	void CrsmDynamicObject::predict(double const currentExecutionTime)
	{
		dt += currentExecutionTime;
		
		centroid.x = lastCentroid.x + xVelocity * dt;
		centroid.y = lastCentroid.y + yVelocity * dt;
	}
	
	void CrsmDynamicObject::update(CrsmPoint const &newCentroid)
	{
		if(dx.size() == 10)
		{
			dxSum -= dx.front();
			dySum -= dy.front();
			
			dx.pop();
			dy.pop();
		}
		
		dx.push( (newCentroid.x - lastCentroid.x) / dt );
		dy.push( (newCentroid.y - lastCentroid.y) / dt );
		
		dxSum += dx.back();
		dySum += dy.back();
		
		xVelocity = dxSum / dx.size();
		yVelocity = dySum / dy.size();
		yawAngle = atan2(yVelocity, xVelocity);
		
		timesStill = (lastCentroid == newCentroid) ? timesStill + 1 : 0;
		
		centroid = lastCentroid = newCentroid;
		
		dt = 0;
	}
	
	bool CrsmDynamicObject::still() const
	{
		return (timesStill >= 3);
	}
	
	int CrsmDynamicObject::getID() const
	{
		return id;
	}
	
	double CrsmDynamicObject::timeUnscanned() const
	{
		return dt;
	}
	
	double CrsmDynamicObject::getYawAngle() const
	{
		return yawAngle;
	}
	
	double CrsmDynamicObject::getR() const
	{
		return r;
	}
	
	double CrsmDynamicObject::getG() const
	{
		return g;
	}
	
	double CrsmDynamicObject::getB() const
	{
		return b;
	}
	
	const CrsmPoint& CrsmDynamicObject::getCentroid() const
	{
		return centroid;
	}
}
