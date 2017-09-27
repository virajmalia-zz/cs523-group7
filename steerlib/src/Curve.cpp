//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points
	if(!checkRobust())
		return;
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points

	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve

	float t_cur = controlPoints[0].time + (float)window;

	Point p_prev = controlPoints[0].position;
	Point p_i;

	while (t_cur <= controlPoints[controlPoints.size() - 1].time)
	{
		calculatePoint(p_i, t_cur);
		DrawLib::drawLine(p_prev, p_i, curveColor, curveThickness);
		p_prev = p_i;
		t_cur = t_cur + window;
	}

	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	// Sort controlPoints{position(x,y,z), tanget(x,y,z), time}

	//Might need to add comparing function - Jack
	std::sort(controlPoints.begin(), controlPoints.end());
	return;
}


// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	return controlPoints.size() < 2 ? false : true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{	
	//if( time > controlPoints[0].time && time < controlPoints[1].time){								//time between cp[0] --- cp[1], nextPoint=1
	//	nextPoint = 1;
	//}
	//else if(time > controlPoints[1].time && time < controlPoints[2].time){					//time between cp[1] --- cp[2]
	//	nextPoint = 2;
	//}
	//else if(time > controlPoints[2].time && time < controlPoints[3].time){					//time between cp[2] --- cp[3]
	//	nextPoint = 3;
	//}
	//else{
	//	return false;
	//}

	//We might need a general function that handles multiple points (more than 3). -Jack
	//Below is the binary search

	for (int i = 0; i < controlPoints.size(); i++)
	{
		if (controlPoints[i].time >= time)
		{
			nextPoint = i;
			return true;
		}
	}
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	const unsigned int curPoint = nextPoint - 1;
	intervalTime = controlPoints[nextPoint].time - controlPoints[curPoint].time;
	normalTime = (time - controlPoints[curPoint].time) / intervalTime;

	const float t = normalTime;
	const float t2 = t*t;
	const float t3 = t*t*t;

	//Formula: p = (2t3 - 3t2 + 1)p0 + (t3 - 2t2 +t)m0 + (-2t3 + 3t2)p1 + (t3 - t2)m1

	newPosition = (2 * t3 - 3 * t2 + 1) * controlPoints[curPoint].position +
		(t3 - 2 * t2 + t) * intervalTime * controlPoints[curPoint].tangent +
		(-2 * t3 + 3 * t2) * controlPoints[nextPoint].position +
		(t3 - t2) * intervalTime * controlPoints[nextPoint].tangent;


	//================DELETE THIS PART AND THEN START CODING===================
	/*static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useHermiteCurve is not implemented!" << std::endl;
		flag = true;
	}*/
	//=========================================================================

	// Calculate position at t = time on Hermite curve

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	/*
	Should use parameter nextPoint and handle begin, end points - Jack
	0 = nextPoint - 2;
	1 = nextPoint - 1;
	2 = nextPoint;
	3 = nextPoint + 1;
	*/

	Point newPosition;
	//Point p0 = controlPoints[0].position,
	//			p1 = controlPoints[1].position,
	//			p2 = controlPoints[2].position,
	//			p3 = controlPoints[3].position;

	//// Calculate position at t = time on Catmull-Rom curve
	//newPosition = 0.5 * (
	//	2 * p1 +
	//	(p2 - p0)*time +
	//	(2 * p0 - 5 * p1 + 4 * p2 - p3)*pow(time, 2) +
	//	(p3 - 3 * p2 + 3 * p1 - p0)*pow(time, 3)
	//	);

	// Return result
	return newPosition;
}
