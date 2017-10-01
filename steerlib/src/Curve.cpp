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
	// Sort controlPoints{Point position(x,y,z), Vector tanget(x,y,z), float time}

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

	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{


	int prevAux = (nextPoint - 2) < 0 ? controlPoints.size() - (nextPoint - 2) : nextPoint - 2;
	int	prev = (nextPoint - 1) < 0 ? controlPoints.size() - 1 : nextPoint - 1;
	int	curr = nextPoint;
	int	nextAux = (nextPoint + 1 % controlPoints.size()) > 0 ? nextPoint + 1 : 0;
	
	float normalTime, intervalTime;

	intervalTime = controlPoints[curr].time - controlPoints[prev].time;
	normalTime = (time - controlPoints[prev].time) / intervalTime;

	const float t = normalTime;

	Point newPosition;
	Point p0 = controlPoints[prevAux].position,
				p1 = controlPoints[prev].position,
				p2 = controlPoints[curr].position,
				p3 = controlPoints[nextAux].position;

	Vector t0 = controlPoints[prevAux].tangent,
		t1 = controlPoints[prev].tangent,
		t2 = controlPoints[curr].tangent,
		t3 = controlPoints[nextAux].tangent;

	// Calculate position at t = time on Catmull-Rom curve
	
	newPosition = 0.7 * (
		2 * p1 +
		(p2 - p0) * t +
		((2 * p0 - 5 * p1) + (4 * p2 - p3)) * pow(t, 2) +
		((p3 - 3 * p2) + (3 * p1 - p0)) * pow(t, 3)
		);
		
	/*
	newPosition = 0.5 * (
		(2.0f * t * t * t - 3.0f * t * t + 1.0f) * p1 + 
		(t * t * t - 2.0f * t * t + t) * t1 +
		(-2.0f * t * t * t + 3.0f * t * t) * p2	+
		(t * t * t - t * t) * t2 );
		*/
	// Method 2
	
	//newPosition = p1 + t1*t + 3 * (p2 - p1)*t*t + (t1 + t2 - 2 * (p2 - p1))*t*t*t;
	
	// Return result
	return newPosition;
}
