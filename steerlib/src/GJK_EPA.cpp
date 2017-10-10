#include "obstacles/GJK_EPA.h"

Util::Vector ORIGIN = Util::Vector(0.0f, 0.0f, 0.0f);
static std::vector<Util::Vector> _simplex;

SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	bool isIntersect = false;
	std::vector<Util::Vector> simplex;
	isIntersect = GJK(_shapeA, _shapeB);

	if (isIntersect)
	{
		EPA(_shapeA, _shapeB, _simplex);
		_simplex.clear();
	}
	else
	{
		_simplex.clear();
		return false;
	}

	return isIntersect; // There is no collision
}

bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	Util::Vector d = Util::Vector(1.0, 0.0, 0.0);

	_simplex.push_back(Support(_shapeA, _shapeB, d));
	d *= -1.0;

	while (true)
	{
		_simplex.push_back(Support(_shapeA, _shapeB, d));

		if (Util::dot(_simplex[_simplex.size() - 1], d) <= 0) 
		{
			// if the point added last was not past the origin in the direction of d
			// then the Minkowski Sum cannot possibly contain the origin since
			// the last point added is on the edge of the Minkowski Difference
			return false;
		}
		else if (checkOrigin(d)) 
		{
			// if it does then we know there is a collision
			return true;
		}
	}

	return false;
}

Util::Vector SteerLib::GJK_EPA::Support(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, Util::Vector d)
{
	// Get shape and direction, return farest point.
	Util::Vector pA, pB, support;

	pA = FarthestPoint(shapeA, d);
	pB = FarthestPoint(shapeB, -d);
	support = pA - pB;
	return support;
}


Util::Vector SteerLib::GJK_EPA::FarthestPoint(const std::vector<Util::Vector>& s, const Util::Vector d)
{
	Util::Vector curr = s[0];
	Util::Vector result = curr;

	// set the farthest point to the first one

	double bestProj = Util::dot(curr, d);
	double proj;

	// search for the farthest point
	for (int i = 1; i < s.size(); i++)
	{
		curr = s[i];
		proj = Util::dot(curr, d);

		if (proj > bestProj)
		{
			bestProj = proj;
			result = curr;
		}
	}

	return result;
}


bool SteerLib::GJK_EPA::checkOrigin(Util::Vector& d)
{
	Util::Vector pA, pB, pC;
	pA = _simplex[_simplex.size() - 1];

	Util::Vector A0 = ORIGIN - pA;

	// Simplex is a triangle
	if (_simplex.size() == 3)
	{
		pB = _simplex[1];
		pC = _simplex[0];

		Util::Vector AB = pB - pA;
		Util::Vector AC = pC - pA;

		// Simplex check on Point C
		d = Util::Vector(-AB.z, 0, AB.x);
		double dis = Util::dot(d, pC);

		dis > 0 ? d = -d : d = d;

		dis = Util::dot(d, A0);
		if (dis > 0)
		{
			_simplex.erase(_simplex.begin());
			return false;
		}

		// Simplex check on Point B
		d = Util::Vector(-AC.z, 0, AC.x);
		dis = Util::dot(d, pB);

		dis > 0 ? d = -d : d = d;

		dis = Util::dot(d, A0);
		if (dis > 0)
		{
			_simplex.erase(_simplex.begin()+1);
			return false;
		}
		
		// Passed Simplex Check
		return true;
	}
	else
	{
		Util::Vector pB = _simplex[0];
		Util::Vector AB = pB - pA;

		d = Util::Vector(-AB.z, 0, AB.x);
		double dis = Util::dot(d, A0);

		dis < 0 ? d = -d : d = d;
	}

	return false;
}