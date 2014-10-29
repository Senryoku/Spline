#pragma once

// Implementation of CubicSpline.hpp

/** Constructor by list of positions **/
template<class T, typename R>
CubicSpline<T, R>::CubicSpline(const std::initializer_list<T>& L) :
	CubicSpline(L.begin(), L.end())
{
}

/** Constructor by list of ControlPoints **/
template<class T, typename R>
CubicSpline<T, R>::CubicSpline(const std::initializer_list<ControlPoint>& L) :
	CubicSpline(L.begin(), L.end())
{
}

/** Constructor by list of positions **/
template<class T, typename R>
template<class Iterator>
CubicSpline<T, R>::CubicSpline(Iterator Begin, Iterator End) : 
	_points(Begin, End)
{
	linearTiming();
	catmullRom();
	_polynomials.resize(getPointCount());
}

template<class T, typename R>
inline void CubicSpline<T, R>::add(const ControlPoint& C)
{
	_points.push_back(C);
	_polynomials.resize(getPointCount());
}

template<class T, typename R>
inline T CubicSpline<T, R>::operator()(R t)
{
	return get(t);
}

template<class T, typename R>
inline T CubicSpline<T, R>::get(R t)
{
	assert(t >= getStartTime() && t < getEndTime());

	size_t idx = getPoint(t);
	checkPolynomial(idx);
	
	return _polynomials[idx](t);
}

template<class T, typename R>
inline T CubicSpline<T, R>::getSpeed(R t)
{
	assert(t >= getStartTime() && t < getEndTime());
	
	size_t idx = getPoint(t);
	checkPolynomial(idx);
	
	return _polynomials[idx].d(t);
}

template<class T, typename R>
inline T CubicSpline<T, R>::getAcceleration(R t)
{
	assert(t >= getStartTime() && t < getEndTime());

	size_t idx = getPoint(t);
	checkPolynomial(idx);
	
	return _polynomials[idx].d2(t);
}

template<class T, typename R>
inline void CubicSpline<T, R>::linearTiming(R t)
{
	size_t i = 0;
	for(auto& P : _points)
	{
		P.setTime(i * t  / static_cast<R>(getPointCount() - 1));
		++i;
	}
}

// Private

template<class T, typename R>
inline size_t  CubicSpline<T, R>::getPoint(R t)
{
	size_t i = 1;
	while(t > _points[i].getTime()) ++i;
	return i - 1;
}
	
template<class T, typename R>
inline void CubicSpline<T, R>::catmullRom()
{	
	for(size_t i = 1; i < getPointCount() - 1; ++i)
		_points[i].setSpeed((_points[i - 1].getPosition() - _points[i + 1].getPosition()) / (_points[i - 1].getTime() - _points[i + 1].getTime()));
	
	// First and Last point special cases
	// Acceleration will be null at these points.
	{ // First
		const T& A = _points[0].getPosition();
		const T& B = _points[1].getPosition();
		const T& Y = _points[1].getSpeed();
		const R& U = _points[0].getTime();
		const R& V = _points[1].getTime();
		_points[0].setSpeed(static_cast<R>(-0.5f)*(U*Y - V*Y - static_cast<R>(3.0)*A + static_cast<R>(3.0)*B)/(U - V));
	}
	{ // Last (Note that A and B are "inverted" here, since U is the point in time where the acceleration is null)
		const T& A = _points[getPointCount() - 1].getPosition();
		const T& B = _points[getPointCount() - 2].getPosition();
		const T& Y = _points[getPointCount() - 2].getSpeed();
		const R& U = _points[getPointCount() - 1].getTime();
		const R& V = _points[getPointCount() - 2].getTime();
		_points[getPointCount() - 1].setSpeed(static_cast<R>(-0.5)*(U*Y - V*Y - static_cast<R>(3.0)*A + static_cast<R>(3.0)*B)/(U - V));
	}
}

template<class T, typename R>
void CubicSpline<T, R>::checkPolynomial(size_t i)
{
	assert(i < getPointCount() - 1);
	
	if(_points[i].isDirty() || _points[i + 1].isDirty());
	{
		updatePolynomial(i);
		// This is necessary to be sure that every polynomial
		// is up-to-date before setting this to non-dirty.
		if(i > 0 && _points[i].isDirty())
			checkPolynomial(i - 1);
		_points[i].setDirty(false);
	}
}

template<class T, typename R>
void CubicSpline<T, R>::updatePolynomial(size_t i)
{
	assert(i < getPointCount() - 1);

	const ControlPoint& P1 = _points[i];
	const ControlPoint& P2 = _points[i + 1];
	
	const T& A = P1.getPosition(); 
	const T& B = P2.getPosition();		
	const T& X = P1.getSpeed();		
	const T& Y = P2.getSpeed();
	const R& u = P1.getTime();
	const R u2 = u * u;
	const R u3 = u * u2;
	const R& v = P2.getTime();
	const R v2 = v * v;
	const R v3 = v * v2;
	
	const R c2 = static_cast<R>(2.0);
	const R c3 = static_cast<R>(3.0);
	const R c6 = static_cast<R>(6.0);
	
	// Trust me, I'm (almost) an engineer.
	_polynomials[i][0] = -((v*Y - B)*u3 + A*v3 + ((X - Y)*v2 + c3*B*v)*u2 - (v3*X + c3*A*v2)*u)/(u3 - c3*u2*v + c3*u*v2 - v3);
	_polynomials[i][1] = ((c2*X + Y)*u2*v + u3*Y - v3*X - ((X + c2*Y)*v2 + c6*A*v - c6*B*v)*u)/(u3 - c3*u2*v + c3*u*v2 - v3);
	_polynomials[i][2] = -((X + c2*Y)*u2 - (c2*X + Y)*v2 + ((X - Y)*v - c3*A+ c3*B)*u - c3*A*v + c3*B*v)/(u3 - c3*u2*v + c3*u*v2 - v3); 
	_polynomials[i][3] = ((X +Y)*u - (X + Y)*v - c2*A + c2*B)/(u3 - c3*u2*v + c3*u*v2 - v3);
}
