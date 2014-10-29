#pragma once

#include <vector>
	
/** 
 * @brief CubicSpline
 *
 * Describes a CubicSpline defined by ControlPoints.
 * Each ControlPoint consist of at least a position, and
 * optionally a speed (tangent) and a time (these two can be set to
 * default values, giving you a [0, 1] catmull-rom spline).
 *
 * @todo Easier point insertion ? Auto sorting if time is specified ?
 * @todo Comment the whole thing
 *
 * @param T Well defined vector class (addition, multiplication whith a scalar R)
 * @param R Scalar type of the vector class (default: float)
**/
template<class T, typename R = float>
class CubicSpline
{
public:	
	/**
	 * @brief ControlPoint
	 *
	 * Describes a point in space (T) where an object following the 
	 * spline should be at a given time (R), at a given speed (T). 
	 * Speed and Time can be omitted, in this case, the CubicSpline
	 * class will use default values for there two properties.
	**/
	class ControlPoint
	{
	public:
		ControlPoint() =default; ///< Default Constructor
		ControlPoint(const ControlPoint&) =default; ///< Copy Constructor
		ControlPoint(ControlPoint&&) =default; ///< Move Constructor
		
		/**
		 * @brief Constructor
		 * 
		 * @param position Position in space
		 * @param speed Speed
		 * @param time Time Spline(time) will return position
		**/
		ControlPoint(T position, T speed = T(), R time = R()) :
			_position(position),
			_speed(speed),
			_time(time)
		{ }
		
		~ControlPoint() =default;
		
		/**
		 * Should not be used outside of CubicSpline
		 * @return true if the point as changed since the last update
		**/
		inline bool isDirty() const { return _dirty; }
		/** 
		 * Should not be used outside of CubicSpline
		 * @param b New value for the dirty flag
		**/
		inline void setDirty(bool b = true) { _dirty = b; }
	
		/// @return Position in space of the point
		inline const T& getPosition() const { return _position; }
		
		/// @return Speed (First derivative) of the spline at this point
		inline const T& getSpeed() const { return _speed; }
		
		/// @return Time such as Spline(Time) = Position of this ControlPoint
		inline const R& getTime() const { return _time; }
		
		/// @param v New Position
		inline void setPosition(const T& v) { _position = v; setDirty(); }
		
		/// @param v New Speed
		inline void setSpeed(const T& v) { _speed = v; setDirty(); }
		
		/// @param v New Time
		inline void setTime(const R& v) { _time = v; setDirty(); }
	private:
		bool	_dirty = true;	///< Marks as 'changed'
		T		_position;			///< Position
		T		_speed;				///< Speed (First derivative)
		R		_time;				///< Time
	};
	
	// Constructors
	
	/** @brief Default Constructor **/
	CubicSpline() =default;
	/** @brief Copy Constructor **/
	CubicSpline(const CubicSpline<T, R>&) =default;
	/** @brief Move Constructor **/
	CubicSpline(CubicSpline<T, R>&&) =default;

	/** 
	 * @brief Constructor by list of positions 
	 *
	 * This will construct a Catmull-Rom spline.
	 * (calling linearTiming() then catmullRom())
	 * @param l List of positions
	 * @see template<class Iterator> CubicSpline(Iterator begin, Iterator end)
	**/
	CubicSpline(const std::initializer_list<T>& l);

	/** @brief Constructor by list of ControlPoints
	 * @param l List of ControlPoints
	**/
	CubicSpline(const std::initializer_list<ControlPoint>& l);

	/** @brief Constructor by list of positions 
	 *
	 * This will construct a Catmull-Rom spline.
	 * (calling linearTiming() then catmullRom())
	 * @param begin Iterator pointing to the first position
	 * @param end Iterator pointing right after the last position
	 * @see CubicSpline(const std::initializer_list<T>& l)
	**/
	template<class Iterator>
	CubicSpline(Iterator begin, Iterator end);
	
	/// @brief Destructor
	~CubicSpline() =default;
	
	/// @brief Assignment operator
	CubicSpline& operator=(const CubicSpline&) =default;
	
	/// @brief Move operator
	CubicSpline& operator=(CubicSpline&&) =default;
	
	/// @return The number of ControlPoint describing the spline
	inline size_t getPointCount() const { return _points.size(); }

	/** 
	 * @brief Adds c at the end of the spline.
	 * @param c ControlPoint to add.
	 * @see add(const ControlPoint&)
	**/
	inline void operator+=(const ControlPoint& c) { add(c); }
	
	/** 
	 * @brief Adds c at the end of the spline.
	 * @param c ControlPoint to add.
	 * @see operator+=(const ControlPoint&)
	**/
	inline void add(const ControlPoint& c);
	
	/**
	 * @return First correct value for get(R t)
	 * @see get(R T)
	 * @see getEndTime()
	**/
	inline R getStartTime() const { return _points[0].getTime(); }
	
	/**
	 * @return Highest correct value for get(R t)
	 * @see get(R T)
	 * @see getStartTime()
	**/
	inline R getEndTime() const { return _points[getPointCount() - 1].getTime(); }
	
	/**
	 * @param t Time
	 * @return Value of the spline at t
	 * @see get(R t)
	**/
	inline T operator()(R t);
	
	/**
	 * @param t Time
	 * @return Value of the spline at t
	 * @see operator()(R t)
	**/
	inline T get(R t);
	
	/**
	 * @param t Time
	 * @return Value of the speed (first derivative) of the spline at t
	**/
	inline T getSpeed(R t);
	
	/**
	 * @param t Time
	 * @return Value of the acceleration (second derivative) of the spline at t
	**/
	inline T getAcceleration(R t);
	
	/**
	 * @brief Modify the ControlPoint's times.
	 * If the spline is described by 3 ControlPoints C1, C2, C3 :
	 * C1 will be reached for t = 0.0 * m / 2.0 = 0.0
	 * C2 will be reached for t = 1.0 * m / 2.0 = 0.5
	 * C3 will be reached for t = 2.0 * m / 2.0 = 1.0
	 * @param m Maximum time (i.e. getEndTime() will return m)
	**/
	inline void linearTiming(R m = static_cast<R>(1.0));
	
	/**
	 * @brief Catmull-Rom Spline
	 *
	 * Compute ControlPoint's speeds (~tangents/derivatives)
	 * to match a Catmull-Rom spline.
	**/
	inline void catmullRom();
	
	/**
	 * @brief Used to iterate over ControlPoints
	 *
	 * @see end()
	**/
	inline typename std::vector<ControlPoint>::iterator begin() { return _points.begin(); }
	/**
	 * @brief Used to iterate over ControlPoints
	 *
	 * @see begin()
	**/
	inline typename std::vector<ControlPoint>::iterator end() { return _points.end(); }
	
	/** @brief Used to iterate over const ControlPoints
	 *
	 * @see cend()
	**/
	inline typename std::vector<ControlPoint>::const_iterator cbegin() const { return _points.cbegin(); }
	/**
	 * @brief Used to iterate over const ControlPoints
	 *
	 * @see cbegin()
	**/
	inline typename std::vector<ControlPoint>::const_iterator cend() const { return _points.cend(); }
	
private:
	/**
	 * @brief Polynomial
	**/
	class Polynomial : public std::array<T, 4>
	{
	public:
		/// @return Evaluation of the polynomial at t
		inline T operator()(R t) const { return (*this)[0] + t * ((*this)[1] + t * ((*this)[2] + t * (*this)[3])); }
		/// @return Evaluation of the first derivative of the polynomial at t
		inline T d(R t) const { return (*this)[1] + t * (2.0 * (*this)[2] + t * 3.0 * (*this)[3]); }
		/// @return Evaluation of the second derivative of the polynomial at t
		inline T d2(R t) const { return 2.0 * (*this)[2] + 6.0 * (*this)[3] * t; }
		
		/// @return First derivative of the polynomial (So, also a polynomial)
		inline Polynomial d() const { return {(*this)[1], 2.0 * (*this)[2], 3.0 * (*this)[3], R()}; }
		/// @return Second derivative of the polynomial (So, also a polynomial)
		inline Polynomial d2() const { return {2.0 * (*this)[2], 6.0 * (*this)[3], R(), R()}; }
	};
	
	std::vector<ControlPoint>	_points;			///< ControlPoints
	std::vector<Polynomial>		_polynomials;		///< Polynomials corresponding to each portion of the spline
	
	/**
	 * @param t Time
	 * @return Index of the ControlPoint right before t
	**/
	size_t getPoint(R t);
	
	/**
	 * @brief Makes sure the i-th polynomial is up-to-date.
	 * @see updatePolynomial(size_t i)
	**/
	void checkPolynomial(size_t i);
	
	/**
	 * @brief Compute coefficients for the i-th polynomial.
	**/
	void updatePolynomial(size_t i);
};

// Implementation Details
#include <CubicSpline.tcc>
