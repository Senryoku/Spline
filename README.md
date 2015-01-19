# Spline
Simple spline class(es).

## CubicSpline
Describes a CubicSpline defined by ControlPoints. (See CubicSpline.hpp for details)
Each ControlPoint consist of at least a position, and optionally a speed (tangent) and a time (these two can be set to default values, giving you a [0, 1] Catmull-Rom spline). The spline will reach each of its ConstrolPoint's positions at the given time and speed.
