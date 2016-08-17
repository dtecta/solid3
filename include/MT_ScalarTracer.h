/*
 * SOLID - Software Library for Interference Detection
 * 
 * Copyright (C) 2001-2003  Dtecta.  All rights reserved.
 *
 * This library may be distributed under the terms of the Q Public License
 * (QPL) as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE.QPL included in the packaging of this file.
 *
 * This library may be distributed and/or modified under the terms of the
 * GNU General Public License (GPL) version 2 as published by the Free Software
 * Foundation and appearing in the file LICENSE.GPL included in the
 * packaging of this file.
 *
 * This library is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Commercial use or any other use of this library not covered by either 
 * the QPL or the GPL requires an additional license from Dtecta. 
 * Please contact info@dtecta.com for enquiries about the terms of commercial
 * use of this library.
 */

#ifndef MT_SCALAR_TRACER_H
#define MT_SCALAR_TRACER_H

#include <cmath>
#include <iostream>
#include <cassert>

template <typename Scalar> 
class MT_ScalarTracer {
public:    
    MT_ScalarTracer() {}
    explicit MT_ScalarTracer(Scalar value, Scalar error = Scalar(0.0)) 
	  : m_value(value), 
		m_error(value == Scalar(0.0) ? Scalar(0.0) : error) 
	{}

    Scalar getValue() const { return m_value; }
    Scalar getError() const { return m_error; }

	void setError(Scalar error) { m_error = error; }

    operator Scalar() const { return m_value; }

    MT_ScalarTracer operator-() const 
	{
        return MT_ScalarTracer<Scalar>(-m_value, m_error);
    }

    MT_ScalarTracer& operator=(Scalar value) 
	{
        m_value = value;
        m_error = Scalar(0.0);
        return *this;
    }

    MT_ScalarTracer& operator+=(const MT_ScalarTracer& x) 
	{
        *this = *this + x;
        return *this;
    }

    MT_ScalarTracer& operator-=(const MT_ScalarTracer& x) 
	{
        *this = *this - x;
        return *this;
    }

    MT_ScalarTracer& operator*=(const MT_ScalarTracer& x) 
	{
        *this = *this * x;
        return *this;
    }

    MT_ScalarTracer& operator/=(const MT_ScalarTracer& x) 
	{
        *this = *this / x;
        return *this;
    }

private:
    Scalar m_value; // the value of the scalar
    Scalar m_error; // the relative rounding error
};

// `m_error' times the machine epsilon (FLT_EPSILON for floats, DBL_EPSILON for doubles) 
// gives an estimated upper bound for the rounding error in `m_value'.
// The number of dirty bits in the mantissa is `log(m_error) / log(2)', so if `m_error'
// reaches 2^24 = 16777216 then we used up all bits in the mantissa of a float. 
// Note that the error is a rough upper bound. In reality i386 platforms may 
// evaluate compound expressions in higher precision, so the given machine epsilon
// is usually too large in most cases.   


template <class Scalar>
inline MT_ScalarTracer<Scalar> operator+(const MT_ScalarTracer<Scalar>& x, 
										 const MT_ScalarTracer<Scalar>& y) 
{
	Scalar value = x.getValue() + y.getValue();
    return MT_ScalarTracer<Scalar>(
		value, 
		(Scalar_traits<Scalar>::abs(x.getValue()) * x.getError() + 
		 Scalar_traits<Scalar>::abs(y.getValue()) * y.getError()) / Scalar_traits<Scalar>::abs(value) + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> operator-(const MT_ScalarTracer<Scalar>& x, 
										 const MT_ScalarTracer<Scalar>& y) 
{
	Scalar value = x.getValue() - y.getValue();
    return MT_ScalarTracer<Scalar>(
		value, 
		(Scalar_traits<Scalar>::abs(x.getValue()) * x.getError() + 
		 Scalar_traits<Scalar>::abs(y.getValue()) * y.getError()) / Scalar_traits<Scalar>::abs(value) +	Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> operator*(const MT_ScalarTracer<Scalar>& x, 
										 const MT_ScalarTracer<Scalar>& y)
{
    return MT_ScalarTracer<Scalar>(x.getValue() * y.getValue(), 
								   x.getError() + y.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> operator/(const MT_ScalarTracer<Scalar>& x, const MT_ScalarTracer<Scalar>& y) 
{
    return MT_ScalarTracer<Scalar>(x.getValue() / y.getValue(), 
								   x.getError() + y.getError() + Scalar(1.0));
}

template <typename Scalar>
inline std::ostream& operator<<(std::ostream& os, const MT_ScalarTracer<Scalar>& x) 
{
    return os << x.getValue() << '[' << x.getError() << ']';
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> sqrt(const MT_ScalarTracer<Scalar>& x) 
{
    return MT_ScalarTracer<Scalar>(sqrt(x.getValue()), Scalar(0.5) * x.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> abs(const MT_ScalarTracer<Scalar>& x) 
{
    return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::abs(x.getValue()), x.getError());
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> cos(const MT_ScalarTracer<Scalar>& x) 
{
    return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::cos(x.getValue()), x.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> sin(const MT_ScalarTracer<Scalar>& x) 
{
    return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::sin(x.getValue()), x.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> tan(const MT_ScalarTracer<Scalar>& x) 
{
	Scalar value = Scalar_traits<Scalar>::tan(x.getValue());
    return MT_ScalarTracer<Scalar>(value, (Scalar(1.0) + value * value) * x.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> acos(const MT_ScalarTracer<Scalar>& x) 
{
    return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::acos(x.getValue()), x.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> asin(const MT_ScalarTracer<Scalar>& x) 
{
    return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::asin(x.getValue()), x.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> atan(const MT_ScalarTracer<Scalar>& x) 
{
    return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::atan(x.getValue()), x.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> atan2(const MT_ScalarTracer<Scalar>& x, const MT_ScalarTracer<Scalar>& y) 
{
    return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::atan2(x.getValue(), y.getValue()), x.getError() + y.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> exp(const MT_ScalarTracer<Scalar>& x) 
{
	Scalar value = Scalar_traits<Scalar>::exp(x.getValue());
    return MT_ScalarTracer<Scalar>(value, value * x.getError() + Scalar(1.0));
}

template <typename Scalar>
inline MT_ScalarTracer<Scalar> log(const MT_ScalarTracer<Scalar>& x) 
{
    return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::log(x.getValue()), x.getError() / x.getValue() + Scalar(1.0));
}


template <typename Scalar>
inline MT_ScalarTracer<Scalar> pow(const MT_ScalarTracer<Scalar>& x, const MT_ScalarTracer<Scalar>& y) 
{
	assert(x.getValue() >= Scalar(0.0));
	Scalar value = Scalar_traits<Scalar>::pow(x.getValue(), y.getValue());
    return MT_ScalarTracer<Scalar>(value, 
								   Scalar_traits<Scalar>::abs(y.getValue()) * x.getError() + 
								   Scalar_traits<Scalar>::abs(value * Scalar_traits<Scalar>::log(x.getValue())) * y.getError() + Scalar(1.0));
}


template <typename Scalar>
struct Scalar_traits<MT_ScalarTracer<Scalar> > {
	static MT_ScalarTracer<Scalar> TwoTimesPi() { return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::TwoTimesPi()); }
	static MT_ScalarTracer<Scalar> epsilon() { return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::epsilon()); }
	static MT_ScalarTracer<Scalar> max() { return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::max()); }
	
	static MT_ScalarTracer<Scalar> random() { return MT_ScalarTracer<Scalar>(Scalar_traits<Scalar>::random()); }
	static MT_ScalarTracer<Scalar> sqrt(MT_ScalarTracer<Scalar> x) { return ::sqrt(x); } 
	static MT_ScalarTracer<Scalar> abs(MT_ScalarTracer<Scalar> x) { return ::abs(x); } 

	static MT_ScalarTracer<Scalar> cos(MT_ScalarTracer<Scalar> x) { return ::cos(x); } 
	static MT_ScalarTracer<Scalar> sin(MT_ScalarTracer<Scalar> x) { return ::sin(x); } 
	static MT_ScalarTracer<Scalar> tan(MT_ScalarTracer<Scalar> x) { return ::tan(x); } 

	static MT_ScalarTracer<Scalar> acos(MT_ScalarTracer<Scalar> x) { return ::acos(x); } 
	static MT_ScalarTracer<Scalar> asin(MT_ScalarTracer<Scalar> x) { return ::asin(x); } 
	static MT_ScalarTracer<Scalar> atan(MT_ScalarTracer<Scalar> x) { return ::atan(x); } 
	static MT_ScalarTracer<Scalar> atan2(MT_ScalarTracer<Scalar> x, MT_ScalarTracer<Scalar> y) { return ::atan2(x, y); } 

	static MT_ScalarTracer<Scalar> exp(MT_ScalarTracer<Scalar> x) { return ::exp(x); } 
	static MT_ScalarTracer<Scalar> log(MT_ScalarTracer<Scalar> x) { return ::log(x); } 
	static MT_ScalarTracer<Scalar> pow(MT_ScalarTracer<Scalar> x, MT_ScalarTracer<Scalar> y) { return ::pow(x, y); } 
};

#endif





