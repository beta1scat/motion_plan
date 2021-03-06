/******************************************
 * C++ Trigintaduonions
 * Version: 1.0.9
 * Author:  Douglas Wilhelm Harder
 * Date:    2008/03/03
 *
 * Copyright (c) 2006-2008 by Douglas Wilhelm Harder.
 * All rights reserved.
 ******************************************/

#include "Complex.h"
#include "Trigintaduonion.h"
#include "Support.h"
#include <iostream>
#include <cmath>
#include <string>

/******************************************
 * Constructors
 ******************************************/

template <typename T> Trigintaduonion<T>::Trigintaduonion( T real, T imagi, T imagj, T imagk, T image1, T imagi1, T imagj1, T imagk1, T image2, T imagi2, T imagj2, T imagk2, T image3, T imagi3, T imagj3, T imagk3, T image4, T imagi4, T imagj4, T imagk4, T image5, T imagi5, T imagj5, T imagk5, T image6, T imagi6, T imagj6, T imagk6, T image7, T imagi7, T imagj7, T imagk7 ):
	 r(real),   i(imagi),   j(imagj),  k(imagk),
	u1(image1), i1(imagi1), j1(imagj1), k1(imagk1),
	u2(image2), i2(imagi2), j2(imagj2), k2(imagk2),
	u3(image3), i3(imagi3), j3(imagj3), k3(imagk3),
	u4(image4), i4(imagi4), j4(imagj4), k4(imagk4),
	u5(image5), i5(imagi5), j5(imagj5), k5(imagk5),
	u6(image6), i6(imagi6), j6(imagj6), k6(imagk6),
	u7(image7), i7(imagi7), j7(imagj7), k7(imagk7)
{
	// empty constructor
}

template <typename T> Trigintaduonion<T>::Trigintaduonion( T real ):
	 r(real), i(0.0),  j(0.0),  k(0.0),
	u1(0.0), i1(0.0), j1(0.0), k1(0.0),
	u2(0.0), i2(0.0), j2(0.0), k2(0.0),
	u3(0.0), i3(0.0), j3(0.0), k3(0.0),
	u4(0.0), i4(0.0), j4(0.0), k4(0.0),
	u5(0.0), i5(0.0), j5(0.0), k5(0.0),
	u6(0.0), i6(0.0), j6(0.0), k6(0.0),
	u7(0.0), i7(0.0), j7(0.0), k7(0.0)
{
	// empty constructor
}

/******************************************
 * Assignment Operator
 ******************************************/

template <typename T> const Trigintaduonion<T> & Trigintaduonion<T>::operator = ( T real ) {
	r = real;
	i = 0.0;
	j = 0.0;
	k = 0.0;
	u1 = 0.0;
	i1 = 0.0;
	j1 = 0.0;
	k1 = 0.0;
	u2 = 0.0;
	i2 = 0.0;
	j2 = 0.0;
	k2 = 0.0;
	u3 = 0.0;
	i3 = 0.0;
	j3 = 0.0;
	k3 = 0.0;
	u4 = 0.0;
	i4 = 0.0;
	j4 = 0.0;
	k4 = 0.0;
	u5 = 0.0;
	i5 = 0.0;
	j5 = 0.0;
	k5 = 0.0;
	u6 = 0.0;
	i6 = 0.0;
	j6 = 0.0;
	k6 = 0.0;
	u7 = 0.0;
	i7 = 0.0;
	j7 = 0.0;
	k7 = 0.0;

	return *this;
}

/******************************************
 * Mutating Arithmetic Operators
 ******************************************/

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator += ( const Trigintaduonion<T> & q ) {
	 r += q.r;
	 i += q.i;
	 j += q.j;
	 k += q.k;
	u1 += q.u1;
	i1 += q.i1;
	j1 += q.j1;
	k1 += q.k1;
	u2 += q.u2;
	i2 += q.i2;
	j2 += q.j2;
	k2 += q.k2;
	u3 += q.u3;
	i3 += q.i3;
	j3 += q.j3;
	k3 += q.k3;
	u4 += q.u4;
	i4 += q.i4;
	j4 += q.j4;
	k4 += q.k4;
	u5 += q.u5;
	i5 += q.i5;
	j5 += q.j5;
	k5 += q.k5;
	u6 += q.u6;
	i6 += q.i6;
	j6 += q.j6;
	k6 += q.k6;
	u7 += q.u7;
	i7 += q.i7;
	j7 += q.j7;
	k7 += q.k7;

	return *this;
}

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator -= ( const Trigintaduonion<T> & q ) {
	 r -= q.r;
	 i -= q.i;
	 j -= q.j;
	 k -= q.k;
	u1 -= q.u1;
	i1 -= q.i1;
	j1 -= q.j1;
	k1 -= q.k1;
	u2 -= q.u2;
	i2 -= q.i2;
	j2 -= q.j2;
	k2 -= q.k2;
	u3 -= q.u3;
	i3 -= q.i3;
	j3 -= q.j3;
	k3 -= q.k3;
	u4 -= q.u4;
	i4 -= q.i4;
	j4 -= q.j4;
	k4 -= q.k4;
	u5 -= q.u5;
	i5 -= q.i5;
	j5 -= q.j5;
	k5 -= q.k5;
	u6 -= q.u6;
	i6 -= q.i6;
	j6 -= q.j6;
	k6 -= q.k6;
	u7 -= q.u7;
	i7 -= q.i7;
	j7 -= q.j7;
	k7 -= q.k7;

	return *this;
}

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator *= ( const Trigintaduonion<T> & q ) {
	T RE = r, I = i, J = j, K = k, U1 = u1, I1 = i1, J1 = j1, K1 = k1, U2 = u2, I2 = i2, J2 = j2, K2 = k2, U3 = u3, I3 = i3, J3 = j3, K3 = k3, U4 = u4, I4 = i4, J4 = j4, K4 = k4, U5 = u5, I5 = i5, J5 = j5, K5 = k5, U6 = u6, I6 = i6, J6 = j6, K6 = k6, U7 = u7, I7 = i7, J7 = j7;

	r = RE*q.r - I*q.i - J*q.j - K*q.k - U1*q.u1 - I1*q.i1 - J1*q.j1 - K1*q.k1 - U2*q.u2 - I2*q.i2 - J2*q.j2 - K2*q.k2 - U3*q.u3 - I3*q.i3 - J3*q.j3 - K3*q.k3 - U4*q.u4 - I4*q.i4 - J4*q.j4 - K4*q.k4 - U5*q.u5 - I5*q.i5 - J5*q.j5 - K5*q.k5 - U6*q.u6 - I6*q.i6 - J6*q.j6 - K6*q.k6 - U7*q.u7 - I7*q.i7 - J7*q.j7 - k7*q.k7;
	i = RE*q.i + I*q.r + J*q.k - K*q.j + U1*q.i1 - I1*q.u1 - J1*q.k1 + K1*q.j1 + U2*q.i2 - I2*q.u2 - J2*q.k2 + K2*q.j2 - U3*q.i3 + I3*q.u3 + J3*q.k3 - K3*q.j3 + U4*q.i4 - I4*q.u4 - J4*q.k4 + K4*q.j4 - U5*q.i5 + I5*q.u5 + J5*q.k5 - K5*q.j5 - U6*q.i6 + I6*q.u6 + J6*q.k6 - K6*q.j6 + U7*q.i7 - I7*q.u7 - J7*q.k7 + k7*q.j7;
	j = RE*q.j - I*q.k + J*q.r + K*q.i + U1*q.j1 + I1*q.k1 - J1*q.u1 - K1*q.i1 + U2*q.j2 + I2*q.k2 - J2*q.u2 - K2*q.i2 - U3*q.j3 - I3*q.k3 + J3*q.u3 + K3*q.i3 + U4*q.j4 + I4*q.k4 - J4*q.u4 - K4*q.i4 - U5*q.j5 - I5*q.k5 + J5*q.u5 + K5*q.i5 - U6*q.j6 - I6*q.k6 + J6*q.u6 + K6*q.i6 + U7*q.j7 + I7*q.k7 - J7*q.u7 - k7*q.i7;
	k = RE*q.k + I*q.j - J*q.i + K*q.r + U1*q.k1 - I1*q.j1 + J1*q.i1 - K1*q.u1 + U2*q.k2 - I2*q.j2 + J2*q.i2 - K2*q.u2 - U3*q.k3 + I3*q.j3 - J3*q.i3 + K3*q.u3 + U4*q.k4 - I4*q.j4 + J4*q.i4 - K4*q.u4 - U5*q.k5 + I5*q.j5 - J5*q.i5 + K5*q.u5 - U6*q.k6 + I6*q.j6 - J6*q.i6 + K6*q.u6 + U7*q.k7 - I7*q.j7 + J7*q.i7 - k7*q.u7;
	u1 = RE*q.u1 - I*q.i1 - J*q.j1 - K*q.k1 + U1*q.r + I1*q.i + J1*q.j + K1*q.k + U2*q.u3 + I2*q.i3 + J2*q.j3 + K2*q.k3 - U3*q.u2 - I3*q.i2 - J3*q.j2 - K3*q.k2 + U4*q.u5 + I4*q.i5 + J4*q.j5 + K4*q.k5 - U5*q.u4 - I5*q.i4 - J5*q.j4 - K5*q.k4 - U6*q.u7 - I6*q.i7 - J6*q.j7 - K6*q.k7 + U7*q.u6 + I7*q.i6 + J7*q.j6 + k7*q.k6;
	i1 = RE*q.i1 + I*q.u1 - J*q.k1 + K*q.j1 - U1*q.i + I1*q.r - J1*q.k + K1*q.j + U2*q.i3 - I2*q.u3 + J2*q.k3 - K2*q.j3 + U3*q.i2 - I3*q.u2 + J3*q.k2 - K3*q.j2 + U4*q.i5 - I4*q.u5 + J4*q.k5 - K4*q.j5 + U5*q.i4 - I5*q.u4 + J5*q.k4 - K5*q.j4 - U6*q.i7 + I6*q.u7 - J6*q.k7 + K6*q.j7 - U7*q.i6 + I7*q.u6 - J7*q.k6 + k7*q.j6;
	j1 = RE*q.j1 + I*q.k1 + J*q.u1 - K*q.i1 - U1*q.j + I1*q.k + J1*q.r - K1*q.i + U2*q.j3 - I2*q.k3 - J2*q.u3 + K2*q.i3 + U3*q.j2 - I3*q.k2 - J3*q.u2 + K3*q.i2 + U4*q.j5 - I4*q.k5 - J4*q.u5 + K4*q.i5 + U5*q.j4 - I5*q.k4 - J5*q.u4 + K5*q.i4 - U6*q.j7 + I6*q.k7 + J6*q.u7 - K6*q.i7 - U7*q.j6 + I7*q.k6 + J7*q.u6 - k7*q.i6;
	k1 = RE*q.k1 - I*q.j1 + J*q.i1 + K*q.u1 - U1*q.k - I1*q.j + J1*q.i + K1*q.r + U2*q.k3 + I2*q.j3 - J2*q.i3 - K2*q.u3 + U3*q.k2 + I3*q.j2 - J3*q.i2 - K3*q.u2 + U4*q.k5 + I4*q.j5 - J4*q.i5 - K4*q.u5 + U5*q.k4 + I5*q.j4 - J5*q.i4 - K5*q.u4 - U6*q.k7 - I6*q.j7 + J6*q.i7 + K6*q.u7 - U7*q.k6 - I7*q.j6 + J7*q.i6 + k7*q.u6;
	u2 = RE*q.u2 - I*q.i2 - J*q.j2 - K*q.k2 - U1*q.u3 - I1*q.i3 - J1*q.j3 - K1*q.k3 + U2*q.r + I2*q.i + J2*q.j + K2*q.k + U3*q.u1 + I3*q.i1 + J3*q.j1 + K3*q.k1 + U4*q.u6 + I4*q.i6 + J4*q.j6 + K4*q.k6 + U5*q.u7 + I5*q.i7 + J5*q.j7 + K5*q.k7 - U6*q.u4 - I6*q.i4 - J6*q.j4 - K6*q.k4 - U7*q.u5 - I7*q.i5 - J7*q.j5 - k7*q.k5;
	i2 = RE*q.i2 + I*q.u2 - J*q.k2 + K*q.j2 - U1*q.i3 + I1*q.u3 + J1*q.k3 - K1*q.j3 - U2*q.i + I2*q.r - J2*q.k + K2*q.j - U3*q.i1 + I3*q.u1 + J3*q.k1 - K3*q.j1 + U4*q.i6 - I4*q.u6 + J4*q.k6 - K4*q.j6 + U5*q.i7 - I5*q.u7 - J5*q.k7 + K5*q.j7 + U6*q.i4 - I6*q.u4 + J6*q.k4 - K6*q.j4 + U7*q.i5 - I7*q.u5 - J7*q.k5 + k7*q.j5;
	j2 = RE*q.j2 + I*q.k2 + J*q.u2 - K*q.i2 - U1*q.j3 - I1*q.k3 + J1*q.u3 + K1*q.i3 - U2*q.j + I2*q.k + J2*q.r - K2*q.i - U3*q.j1 - I3*q.k1 + J3*q.u1 + K3*q.i1 + U4*q.j6 - I4*q.k6 - J4*q.u6 + K4*q.i6 + U5*q.j7 + I5*q.k7 - J5*q.u7 - K5*q.i7 + U6*q.j4 - I6*q.k4 - J6*q.u4 + K6*q.i4 + U7*q.j5 + I7*q.k5 - J7*q.u5 - k7*q.i5;
	k2 = RE*q.k2 - I*q.j2 + J*q.i2 + K*q.u2 - U1*q.k3 + I1*q.j3 - J1*q.i3 + K1*q.u3 - U2*q.k - I2*q.j + J2*q.i + K2*q.r - U3*q.k1 + I3*q.j1 - J3*q.i1 + K3*q.u1 + U4*q.k6 + I4*q.j6 - J4*q.i6 - K4*q.u6 + U5*q.k7 - I5*q.j7 + J5*q.i7 - K5*q.u7 + U6*q.k4 + I6*q.j4 - J6*q.i4 - K6*q.u4 + U7*q.k5 - I7*q.j5 + J7*q.i5 - k7*q.u5;
	u3 = RE*q.u3 + I*q.i3 + J*q.j3 + K*q.k3 + U1*q.u2 - I1*q.i2 - J1*q.j2 - K1*q.k2 - U2*q.u1 + I2*q.i1 + J2*q.j1 + K2*q.k1 + U3*q.r - I3*q.i - J3*q.j - K3*q.k + U4*q.u7 - I4*q.i7 - J4*q.j7 - K4*q.k7 - U5*q.u6 + I5*q.i6 + J5*q.j6 + K5*q.k6 + U6*q.u5 - I6*q.i5 - J6*q.j5 - K6*q.k5 - U7*q.u4 + I7*q.i4 + J7*q.j4 + k7*q.k4;
	i3 = RE*q.i3 - I*q.u3 + J*q.k3 - K*q.j3 + U1*q.i2 + I1*q.u2 + J1*q.k2 - K1*q.j2 - U2*q.i1 - I2*q.u1 + J2*q.k1 - K2*q.j1 + U3*q.i + I3*q.r + J3*q.k - K3*q.j + U4*q.i7 + I4*q.u7 - J4*q.k7 + K4*q.j7 - U5*q.i6 - I5*q.u6 - J5*q.k6 + K5*q.j6 + U6*q.i5 + I6*q.u5 - J6*q.k5 + K6*q.j5 - U7*q.i4 - I7*q.u4 - J7*q.k4 + k7*q.j4;
	j3 = RE*q.j3 - I*q.k3 - J*q.u3 + K*q.i3 + U1*q.j2 - I1*q.k2 + J1*q.u2 + K1*q.i2 - U2*q.j1 - I2*q.k1 - J2*q.u1 + K2*q.i1 + U3*q.j - I3*q.k + J3*q.r + K3*q.i + U4*q.j7 + I4*q.k7 + J4*q.u7 - K4*q.i7 - U5*q.j6 + I5*q.k6 - J5*q.u6 - K5*q.i6 + U6*q.j5 + I6*q.k5 + J6*q.u5 - K6*q.i5 - U7*q.j4 + I7*q.k4 - J7*q.u4 - k7*q.i4;
	k3 = RE*q.k3 + I*q.j3 - J*q.i3 - K*q.u3 + U1*q.k2 + I1*q.j2 - J1*q.i2 + K1*q.u2 - U2*q.k1 + I2*q.j1 - J2*q.i1 - K2*q.u1 + U3*q.k + I3*q.j - J3*q.i + K3*q.r + U4*q.k7 - I4*q.j7 + J4*q.i7 + K4*q.u7 - U5*q.k6 - I5*q.j6 + J5*q.i6 - K5*q.u6 + U6*q.k5 - I6*q.j5 + J6*q.i5 + K6*q.u5 - U7*q.k4 - I7*q.j4 + J7*q.i4 - k7*q.u4;
	u4 = RE*q.u4 - I*q.i4 - J*q.j4 - K*q.k4 - U1*q.u5 - I1*q.i5 - J1*q.j5 - K1*q.k5 - U2*q.u6 - I2*q.i6 - J2*q.j6 - K2*q.k6 - U3*q.u7 - I3*q.i7 - J3*q.j7 - K3*q.k7 + U4*q.r + I4*q.i + J4*q.j + K4*q.k + U5*q.u1 + I5*q.i1 + J5*q.j1 + K5*q.k1 + U6*q.u2 + I6*q.i2 + J6*q.j2 + K6*q.k2 + U7*q.u3 + I7*q.i3 + J7*q.j3 + k7*q.k3;
	i4 = RE*q.i4 + I*q.u4 - J*q.k4 + K*q.j4 - U1*q.i5 + I1*q.u5 + J1*q.k5 - K1*q.j5 - U2*q.i6 + I2*q.u6 + J2*q.k6 - K2*q.j6 + U3*q.i7 - I3*q.u7 - J3*q.k7 + K3*q.j7 - U4*q.i + I4*q.r - J4*q.k + K4*q.j - U5*q.i1 + I5*q.u1 + J5*q.k1 - K5*q.j1 - U6*q.i2 + I6*q.u2 + J6*q.k2 - K6*q.j2 + U7*q.i3 - I7*q.u3 - J7*q.k3 + k7*q.j3;
	j4 = RE*q.j4 + I*q.k4 + J*q.u4 - K*q.i4 - U1*q.j5 - I1*q.k5 + J1*q.u5 + K1*q.i5 - U2*q.j6 - I2*q.k6 + J2*q.u6 + K2*q.i6 + U3*q.j7 + I3*q.k7 - J3*q.u7 - K3*q.i7 - U4*q.j + I4*q.k + J4*q.r - K4*q.i - U5*q.j1 - I5*q.k1 + J5*q.u1 + K5*q.i1 - U6*q.j2 - I6*q.k2 + J6*q.u2 + K6*q.i2 + U7*q.j3 + I7*q.k3 - J7*q.u3 - k7*q.i3;
	k4 = RE*q.k4 - I*q.j4 + J*q.i4 + K*q.u4 - U1*q.k5 + I1*q.j5 - J1*q.i5 + K1*q.u5 - U2*q.k6 + I2*q.j6 - J2*q.i6 + K2*q.u6 + U3*q.k7 - I3*q.j7 + J3*q.i7 - K3*q.u7 - U4*q.k - I4*q.j + J4*q.i + K4*q.r - U5*q.k1 + I5*q.j1 - J5*q.i1 + K5*q.u1 - U6*q.k2 + I6*q.j2 - J6*q.i2 + K6*q.u2 + U7*q.k3 - I7*q.j3 + J7*q.i3 - k7*q.u3;
	u5 = RE*q.u5 + I*q.i5 + J*q.j5 + K*q.k5 + U1*q.u4 - I1*q.i4 - J1*q.j4 - K1*q.k4 - U2*q.u7 - I2*q.i7 - J2*q.j7 - K2*q.k7 + U3*q.u6 + I3*q.i6 + J3*q.j6 + K3*q.k6 - U4*q.u1 + I4*q.i1 + J4*q.j1 + K4*q.k1 + U5*q.r - I5*q.i - J5*q.j - K5*q.k - U6*q.u3 - I6*q.i3 - J6*q.j3 - K6*q.k3 + U7*q.u2 + I7*q.i2 + J7*q.j2 + k7*q.k2;
	i5 = RE*q.i5 - I*q.u5 + J*q.k5 - K*q.j5 + U1*q.i4 + I1*q.u4 + J1*q.k4 - K1*q.j4 - U2*q.i7 + I2*q.u7 - J2*q.k7 + K2*q.j7 - U3*q.i6 + I3*q.u6 - J3*q.k6 + K3*q.j6 - U4*q.i1 - I4*q.u1 + J4*q.k1 - K4*q.j1 + U5*q.i + I5*q.r + J5*q.k - K5*q.j - U6*q.i3 + I6*q.u3 - J6*q.k3 + K6*q.j3 - U7*q.i2 + I7*q.u2 - J7*q.k2 + k7*q.j2;
	j5 = RE*q.j5 - I*q.k5 - J*q.u5 + K*q.i5 + U1*q.j4 - I1*q.k4 + J1*q.u4 + K1*q.i4 - U2*q.j7 + I2*q.k7 + J2*q.u7 - K2*q.i7 - U3*q.j6 + I3*q.k6 + J3*q.u6 - K3*q.i6 - U4*q.j1 - I4*q.k1 - J4*q.u1 + K4*q.i1 + U5*q.j - I5*q.k + J5*q.r + K5*q.i - U6*q.j3 + I6*q.k3 + J6*q.u3 - K6*q.i3 - U7*q.j2 + I7*q.k2 + J7*q.u2 - k7*q.i2;
	k5 = RE*q.k5 + I*q.j5 - J*q.i5 - K*q.u5 + U1*q.k4 + I1*q.j4 - J1*q.i4 + K1*q.u4 - U2*q.k7 - I2*q.j7 + J2*q.i7 + K2*q.u7 - U3*q.k6 - I3*q.j6 + J3*q.i6 + K3*q.u6 - U4*q.k1 + I4*q.j1 - J4*q.i1 - K4*q.u1 + U5*q.k + I5*q.j - J5*q.i + K5*q.r - U6*q.k3 - I6*q.j3 + J6*q.i3 + K6*q.u3 - U7*q.k2 - I7*q.j2 + J7*q.i2 + k7*q.u2;
	u6 = RE*q.u6 + I*q.i6 + J*q.j6 + K*q.k6 + U1*q.u7 + I1*q.i7 + J1*q.j7 + K1*q.k7 + U2*q.u4 - I2*q.i4 - J2*q.j4 - K2*q.k4 - U3*q.u5 - I3*q.i5 - J3*q.j5 - K3*q.k5 - U4*q.u2 + I4*q.i2 + J4*q.j2 + K4*q.k2 + U5*q.u3 + I5*q.i3 + J5*q.j3 + K5*q.k3 + U6*q.r - I6*q.i - J6*q.j - K6*q.k - U7*q.u1 - I7*q.i1 - J7*q.j1 - k7*q.k1;
	i6 = RE*q.i6 - I*q.u6 + J*q.k6 - K*q.j6 + U1*q.i7 - I1*q.u7 - J1*q.k7 + K1*q.j7 + U2*q.i4 + I2*q.u4 + J2*q.k4 - K2*q.j4 + U3*q.i5 - I3*q.u5 - J3*q.k5 + K3*q.j5 - U4*q.i2 - I4*q.u2 + J4*q.k2 - K4*q.j2 + U5*q.i3 - I5*q.u3 - J5*q.k3 + K5*q.j3 + U6*q.i + I6*q.r + J6*q.k - K6*q.j + U7*q.i1 - I7*q.u1 - J7*q.k1 + k7*q.j1;
	j6 = RE*q.j6 - I*q.k6 - J*q.u6 + K*q.i6 + U1*q.j7 + I1*q.k7 - J1*q.u7 - K1*q.i7 + U2*q.j4 - I2*q.k4 + J2*q.u4 + K2*q.i4 + U3*q.j5 + I3*q.k5 - J3*q.u5 - K3*q.i5 - U4*q.j2 - I4*q.k2 - J4*q.u2 + K4*q.i2 + U5*q.j3 + I5*q.k3 - J5*q.u3 - K5*q.i3 + U6*q.j - I6*q.k + J6*q.r + K6*q.i + U7*q.j1 + I7*q.k1 - J7*q.u1 - k7*q.i1;
	k6 = RE*q.k6 + I*q.j6 - J*q.i6 - K*q.u6 + U1*q.k7 - I1*q.j7 + J1*q.i7 - K1*q.u7 + U2*q.k4 + I2*q.j4 - J2*q.i4 + K2*q.u4 + U3*q.k5 - I3*q.j5 + J3*q.i5 - K3*q.u5 - U4*q.k2 + I4*q.j2 - J4*q.i2 - K4*q.u2 + U5*q.k3 - I5*q.j3 + J5*q.i3 - K5*q.u3 + U6*q.k + I6*q.j - J6*q.i + K6*q.r + U7*q.k1 - I7*q.j1 + J7*q.i1 - k7*q.u1;
	u7 = RE*q.u7 - I*q.i7 - J*q.j7 - K*q.k7 - U1*q.u6 + I1*q.i6 + J1*q.j6 + K1*q.k6 + U2*q.u5 - I2*q.i5 - J2*q.j5 - K2*q.k5 + U3*q.u4 + I3*q.i4 + J3*q.j4 + K3*q.k4 - U4*q.u3 - I4*q.i3 - J4*q.j3 - K4*q.k3 - U5*q.u2 + I5*q.i2 + J5*q.j2 + K5*q.k2 + U6*q.u1 - I6*q.i1 - J6*q.j1 - K6*q.k1 + U7*q.r + I7*q.i + J7*q.j + k7*q.k;
	i7 = RE*q.i7 + I*q.u7 - J*q.k7 + K*q.j7 - U1*q.i6 - I1*q.u6 - J1*q.k6 + K1*q.j6 + U2*q.i5 + I2*q.u5 - J2*q.k5 + K2*q.j5 - U3*q.i4 + I3*q.u4 - J3*q.k4 + K3*q.j4 - U4*q.i3 + I4*q.u3 - J4*q.k3 + K4*q.j3 - U5*q.i2 - I5*q.u2 - J5*q.k2 + K5*q.j2 + U6*q.i1 + I6*q.u1 - J6*q.k1 + K6*q.j1 - U7*q.i + I7*q.r - J7*q.k + k7*q.j;
	j7 = RE*q.j7 + I*q.k7 + J*q.u7 - K*q.i7 - U1*q.j6 + I1*q.k6 - J1*q.u6 - K1*q.i6 + U2*q.j5 + I2*q.k5 + J2*q.u5 - K2*q.i5 - U3*q.j4 + I3*q.k4 + J3*q.u4 - K3*q.i4 - U4*q.j3 + I4*q.k3 + J4*q.u3 - K4*q.i3 - U5*q.j2 + I5*q.k2 - J5*q.u2 - K5*q.i2 + U6*q.j1 + I6*q.k1 + J6*q.u1 - K6*q.i1 - U7*q.j + I7*q.k + J7*q.r - k7*q.i;
	k7 = RE*q.k7 - I*q.j7 + J*q.i7 + K*q.u7 - U1*q.k6 - I1*q.j6 + J1*q.i6 - K1*q.u6 + U2*q.k5 - I2*q.j5 + J2*q.i5 + K2*q.u5 - U3*q.k4 - I3*q.j4 + J3*q.i4 + K3*q.u4 - U4*q.k3 - I4*q.j3 + J4*q.i3 + K4*q.u3 - U5*q.k2 - I5*q.j2 + J5*q.i2 - K5*q.u2 + U6*q.k1 - I6*q.j1 + J6*q.i1 + K6*q.u1 - U7*q.k - I7*q.j + J7*q.i + k7*q.r;

	return * this;
}

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator /= ( const Trigintaduonion<T> & q ) {
	T denom = q.norm();
	T RE = r, I = i, J = j, K = k, U1 = u1, I1 = i1, J1 = j1, K1 = k1, U2 = u2, I2 = i2, J2 = j2, K2 = k2, U3 = u3, I3 = i3, J3 = j3, K3 = k3, U4 = u4, I4 = i4, J4 = j4, K4 = k4, U5 = u5, I5 = i5, J5 = j5, K5 = k5, U6 = u6, I6 = i6, J6 = j6, K6 = k6, U7 = u7, I7 = i7, J7 = j7;

	r = (RE*q.r + I*q.i + J*q.j + K*q.k + U1*q.u1 + I1*q.i1 + J1*q.j1 + K1*q.k1 + U2*q.u2 + I2*q.i2 + J2*q.j2 + K2*q.k2 + U3*q.u3 + I3*q.i3 + J3*q.j3 + K3*q.k3 + U4*q.u4 + I4*q.i4 + J4*q.j4 + K4*q.k4 + U5*q.u5 + I5*q.i5 + J5*q.j5 + K5*q.k5 + U6*q.u6 + I6*q.i6 + J6*q.j6 + K6*q.k6 + U7*q.u7 + I7*q.i7 + J7*q.j7 + k7*q.k7)/denom;
	i = (-RE*q.i + I*q.r - J*q.k + K*q.j - U1*q.i1 + I1*q.u1 + J1*q.k1 - K1*q.j1 - U2*q.i2 + I2*q.u2 + J2*q.k2 - K2*q.j2 + U3*q.i3 - I3*q.u3 - J3*q.k3 + K3*q.j3 - U4*q.i4 + I4*q.u4 + J4*q.k4 - K4*q.j4 + U5*q.i5 - I5*q.u5 - J5*q.k5 + K5*q.j5 + U6*q.i6 - I6*q.u6 - J6*q.k6 + K6*q.j6 - U7*q.i7 + I7*q.u7 + J7*q.k7 - k7*q.j7)/denom;
	j = (-RE*q.j + I*q.k + J*q.r - K*q.i - U1*q.j1 - I1*q.k1 + J1*q.u1 + K1*q.i1 - U2*q.j2 - I2*q.k2 + J2*q.u2 + K2*q.i2 + U3*q.j3 + I3*q.k3 - J3*q.u3 - K3*q.i3 - U4*q.j4 - I4*q.k4 + J4*q.u4 + K4*q.i4 + U5*q.j5 + I5*q.k5 - J5*q.u5 - K5*q.i5 + U6*q.j6 + I6*q.k6 - J6*q.u6 - K6*q.i6 - U7*q.j7 - I7*q.k7 + J7*q.u7 + k7*q.i7)/denom;
	k = (-RE*q.k - I*q.j + J*q.i + K*q.r - U1*q.k1 + I1*q.j1 - J1*q.i1 + K1*q.u1 - U2*q.k2 + I2*q.j2 - J2*q.i2 + K2*q.u2 + U3*q.k3 - I3*q.j3 + J3*q.i3 - K3*q.u3 - U4*q.k4 + I4*q.j4 - J4*q.i4 + K4*q.u4 + U5*q.k5 - I5*q.j5 + J5*q.i5 - K5*q.u5 + U6*q.k6 - I6*q.j6 + J6*q.i6 - K6*q.u6 - U7*q.k7 + I7*q.j7 - J7*q.i7 + k7*q.u7)/denom;
	u1 = (-RE*q.u1 + I*q.i1 + J*q.j1 + K*q.k1 + U1*q.r - I1*q.i - J1*q.j - K1*q.k - U2*q.u3 - I2*q.i3 - J2*q.j3 - K2*q.k3 + U3*q.u2 + I3*q.i2 + J3*q.j2 + K3*q.k2 - U4*q.u5 - I4*q.i5 - J4*q.j5 - K4*q.k5 + U5*q.u4 + I5*q.i4 + J5*q.j4 + K5*q.k4 + U6*q.u7 + I6*q.i7 + J6*q.j7 + K6*q.k7 - U7*q.u6 - I7*q.i6 - J7*q.j6 - k7*q.k6)/denom;
	i1 = (-RE*q.i1 - I*q.u1 + J*q.k1 - K*q.j1 + U1*q.i + I1*q.r + J1*q.k - K1*q.j - U2*q.i3 + I2*q.u3 - J2*q.k3 + K2*q.j3 - U3*q.i2 + I3*q.u2 - J3*q.k2 + K3*q.j2 - U4*q.i5 + I4*q.u5 - J4*q.k5 + K4*q.j5 - U5*q.i4 + I5*q.u4 - J5*q.k4 + K5*q.j4 + U6*q.i7 - I6*q.u7 + J6*q.k7 - K6*q.j7 + U7*q.i6 - I7*q.u6 + J7*q.k6 - k7*q.j6)/denom;
	j1 = (-RE*q.j1 - I*q.k1 - J*q.u1 + K*q.i1 + U1*q.j - I1*q.k + J1*q.r + K1*q.i - U2*q.j3 + I2*q.k3 + J2*q.u3 - K2*q.i3 - U3*q.j2 + I3*q.k2 + J3*q.u2 - K3*q.i2 - U4*q.j5 + I4*q.k5 + J4*q.u5 - K4*q.i5 - U5*q.j4 + I5*q.k4 + J5*q.u4 - K5*q.i4 + U6*q.j7 - I6*q.k7 - J6*q.u7 + K6*q.i7 + U7*q.j6 - I7*q.k6 - J7*q.u6 + k7*q.i6)/denom;
	k1 = (-RE*q.k1 + I*q.j1 - J*q.i1 - K*q.u1 + U1*q.k + I1*q.j - J1*q.i + K1*q.r - U2*q.k3 - I2*q.j3 + J2*q.i3 + K2*q.u3 - U3*q.k2 - I3*q.j2 + J3*q.i2 + K3*q.u2 - U4*q.k5 - I4*q.j5 + J4*q.i5 + K4*q.u5 - U5*q.k4 - I5*q.j4 + J5*q.i4 + K5*q.u4 + U6*q.k7 + I6*q.j7 - J6*q.i7 - K6*q.u7 + U7*q.k6 + I7*q.j6 - J7*q.i6 - k7*q.u6)/denom;
	u2 = (-RE*q.u2 + I*q.i2 + J*q.j2 + K*q.k2 + U1*q.u3 + I1*q.i3 + J1*q.j3 + K1*q.k3 + U2*q.r - I2*q.i - J2*q.j - K2*q.k - U3*q.u1 - I3*q.i1 - J3*q.j1 - K3*q.k1 - U4*q.u6 - I4*q.i6 - J4*q.j6 - K4*q.k6 - U5*q.u7 - I5*q.i7 - J5*q.j7 - K5*q.k7 + U6*q.u4 + I6*q.i4 + J6*q.j4 + K6*q.k4 + U7*q.u5 + I7*q.i5 + J7*q.j5 + k7*q.k5)/denom;
	i2 = (-RE*q.i2 - I*q.u2 + J*q.k2 - K*q.j2 + U1*q.i3 - I1*q.u3 - J1*q.k3 + K1*q.j3 + U2*q.i + I2*q.r + J2*q.k - K2*q.j + U3*q.i1 - I3*q.u1 - J3*q.k1 + K3*q.j1 - U4*q.i6 + I4*q.u6 - J4*q.k6 + K4*q.j6 - U5*q.i7 + I5*q.u7 + J5*q.k7 - K5*q.j7 - U6*q.i4 + I6*q.u4 - J6*q.k4 + K6*q.j4 - U7*q.i5 + I7*q.u5 + J7*q.k5 - k7*q.j5)/denom;
	j2 = (-RE*q.j2 - I*q.k2 - J*q.u2 + K*q.i2 + U1*q.j3 + I1*q.k3 - J1*q.u3 - K1*q.i3 + U2*q.j - I2*q.k + J2*q.r + K2*q.i + U3*q.j1 + I3*q.k1 - J3*q.u1 - K3*q.i1 - U4*q.j6 + I4*q.k6 + J4*q.u6 - K4*q.i6 - U5*q.j7 - I5*q.k7 + J5*q.u7 + K5*q.i7 - U6*q.j4 + I6*q.k4 + J6*q.u4 - K6*q.i4 - U7*q.j5 - I7*q.k5 + J7*q.u5 + k7*q.i5)/denom;
	k2 = (-RE*q.k2 + I*q.j2 - J*q.i2 - K*q.u2 + U1*q.k3 - I1*q.j3 + J1*q.i3 - K1*q.u3 + U2*q.k + I2*q.j - J2*q.i + K2*q.r + U3*q.k1 - I3*q.j1 + J3*q.i1 - K3*q.u1 - U4*q.k6 - I4*q.j6 + J4*q.i6 + K4*q.u6 - U5*q.k7 + I5*q.j7 - J5*q.i7 + K5*q.u7 - U6*q.k4 - I6*q.j4 + J6*q.i4 + K6*q.u4 - U7*q.k5 + I7*q.j5 - J7*q.i5 + k7*q.u5)/denom;
	u3 = (-RE*q.u3 - I*q.i3 - J*q.j3 - K*q.k3 - U1*q.u2 + I1*q.i2 + J1*q.j2 + K1*q.k2 + U2*q.u1 - I2*q.i1 - J2*q.j1 - K2*q.k1 + U3*q.r + I3*q.i + J3*q.j + K3*q.k - U4*q.u7 + I4*q.i7 + J4*q.j7 + K4*q.k7 + U5*q.u6 - I5*q.i6 - J5*q.j6 - K5*q.k6 - U6*q.u5 + I6*q.i5 + J6*q.j5 + K6*q.k5 + U7*q.u4 - I7*q.i4 - J7*q.j4 - k7*q.k4)/denom;
	i3 = (-RE*q.i3 + I*q.u3 - J*q.k3 + K*q.j3 - U1*q.i2 - I1*q.u2 - J1*q.k2 + K1*q.j2 + U2*q.i1 + I2*q.u1 - J2*q.k1 + K2*q.j1 - U3*q.i + I3*q.r - J3*q.k + K3*q.j - U4*q.i7 - I4*q.u7 + J4*q.k7 - K4*q.j7 + U5*q.i6 + I5*q.u6 + J5*q.k6 - K5*q.j6 - U6*q.i5 - I6*q.u5 + J6*q.k5 - K6*q.j5 + U7*q.i4 + I7*q.u4 + J7*q.k4 - k7*q.j4)/denom;
	j3 = (-RE*q.j3 + I*q.k3 + J*q.u3 - K*q.i3 - U1*q.j2 + I1*q.k2 - J1*q.u2 - K1*q.i2 + U2*q.j1 + I2*q.k1 + J2*q.u1 - K2*q.i1 - U3*q.j + I3*q.k + J3*q.r - K3*q.i - U4*q.j7 - I4*q.k7 - J4*q.u7 + K4*q.i7 + U5*q.j6 - I5*q.k6 + J5*q.u6 + K5*q.i6 - U6*q.j5 - I6*q.k5 - J6*q.u5 + K6*q.i5 + U7*q.j4 - I7*q.k4 + J7*q.u4 + k7*q.i4)/denom;
	k3 = (-RE*q.k3 - I*q.j3 + J*q.i3 + K*q.u3 - U1*q.k2 - I1*q.j2 + J1*q.i2 - K1*q.u2 + U2*q.k1 - I2*q.j1 + J2*q.i1 + K2*q.u1 - U3*q.k - I3*q.j + J3*q.i + K3*q.r - U4*q.k7 + I4*q.j7 - J4*q.i7 - K4*q.u7 + U5*q.k6 + I5*q.j6 - J5*q.i6 + K5*q.u6 - U6*q.k5 + I6*q.j5 - J6*q.i5 - K6*q.u5 + U7*q.k4 + I7*q.j4 - J7*q.i4 + k7*q.u4)/denom;
	u4 = (-RE*q.u4 + I*q.i4 + J*q.j4 + K*q.k4 + U1*q.u5 + I1*q.i5 + J1*q.j5 + K1*q.k5 + U2*q.u6 + I2*q.i6 + J2*q.j6 + K2*q.k6 + U3*q.u7 + I3*q.i7 + J3*q.j7 + K3*q.k7 + U4*q.r - I4*q.i - J4*q.j - K4*q.k - U5*q.u1 - I5*q.i1 - J5*q.j1 - K5*q.k1 - U6*q.u2 - I6*q.i2 - J6*q.j2 - K6*q.k2 - U7*q.u3 - I7*q.i3 - J7*q.j3 - k7*q.k3)/denom;
	i4 = (-RE*q.i4 - I*q.u4 + J*q.k4 - K*q.j4 + U1*q.i5 - I1*q.u5 - J1*q.k5 + K1*q.j5 + U2*q.i6 - I2*q.u6 - J2*q.k6 + K2*q.j6 - U3*q.i7 + I3*q.u7 + J3*q.k7 - K3*q.j7 + U4*q.i + I4*q.r + J4*q.k - K4*q.j + U5*q.i1 - I5*q.u1 - J5*q.k1 + K5*q.j1 + U6*q.i2 - I6*q.u2 - J6*q.k2 + K6*q.j2 - U7*q.i3 + I7*q.u3 + J7*q.k3 - k7*q.j3)/denom;
	j4 = (-RE*q.j4 - I*q.k4 - J*q.u4 + K*q.i4 + U1*q.j5 + I1*q.k5 - J1*q.u5 - K1*q.i5 + U2*q.j6 + I2*q.k6 - J2*q.u6 - K2*q.i6 - U3*q.j7 - I3*q.k7 + J3*q.u7 + K3*q.i7 + U4*q.j - I4*q.k + J4*q.r + K4*q.i + U5*q.j1 + I5*q.k1 - J5*q.u1 - K5*q.i1 + U6*q.j2 + I6*q.k2 - J6*q.u2 - K6*q.i2 - U7*q.j3 - I7*q.k3 + J7*q.u3 + k7*q.i3)/denom;
	k4 = (-RE*q.k4 + I*q.j4 - J*q.i4 - K*q.u4 + U1*q.k5 - I1*q.j5 + J1*q.i5 - K1*q.u5 + U2*q.k6 - I2*q.j6 + J2*q.i6 - K2*q.u6 - U3*q.k7 + I3*q.j7 - J3*q.i7 + K3*q.u7 + U4*q.k + I4*q.j - J4*q.i + K4*q.r + U5*q.k1 - I5*q.j1 + J5*q.i1 - K5*q.u1 + U6*q.k2 - I6*q.j2 + J6*q.i2 - K6*q.u2 - U7*q.k3 + I7*q.j3 - J7*q.i3 + k7*q.u3)/denom;
	u5 = (-RE*q.u5 - I*q.i5 - J*q.j5 - K*q.k5 - U1*q.u4 + I1*q.i4 + J1*q.j4 + K1*q.k4 + U2*q.u7 + I2*q.i7 + J2*q.j7 + K2*q.k7 - U3*q.u6 - I3*q.i6 - J3*q.j6 - K3*q.k6 + U4*q.u1 - I4*q.i1 - J4*q.j1 - K4*q.k1 + U5*q.r + I5*q.i + J5*q.j + K5*q.k + U6*q.u3 + I6*q.i3 + J6*q.j3 + K6*q.k3 - U7*q.u2 - I7*q.i2 - J7*q.j2 - k7*q.k2)/denom;
	i5 = (-RE*q.i5 + I*q.u5 - J*q.k5 + K*q.j5 - U1*q.i4 - I1*q.u4 - J1*q.k4 + K1*q.j4 + U2*q.i7 - I2*q.u7 + J2*q.k7 - K2*q.j7 + U3*q.i6 - I3*q.u6 + J3*q.k6 - K3*q.j6 + U4*q.i1 + I4*q.u1 - J4*q.k1 + K4*q.j1 - U5*q.i + I5*q.r - J5*q.k + K5*q.j + U6*q.i3 - I6*q.u3 + J6*q.k3 - K6*q.j3 + U7*q.i2 - I7*q.u2 + J7*q.k2 - k7*q.j2)/denom;
	j5 = (-RE*q.j5 + I*q.k5 + J*q.u5 - K*q.i5 - U1*q.j4 + I1*q.k4 - J1*q.u4 - K1*q.i4 + U2*q.j7 - I2*q.k7 - J2*q.u7 + K2*q.i7 + U3*q.j6 - I3*q.k6 - J3*q.u6 + K3*q.i6 + U4*q.j1 + I4*q.k1 + J4*q.u1 - K4*q.i1 - U5*q.j + I5*q.k + J5*q.r - K5*q.i + U6*q.j3 - I6*q.k3 - J6*q.u3 + K6*q.i3 + U7*q.j2 - I7*q.k2 - J7*q.u2 + k7*q.i2)/denom;
	k5 = (-RE*q.k5 - I*q.j5 + J*q.i5 + K*q.u5 - U1*q.k4 - I1*q.j4 + J1*q.i4 - K1*q.u4 + U2*q.k7 + I2*q.j7 - J2*q.i7 - K2*q.u7 + U3*q.k6 + I3*q.j6 - J3*q.i6 - K3*q.u6 + U4*q.k1 - I4*q.j1 + J4*q.i1 + K4*q.u1 - U5*q.k - I5*q.j + J5*q.i + K5*q.r + U6*q.k3 + I6*q.j3 - J6*q.i3 - K6*q.u3 + U7*q.k2 + I7*q.j2 - J7*q.i2 - k7*q.u2)/denom;
	u6 = (-RE*q.u6 - I*q.i6 - J*q.j6 - K*q.k6 - U1*q.u7 - I1*q.i7 - J1*q.j7 - K1*q.k7 - U2*q.u4 + I2*q.i4 + J2*q.j4 + K2*q.k4 + U3*q.u5 + I3*q.i5 + J3*q.j5 + K3*q.k5 + U4*q.u2 - I4*q.i2 - J4*q.j2 - K4*q.k2 - U5*q.u3 - I5*q.i3 - J5*q.j3 - K5*q.k3 + U6*q.r + I6*q.i + J6*q.j + K6*q.k + U7*q.u1 + I7*q.i1 + J7*q.j1 + k7*q.k1)/denom;
	i6 = (-RE*q.i6 + I*q.u6 - J*q.k6 + K*q.j6 - U1*q.i7 + I1*q.u7 + J1*q.k7 - K1*q.j7 - U2*q.i4 - I2*q.u4 - J2*q.k4 + K2*q.j4 - U3*q.i5 + I3*q.u5 + J3*q.k5 - K3*q.j5 + U4*q.i2 + I4*q.u2 - J4*q.k2 + K4*q.j2 - U5*q.i3 + I5*q.u3 + J5*q.k3 - K5*q.j3 - U6*q.i + I6*q.r - J6*q.k + K6*q.j - U7*q.i1 + I7*q.u1 + J7*q.k1 - k7*q.j1)/denom;
	j6 = (-RE*q.j6 + I*q.k6 + J*q.u6 - K*q.i6 - U1*q.j7 - I1*q.k7 + J1*q.u7 + K1*q.i7 - U2*q.j4 + I2*q.k4 - J2*q.u4 - K2*q.i4 - U3*q.j5 - I3*q.k5 + J3*q.u5 + K3*q.i5 + U4*q.j2 + I4*q.k2 + J4*q.u2 - K4*q.i2 - U5*q.j3 - I5*q.k3 + J5*q.u3 + K5*q.i3 - U6*q.j + I6*q.k + J6*q.r - K6*q.i - U7*q.j1 - I7*q.k1 + J7*q.u1 + k7*q.i1)/denom;
	k6 = (-RE*q.k6 - I*q.j6 + J*q.i6 + K*q.u6 - U1*q.k7 + I1*q.j7 - J1*q.i7 + K1*q.u7 - U2*q.k4 - I2*q.j4 + J2*q.i4 - K2*q.u4 - U3*q.k5 + I3*q.j5 - J3*q.i5 + K3*q.u5 + U4*q.k2 - I4*q.j2 + J4*q.i2 + K4*q.u2 - U5*q.k3 + I5*q.j3 - J5*q.i3 + K5*q.u3 - U6*q.k - I6*q.j + J6*q.i + K6*q.r - U7*q.k1 + I7*q.j1 - J7*q.i1 + k7*q.u1)/denom;
	u7 = (-RE*q.u7 + I*q.i7 + J*q.j7 + K*q.k7 + U1*q.u6 - I1*q.i6 - J1*q.j6 - K1*q.k6 - U2*q.u5 + I2*q.i5 + J2*q.j5 + K2*q.k5 - U3*q.u4 - I3*q.i4 - J3*q.j4 - K3*q.k4 + U4*q.u3 + I4*q.i3 + J4*q.j3 + K4*q.k3 + U5*q.u2 - I5*q.i2 - J5*q.j2 - K5*q.k2 - U6*q.u1 + I6*q.i1 + J6*q.j1 + K6*q.k1 + U7*q.r - I7*q.i - J7*q.j - k7*q.k)/denom;
	i7 = (-RE*q.i7 - I*q.u7 + J*q.k7 - K*q.j7 + U1*q.i6 + I1*q.u6 + J1*q.k6 - K1*q.j6 - U2*q.i5 - I2*q.u5 + J2*q.k5 - K2*q.j5 + U3*q.i4 - I3*q.u4 + J3*q.k4 - K3*q.j4 + U4*q.i3 - I4*q.u3 + J4*q.k3 - K4*q.j3 + U5*q.i2 + I5*q.u2 + J5*q.k2 - K5*q.j2 - U6*q.i1 - I6*q.u1 + J6*q.k1 - K6*q.j1 + U7*q.i + I7*q.r + J7*q.k - k7*q.j)/denom;
	j7 = (-RE*q.j7 - I*q.k7 - J*q.u7 + K*q.i7 + U1*q.j6 - I1*q.k6 + J1*q.u6 + K1*q.i6 - U2*q.j5 - I2*q.k5 - J2*q.u5 + K2*q.i5 + U3*q.j4 - I3*q.k4 - J3*q.u4 + K3*q.i4 + U4*q.j3 - I4*q.k3 - J4*q.u3 + K4*q.i3 + U5*q.j2 - I5*q.k2 + J5*q.u2 + K5*q.i2 - U6*q.j1 - I6*q.k1 - J6*q.u1 + K6*q.i1 + U7*q.j - I7*q.k + J7*q.r + k7*q.i)/denom;
	k7 = (-RE*q.k7 + I*q.j7 - J*q.i7 - K*q.u7 + U1*q.k6 + I1*q.j6 - J1*q.i6 + K1*q.u6 - U2*q.k5 + I2*q.j5 - J2*q.i5 - K2*q.u5 + U3*q.k4 + I3*q.j4 - J3*q.i4 - K3*q.u4 + U4*q.k3 + I4*q.j3 - J4*q.i3 - K4*q.u3 + U5*q.k2 + I5*q.j2 - J5*q.i2 + K5*q.u2 - U6*q.k1 + I6*q.j1 - J6*q.i1 - K6*q.u1 + U7*q.k + I7*q.j - J7*q.i + k7*q.r)/denom;

	return * this;
}


template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator += ( T x ) {
	r += x;

	return *this;
}

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator -= ( T x ) {
	r -= x;

	return *this;
}

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator *= ( T x ) {
	if ( Support<T>::is_inf( x ) && norm() > 0 ) {
		 r *= (  r == 0 )?Support<T>::sign( x ):x;
		 i *= (  i == 0 )?Support<T>::sign( x ):x;
		 j *= (  j == 0 )?Support<T>::sign( x ):x;
		 k *= (  k == 0 )?Support<T>::sign( x ):x;
		u1 *= ( u1 == 0 )?Support<T>::sign( x ):x;
		i1 *= ( i1 == 0 )?Support<T>::sign( x ):x;
		j1 *= ( j1 == 0 )?Support<T>::sign( x ):x;
		k1 *= ( k1 == 0 )?Support<T>::sign( x ):x;
		u2 *= ( u2 == 0 )?Support<T>::sign( x ):x;
		i2 *= ( i2 == 0 )?Support<T>::sign( x ):x;
		j2 *= ( j2 == 0 )?Support<T>::sign( x ):x;
		k2 *= ( k2 == 0 )?Support<T>::sign( x ):x;
		u3 *= ( u3 == 0 )?Support<T>::sign( x ):x;
		i3 *= ( i3 == 0 )?Support<T>::sign( x ):x;
		j3 *= ( j3 == 0 )?Support<T>::sign( x ):x;
		k3 *= ( k3 == 0 )?Support<T>::sign( x ):x;
		u4 *= ( u4 == 0 )?Support<T>::sign( x ):x;
		i4 *= ( i4 == 0 )?Support<T>::sign( x ):x;
		j4 *= ( j4 == 0 )?Support<T>::sign( x ):x;
		k4 *= ( k4 == 0 )?Support<T>::sign( x ):x;
		u5 *= ( u5 == 0 )?Support<T>::sign( x ):x;
		i5 *= ( i5 == 0 )?Support<T>::sign( x ):x;
		j5 *= ( j5 == 0 )?Support<T>::sign( x ):x;
		k5 *= ( k5 == 0 )?Support<T>::sign( x ):x;
		u6 *= ( u6 == 0 )?Support<T>::sign( x ):x;
		i6 *= ( i6 == 0 )?Support<T>::sign( x ):x;
		j6 *= ( j6 == 0 )?Support<T>::sign( x ):x;
		k6 *= ( k6 == 0 )?Support<T>::sign( x ):x;
		u7 *= ( u7 == 0 )?Support<T>::sign( x ):x;
		i7 *= ( i7 == 0 )?Support<T>::sign( x ):x;
		j7 *= ( j7 == 0 )?Support<T>::sign( x ):x;
		k7 *= ( k7 == 0 )?Support<T>::sign( x ):x;
	} else {
		 r *= x;
		 i *= x;
		 j *= x;
		 k *= x;
		u1 *= x;
		i1 *= x;
		j1 *= x;
		k1 *= x;
		u2 *= x;
		i2 *= x;
		j2 *= x;
		k2 *= x;
		u3 *= x;
		i3 *= x;
		j3 *= x;
		k3 *= x;
		u4 *= x;
		i4 *= x;
		j4 *= x;
		k4 *= x;
		u5 *= x;
		i5 *= x;
		j5 *= x;
		k5 *= x;
		u6 *= x;
		i6 *= x;
		j6 *= x;
		k6 *= x;
		u7 *= x;
		i7 *= x;
		j7 *= x;
		k7 *= x;
	}

	return * this;
}

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator /= ( T x ) {
	if ( x == 0.0 && !is_zero() ) {
		 r /= (  r == 0 )?Support<T>::sign( x ):x;
		 i /= (  i == 0 )?Support<T>::sign( x ):x;
		 j /= (  j == 0 )?Support<T>::sign( x ):x;
		 k /= (  k == 0 )?Support<T>::sign( x ):x;
		u1 /= ( u1 == 0 )?Support<T>::sign( x ):x;
		i1 /= ( i1 == 0 )?Support<T>::sign( x ):x;
		j1 /= ( j1 == 0 )?Support<T>::sign( x ):x;
		k1 /= ( k1 == 0 )?Support<T>::sign( x ):x;
		u2 /= ( u2 == 0 )?Support<T>::sign( x ):x;
		i2 /= ( i2 == 0 )?Support<T>::sign( x ):x;
		j2 /= ( j2 == 0 )?Support<T>::sign( x ):x;
		k2 /= ( k2 == 0 )?Support<T>::sign( x ):x;
		u3 /= ( u3 == 0 )?Support<T>::sign( x ):x;
		i3 /= ( i3 == 0 )?Support<T>::sign( x ):x;
		j3 /= ( j3 == 0 )?Support<T>::sign( x ):x;
		k3 /= ( k3 == 0 )?Support<T>::sign( x ):x;
		u4 /= ( u4 == 0 )?Support<T>::sign( x ):x;
		i4 /= ( i4 == 0 )?Support<T>::sign( x ):x;
		j4 /= ( j4 == 0 )?Support<T>::sign( x ):x;
		k4 /= ( k4 == 0 )?Support<T>::sign( x ):x;
		u5 /= ( u5 == 0 )?Support<T>::sign( x ):x;
		i5 /= ( i5 == 0 )?Support<T>::sign( x ):x;
		j5 /= ( j5 == 0 )?Support<T>::sign( x ):x;
		k5 /= ( k5 == 0 )?Support<T>::sign( x ):x;
		u6 /= ( u6 == 0 )?Support<T>::sign( x ):x;
		i6 /= ( i6 == 0 )?Support<T>::sign( x ):x;
		j6 /= ( j6 == 0 )?Support<T>::sign( x ):x;
		k6 /= ( k6 == 0 )?Support<T>::sign( x ):x;
		u7 /= ( u7 == 0 )?Support<T>::sign( x ):x;
		i7 /= ( i7 == 0 )?Support<T>::sign( x ):x;
		j7 /= ( j7 == 0 )?Support<T>::sign( x ):x;
		k7 /= ( k7 == 0 )?Support<T>::sign( x ):x;
	} else {
		 r /= x;
		 i /= x;
		 j /= x;
		 k /= x;
		u1 /= x;
		i1 /= x;
		j1 /= x;
		k1 /= x;
		u2 /= x;
		i2 /= x;
		j2 /= x;
		k2 /= x;
		u3 /= x;
		i3 /= x;
		j3 /= x;
		k3 /= x;
		u4 /= x;
		i4 /= x;
		j4 /= x;
		k4 /= x;
		u5 /= x;
		i5 /= x;
		j5 /= x;
		k5 /= x;
		u6 /= x;
		i6 /= x;
		j6 /= x;
		k6 /= x;
		u7 /= x;
		i7 /= x;
		j7 /= x;
		k7 /= x;
	}

	return * this;
}

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator ++() {
	++r;

	return *this;
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator ++( int ) {
	Trigintaduonion copy = *this;
	++r;

	return copy;
}

template <typename T> Trigintaduonion<T> & Trigintaduonion<T>::operator --() {
	--r;

	return *this;
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator --( int ) {
	Trigintaduonion copy = *this;
	--r;

	return copy;
}

/******************************************
 * Real-valued Functions
 ******************************************/

template <typename T> T Trigintaduonion<T>::real() const {
	return r;
}

template <typename T> T Trigintaduonion<T>::operator []( int n ) const {
	return reinterpret_cast<const T *>( this )[n];
}

template <typename T> T& Trigintaduonion<T>::operator []( int n ) {
	return reinterpret_cast<T *>( this )[n];
}

template <typename T> T Trigintaduonion<T>::imag_i() const {
	return i;
}

template <typename T> T Trigintaduonion<T>::imag_j() const {
	return j;
}

template <typename T> T Trigintaduonion<T>::imag_k() const {
	return k;
}

template <typename T> T Trigintaduonion<T>::imag_u1() const {
	return u1;
}

template <typename T> T Trigintaduonion<T>::imag_i1() const {
	return i1;
}

template <typename T> T Trigintaduonion<T>::imag_j1() const {
	return j1;
}

template <typename T> T Trigintaduonion<T>::imag_k1() const {
	return k1;
}

template <typename T> T Trigintaduonion<T>::imag_u2() const {
	return u2;
}

template <typename T> T Trigintaduonion<T>::imag_i2() const {
	return i2;
}

template <typename T> T Trigintaduonion<T>::imag_j2() const {
	return j2;
}

template <typename T> T Trigintaduonion<T>::imag_k2() const {
	return k2;
}

template <typename T> T Trigintaduonion<T>::imag_u3() const {
	return u3;
}

template <typename T> T Trigintaduonion<T>::imag_i3() const {
	return i3;
}

template <typename T> T Trigintaduonion<T>::imag_j3() const {
	return j3;
}

template <typename T> T Trigintaduonion<T>::imag_k3() const {
	return k3;
}

template <typename T> T Trigintaduonion<T>::imag_u4() const {
	return u4;
}

template <typename T> T Trigintaduonion<T>::imag_i4() const {
	return i4;
}

template <typename T> T Trigintaduonion<T>::imag_j4() const {
	return j4;
}

template <typename T> T Trigintaduonion<T>::imag_k4() const {
	return k4;
}

template <typename T> T Trigintaduonion<T>::imag_u5() const {
	return u5;
}

template <typename T> T Trigintaduonion<T>::imag_i5() const {
	return i5;
}

template <typename T> T Trigintaduonion<T>::imag_j5() const {
	return j5;
}

template <typename T> T Trigintaduonion<T>::imag_k5() const {
	return k5;
}

template <typename T> T Trigintaduonion<T>::imag_u6() const {
	return u6;
}

template <typename T> T Trigintaduonion<T>::imag_i6() const {
	return i6;
}

template <typename T> T Trigintaduonion<T>::imag_j6() const {
	return j6;
}

template <typename T> T Trigintaduonion<T>::imag_k6() const {
	return k6;
}

template <typename T> T Trigintaduonion<T>::imag_u7() const {
	return u7;
}

template <typename T> T Trigintaduonion<T>::imag_i7() const {
	return i7;
}

template <typename T> T Trigintaduonion<T>::imag_j7() const {
	return j7;
}

template <typename T> T Trigintaduonion<T>::imag_k7() const {
	return k7;
}

template <typename T> T Trigintaduonion<T>::csgn() const {
	return is_zero() ? 0.0 : Support<T>::sign( r );
}

template <typename T> T Trigintaduonion<T>::abs() const {
	return is_inf()? Support<T>::POS_INF : std::sqrt(
		 r*r  +  i*i  +  j*j  +  k*k  + u1*u1 + i1*i1 + j1*j1 + k1*k1 +
		u2*u2 + i2*i2 + j2*j2 + k2*k2 + u3*u3 + i3*i3 + j3*j3 + k3*k3 +
		u4*u4 + i4*i4 + j4*j4 + k4*k4 + u5*u5 + i5*i5 + j5*j5 + k5*k5 +
		u6*u6 + i6*i6 + j6*j6 + k6*k6 + u7*u7 + i7*i7 + j7*j7 + k7*k7
	);
}

template <typename T> T Trigintaduonion<T>::norm() const {
	return is_inf() ? Support<T>::POS_INF :
		 r*r  +  i*i  +  j*j  +  k*k  + u1*u1 + i1*i1 + j1*j1 + k1*k1 +
		u2*u2 + i2*i2 + j2*j2 + k2*k2 + u3*u3 + i3*i3 + j3*j3 + k3*k3 +
		u4*u4 + i4*i4 + j4*j4 + k4*k4 + u5*u5 + i5*i5 + j5*j5 + k5*k5 +
		u6*u6 + i6*i6 + j6*j6 + k6*k6 + u7*u7 + i7*i7 + j7*j7 + k7*k7;
}

template <typename T> T Trigintaduonion<T>::abs_imag() const {
	return is_inf() ? Support<T>::POS_INF : std::sqrt(
		         i*i  +  j*j  +  k*k  + u1*u1 + i1*i1 + j1*j1 + k1*k1 +
		u2*u2 + i2*i2 + j2*j2 + k2*k2 + u3*u3 + i3*i3 + j3*j3 + k3*k3 +
		u4*u4 + i4*i4 + j4*j4 + k4*k4 + u5*u5 + i5*i5 + j5*j5 + k5*k5 +
		u6*u6 + i6*i6 + j6*j6 + k6*k6 + u7*u7 + i7*i7 + j7*j7 + k7*k7
	);
}

template <typename T> T Trigintaduonion<T>::norm_imag() const {
	return is_inf() ? Support<T>::POS_INF :
		i*i + j*j + k*k + u1*u1 + i1*i1 + j1*j1 + k1*k1 +
		u2*u2 + i2*i2 + j2*j2 + k2*k2 + u3*u3 + i3*i3 + j3*j3 + k3*k3 +
		u4*u4 + i4*i4 + j4*j4 + k4*k4 + u5*u5 + i5*i5 + j5*j5 + k5*k5 +
		u6*u6 + i6*i6 + j6*j6 + k6*k6 + u7*u7 + i7*i7 + j7*j7 + k7*k7;
}

template <typename T> T Trigintaduonion<T>::arg() const {
	return std::atan2( abs_imag(), r );
}

/******************************************
 * Trigintaduonion<T>-valued Functions
 ******************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::imag() const {
	return Trigintaduonion<T>( 0.0, i, j, k, u1, i1, j1, k1, u2, i2, j2, k2, u3, i3, j3, k3, u4, i4, j4, k4, u5, i5, j5, k5, u6, i6, j6, k6, u7, i7, j7, k7 );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::conj() const {
	return Trigintaduonion<T>( r, -i, -j, -k, -u1, -i1, -j1, -k1, -u2, -i2, -j2, -k2, -u3, -i3, -j3, -k3, -u4, -i4, -j4, -k4, -u5, -i5, -j5, -k5, -u6, -i6, -j6, -k6, -u7, -i7, -j7, -k7 );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator * () const {
	return Trigintaduonion<T>( r, -i, -j, -k, -u1, -i1, -j1, -k1, -u2, -i2, -j2, -k2, -u3, -i3, -j3, -k3, -u4, -i4, -j4, -k4, -u5, -i5, -j5, -k5, -u6, -i6, -j6, -k6, -u7, -i7, -j7, -k7 );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::signum() const {
	T absq = abs();

	if ( absq == 0.0 || Support<T>::is_nan( absq ) || Support<T>::is_inf( absq ) ) {
		return *this;
	} else {
		return Trigintaduonion<T>(
			 r/absq,  i/absq,  j/absq,  k/absq,
			u1/absq, i1/absq, j1/absq, k1/absq,
			u2/absq, i2/absq, j2/absq, k2/absq,
			u3/absq, i3/absq, j3/absq, k3/absq,
			u4/absq, i4/absq, j4/absq, k4/absq,
			u5/absq, i5/absq, j5/absq, k5/absq,
			u6/absq, i6/absq, j6/absq, k6/absq,
			u7/absq, i7/absq, j7/absq, k7/absq
		);
	}
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::sqr() const {
	return Trigintaduonion<T>(
		r*r - i*i - j*j - k*k - u1*u1 - i1*i1 - j1*j1 - k1*k1 - u2*u2 - i2*i2 - j2*j2 - k2*k2 - u3*u3 - i3*i3 - j3*j3 - k3*k3 - u4*u4 - i4*i4 - j4*j4 - k4*k4 - u5*u5 - i5*i5 - j5*j5 - k5*k5 - u6*u6 - i6*i6 - j6*j6 - k6*k6 - u7*u7 - i7*i7 - j7*j7 - k7*k7,
		2*r*i, 2*r*j, 2*r*k,
		2*r*u1, 2*r*i1, 2*r*j1, 2*r*k1,
		2*r*u2, 2*r*i2, 2*r*j2, 2*r*k2,
		2*r*u3, 2*r*i3, 2*r*j3, 2*r*k3,
		2*r*u4, 2*r*i4, 2*r*j4, 2*r*k4,
		2*r*u5, 2*r*i5, 2*r*j5, 2*r*k5,
		2*r*u6, 2*r*i6, 2*r*j6, 2*r*k6,
		2*r*u7, 2*r*i7, 2*r*j7, 2*r*k7
	);
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::sqrt() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).sqrt();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::rotate( const Trigintaduonion<T> & q ) const {
	// the assumption is that |q| = 1
	// in this case, q.inverse() == q.conj()

	T rr = q.r*q.r;
	T ii = q.i*q.i;
	T jj = q.j*q.j;
	T kk = q.k*q.k;
	T u1u1 = q.u1*q.u1;
	T i1i1 = q.i1*q.i1;
	T j1j1 = q.j1*q.j1;
	T k1k1 = q.k1*q.k1;
	T u2u2 = q.u2*q.u2;
	T i2i2 = q.i2*q.i2;
	T j2j2 = q.j2*q.j2;
	T k2k2 = q.k2*q.k2;
	T u3u3 = q.u3*q.u3;
	T i3i3 = q.i3*q.i3;
	T j3j3 = q.j3*q.j3;
	T k3k3 = q.k3*q.k3;
	T u4u4 = q.u4*q.u4;
	T i4i4 = q.i4*q.i4;
	T j4j4 = q.j4*q.j4;
	T k4k4 = q.k4*q.k4;
	T u5u5 = q.u5*q.u5;
	T i5i5 = q.i5*q.i5;
	T j5j5 = q.j5*q.j5;
	T k5k5 = q.k5*q.k5;
	T u6u6 = q.u6*q.u6;
	T i6i6 = q.i6*q.i6;
	T j6j6 = q.j6*q.j6;
	T k6k6 = q.k6*q.k6;
	T u7u7 = q.u7*q.u7;
	T i7i7 = q.i7*q.i7;
	T j7j7 = q.j7*q.j7;
	T k7k7 = q.k7*q.k7;

	T u1qi = u1*q.i;
	T u1qi2 = u1*q.i2;
	T u1qi3 = u1*q.i3;
	T u1qi4 = u1*q.i4;
	T u1qi5 = u1*q.i5;
	T u1qi6 = u1*q.i6;
	T u1qi7 = u1*q.i7;
	T u1qj = u1*q.j;
	T u1qj2 = u1*q.j2;
	T u1qj3 = u1*q.j3;
	T u1qj4 = u1*q.j4;
	T u1qj5 = u1*q.j5;
	T u1qj6 = u1*q.j6;
	T u1qj7 = u1*q.j7;
	T u1qk = u1*q.k;
	T u1qk2 = u1*q.k2;
	T u1qk3 = u1*q.k3;
	T u1qk4 = u1*q.k4;
	T u1qk5 = u1*q.k5;
	T u1qk6 = u1*q.k6;
	T u1qk7 = u1*q.k7;
	T u1qu1 = u1*q.u1;
	T u1qu6 = u1*q.u6;
	T u1qu7 = u1*q.u7;
	T u2qi = u2*q.i;
	T u2qi4 = u2*q.i4;
	T u2qi5 = u2*q.i5;
	T u2qi6 = u2*q.i6;
	T u2qi7 = u2*q.i7;
	T u2qj = u2*q.j;
	T u2qj4 = u2*q.j4;
	T u2qj5 = u2*q.j5;
	T u2qj6 = u2*q.j6;
	T u2qj7 = u2*q.j7;
	T u2qk = u2*q.k;
	T u2qk4 = u2*q.k4;
	T u2qk5 = u2*q.k5;
	T u2qk6 = u2*q.k6;
	T u2qk7 = u2*q.k7;
	T u2qu2 = u2*q.u2;
	T u2qu5 = u2*q.u5;
	T u2qu7 = u2*q.u7;
	T u3qi = u3*q.i;
	T u3qi1 = u3*q.i1;
	T u3qi4 = u3*q.i4;
	T u3qi5 = u3*q.i5;
	T u3qi6 = u3*q.i6;
	T u3qi7 = u3*q.i7;
	T u3qj = u3*q.j;
	T u3qj1 = u3*q.j1;
	T u3qj4 = u3*q.j4;
	T u3qj5 = u3*q.j5;
	T u3qj6 = u3*q.j6;
	T u3qj7 = u3*q.j7;
	T u3qk = u3*q.k;
	T u3qk1 = u3*q.k1;
	T u3qk4 = u3*q.k4;
	T u3qk5 = u3*q.k5;
	T u3qk6 = u3*q.k6;
	T u3qk7 = u3*q.k7;
	T u3qu3 = u3*q.u3;
	T u3qu5 = u3*q.u5;
	T u3qu6 = u3*q.u6;
	T u4qu4 = u4*q.u4;
	T u5qi = u5*q.i;
	T u5qi1 = u5*q.i1;
	T u5qi2 = u5*q.i2;
	T u5qi3 = u5*q.i3;
	T u5qi6 = u5*q.i6;
	T u5qi7 = u5*q.i7;
	T u5qj = u5*q.j;
	T u5qj1 = u5*q.j1;
	T u5qj2 = u5*q.j2;
	T u5qj3 = u5*q.j3;
	T u5qj6 = u5*q.j6;
	T u5qj7 = u5*q.j7;
	T u5qk = u5*q.k;
	T u5qk1 = u5*q.k1;
	T u5qk2 = u5*q.k2;
	T u5qk3 = u5*q.k3;
	T u5qk6 = u5*q.k6;
	T u5qk7 = u5*q.k7;
	T u5qu2 = u5*q.u2;
	T u5qu3 = u5*q.u3;
	T u5qu5 = u5*q.u5;
	T u6qi = u6*q.i;
	T u6qi1 = u6*q.i1;
	T u6qi2 = u6*q.i2;
	T u6qi3 = u6*q.i3;
	T u6qj = u6*q.j;
	T u6qj1 = u6*q.j1;
	T u6qj2 = u6*q.j2;
	T u6qj3 = u6*q.j3;
	T u6qk = u6*q.k;
	T u6qk1 = u6*q.k1;
	T u6qk2 = u6*q.k2;
	T u6qk3 = u6*q.k3;
	T u6qu1 = u6*q.u1;
	T u6qu3 = u6*q.u3;
	T u6qu6 = u6*q.u6;
	T u7qi = u7*q.i;
	T u7qi1 = u7*q.i1;
	T u7qi2 = u7*q.i2;
	T u7qi3 = u7*q.i3;
	T u7qi4 = u7*q.i4;
	T u7qi5 = u7*q.i5;
	T u7qj = u7*q.j;
	T u7qj1 = u7*q.j1;
	T u7qj2 = u7*q.j2;
	T u7qj3 = u7*q.j3;
	T u7qj4 = u7*q.j4;
	T u7qj5 = u7*q.j5;
	T u7qk = u7*q.k;
	T u7qk1 = u7*q.k1;
	T u7qk2 = u7*q.k2;
	T u7qk3 = u7*q.k3;
	T u7qk4 = u7*q.k4;
	T u7qk5 = u7*q.k5;
	T u7qu1 = u7*q.u1;
	T u7qu2 = u7*q.u2;
	T u7qu7 = u7*q.u7;
	T iqi = i*q.i;
	T iqi3 = i*q.i3;
	T iqi5 = i*q.i5;
	T iqi6 = i*q.i6;
	T iqi7 = i*q.i7;
	T iqj = i*q.j;
	T iqj2 = i*q.j2;
	T iqj3 = i*q.j3;
	T iqj4 = i*q.j4;
	T iqj5 = i*q.j5;
	T iqj6 = i*q.j6;
	T iqj7 = i*q.j7;
	T iqk = i*q.k;
	T iqk2 = i*q.k2;
	T iqk3 = i*q.k3;
	T iqk4 = i*q.k4;
	T iqk5 = i*q.k5;
	T iqk6 = i*q.k6;
	T iqk7 = i*q.k7;
	T iqu3 = i*q.u3;
	T iqu5 = i*q.u5;
	T iqu6 = i*q.u6;
	T iqu7 = i*q.u7;
	T i1qi = i1*q.i;
	T i1qi1 = i1*q.i1;
	T i1qi2 = i1*q.i2;
	T i1qi4 = i1*q.i4;
	T i1qi6 = i1*q.i6;
	T i1qi7 = i1*q.i7;
	T i1qj = i1*q.j;
	T i1qj2 = i1*q.j2;
	T i1qj3 = i1*q.j3;
	T i1qj4 = i1*q.j4;
	T i1qj5 = i1*q.j5;
	T i1qj6 = i1*q.j6;
	T i1qj7 = i1*q.j7;
	T i1qk = i1*q.k;
	T i1qk2 = i1*q.k2;
	T i1qk3 = i1*q.k3;
	T i1qk4 = i1*q.k4;
	T i1qk5 = i1*q.k5;
	T i1qk6 = i1*q.k6;
	T i1qk7 = i1*q.k7;
	T i1qu3 = i1*q.u3;
	T i1qu5 = i1*q.u5;
	T i1qu6 = i1*q.u6;
	T i1qu7 = i1*q.u7;
	T i2qi = i2*q.i;
	T i2qi1 = i2*q.i1;
	T i2qi2 = i2*q.i2;
	T i2qi4 = i2*q.i4;
	T i2qi5 = i2*q.i5;
	T i2qi7 = i2*q.i7;
	T i2qj = i2*q.j;
	T i2qj1 = i2*q.j1;
	T i2qj4 = i2*q.j4;
	T i2qj5 = i2*q.j5;
	T i2qj6 = i2*q.j6;
	T i2qj7 = i2*q.j7;
	T i2qk = i2*q.k;
	T i2qk1 = i2*q.k1;
	T i2qk4 = i2*q.k4;
	T i2qk5 = i2*q.k5;
	T i2qk6 = i2*q.k6;
	T i2qk7 = i2*q.k7;
	T i2qu1 = i2*q.u1;
	T i2qu5 = i2*q.u5;
	T i2qu6 = i2*q.u6;
	T i2qu7 = i2*q.u7;
	T i3qi = i3*q.i;
	T i3qi3 = i3*q.i3;
	T i3qi4 = i3*q.i4;
	T i3qi5 = i3*q.i5;
	T i3qi6 = i3*q.i6;
	T i3qj = i3*q.j;
	T i3qj1 = i3*q.j1;
	T i3qj4 = i3*q.j4;
	T i3qj5 = i3*q.j5;
	T i3qj6 = i3*q.j6;
	T i3qj7 = i3*q.j7;
	T i3qk = i3*q.k;
	T i3qk1 = i3*q.k1;
	T i3qk4 = i3*q.k4;
	T i3qk5 = i3*q.k5;
	T i3qk6 = i3*q.k6;
	T i3qk7 = i3*q.k7;
	T i3qu1 = i3*q.u1;
	T i3qu5 = i3*q.u5;
	T i3qu6 = i3*q.u6;
	T i3qu7 = i3*q.u7;
	T i4qi1 = i4*q.i1;
	T i4qi2 = i4*q.i2;
	T i4qi3 = i4*q.i3;
	T i4qi4 = i4*q.i4;
	T i4qi7 = i4*q.i7;
	T i4qj = i4*q.j;
	T i4qj1 = i4*q.j1;
	T i4qj2 = i4*q.j2;
	T i4qj3 = i4*q.j3;
	T i4qj6 = i4*q.j6;
	T i4qj7 = i4*q.j7;
	T i4qk = i4*q.k;
	T i4qk1 = i4*q.k1;
	T i4qk2 = i4*q.k2;
	T i4qk3 = i4*q.k3;
	T i4qk6 = i4*q.k6;
	T i4qk7 = i4*q.k7;
	T i4qu1 = i4*q.u1;
	T i4qu2 = i4*q.u2;
	T i4qu3 = i4*q.u3;
	T i4qu7 = i4*q.u7;
	T i5qi = i5*q.i;
	T i5qi2 = i5*q.i2;
	T i5qi3 = i5*q.i3;
	T i5qi5 = i5*q.i5;
	T i5qi6 = i5*q.i6;
	T i5qj = i5*q.j;
	T i5qj1 = i5*q.j1;
	T i5qj2 = i5*q.j2;
	T i5qj3 = i5*q.j3;
	T i5qj6 = i5*q.j6;
	T i5qj7 = i5*q.j7;
	T i5qk = i5*q.k;
	T i5qk1 = i5*q.k1;
	T i5qk2 = i5*q.k2;
	T i5qk3 = i5*q.k3;
	T i5qk6 = i5*q.k6;
	T i5qk7 = i5*q.k7;
	T i5qu1 = i5*q.u1;
	T i5qu2 = i5*q.u2;
	T i5qu3 = i5*q.u3;
	T i5qu7 = i5*q.u7;
	T i6qi = i6*q.i;
	T i6qi1 = i6*q.i1;
	T i6qi3 = i6*q.i3;
	T i6qi5 = i6*q.i5;
	T i6qi6 = i6*q.i6;
	T i6qj = i6*q.j;
	T i6qj1 = i6*q.j1;
	T i6qj2 = i6*q.j2;
	T i6qj3 = i6*q.j3;
	T i6qj4 = i6*q.j4;
	T i6qj5 = i6*q.j5;
	T i6qk = i6*q.k;
	T i6qk1 = i6*q.k1;
	T i6qk2 = i6*q.k2;
	T i6qk3 = i6*q.k3;
	T i6qk4 = i6*q.k4;
	T i6qk5 = i6*q.k5;
	T i6qu1 = i6*q.u1;
	T i6qu2 = i6*q.u2;
	T i6qu3 = i6*q.u3;
	T i6qu5 = i6*q.u5;
	T i7qi = i7*q.i;
	T i7qi1 = i7*q.i1;
	T i7qi2 = i7*q.i2;
	T i7qi4 = i7*q.i4;
	T i7qi7 = i7*q.i7;
	T i7qj = i7*q.j;
	T i7qj1 = i7*q.j1;
	T i7qj2 = i7*q.j2;
	T i7qj3 = i7*q.j3;
	T i7qj4 = i7*q.j4;
	T i7qj5 = i7*q.j5;
	T i7qk = i7*q.k;
	T i7qk1 = i7*q.k1;
	T i7qk2 = i7*q.k2;
	T i7qk3 = i7*q.k3;
	T i7qk4 = i7*q.k4;
	T i7qk5 = i7*q.k5;
	T i7qu1 = i7*q.u1;
	T i7qu2 = i7*q.u2;
	T i7qu3 = i7*q.u3;
	T i7qu5 = i7*q.u5;
	T jqi = j*q.i;
	T jqi2 = j*q.i2;
	T jqi3 = j*q.i3;
	T jqi4 = j*q.i4;
	T jqi5 = j*q.i5;
	T jqi6 = j*q.i6;
	T jqi7 = j*q.i7;
	T jqj = j*q.j;
	T jqj3 = j*q.j3;
	T jqj5 = j*q.j5;
	T jqj6 = j*q.j6;
	T jqj7 = j*q.j7;
	T jqk = j*q.k;
	T jqk2 = j*q.k2;
	T jqk3 = j*q.k3;
	T jqk4 = j*q.k4;
	T jqk5 = j*q.k5;
	T jqk6 = j*q.k6;
	T jqk7 = j*q.k7;
	T jqu3 = j*q.u3;
	T jqu5 = j*q.u5;
	T jqu6 = j*q.u6;
	T jqu7 = j*q.u7;
	T j1qi = j1*q.i;
	T j1qi2 = j1*q.i2;
	T j1qi3 = j1*q.i3;
	T j1qi4 = j1*q.i4;
	T j1qi5 = j1*q.i5;
	T j1qi6 = j1*q.i6;
	T j1qi7 = j1*q.i7;
	T j1qj = j1*q.j;
	T j1qj1 = j1*q.j1;
	T j1qj2 = j1*q.j2;
	T j1qj4 = j1*q.j4;
	T j1qj6 = j1*q.j6;
	T j1qj7 = j1*q.j7;
	T j1qk = j1*q.k;
	T j1qk2 = j1*q.k2;
	T j1qk3 = j1*q.k3;
	T j1qk4 = j1*q.k4;
	T j1qk5 = j1*q.k5;
	T j1qk6 = j1*q.k6;
	T j1qk7 = j1*q.k7;
	T j1qu3 = j1*q.u3;
	T j1qu5 = j1*q.u5;
	T j1qu6 = j1*q.u6;
	T j1qu7 = j1*q.u7;
	T j2qi = j2*q.i;
	T j2qi1 = j2*q.i1;
	T j2qi4 = j2*q.i4;
	T j2qi5 = j2*q.i5;
	T j2qi6 = j2*q.i6;
	T j2qi7 = j2*q.i7;
	T j2qj = j2*q.j;
	T j2qj1 = j2*q.j1;
	T j2qj2 = j2*q.j2;
	T j2qj4 = j2*q.j4;
	T j2qj5 = j2*q.j5;
	T j2qj7 = j2*q.j7;
	T j2qk = j2*q.k;
	T j2qk1 = j2*q.k1;
	T j2qk4 = j2*q.k4;
	T j2qk5 = j2*q.k5;
	T j2qk6 = j2*q.k6;
	T j2qk7 = j2*q.k7;
	T j2qu1 = j2*q.u1;
	T j2qu5 = j2*q.u5;
	T j2qu6 = j2*q.u6;
	T j2qu7 = j2*q.u7;
	T j3qi = j3*q.i;
	T j3qi1 = j3*q.i1;
	T j3qi4 = j3*q.i4;
	T j3qi5 = j3*q.i5;
	T j3qi6 = j3*q.i6;
	T j3qi7 = j3*q.i7;
	T j3qj = j3*q.j;
	T j3qj3 = j3*q.j3;
	T j3qj4 = j3*q.j4;
	T j3qj5 = j3*q.j5;
	T j3qj6 = j3*q.j6;
	T j3qk = j3*q.k;
	T j3qk1 = j3*q.k1;
	T j3qk4 = j3*q.k4;
	T j3qk5 = j3*q.k5;
	T j3qk6 = j3*q.k6;
	T j3qk7 = j3*q.k7;
	T j3qu1 = j3*q.u1;
	T j3qu5 = j3*q.u5;
	T j3qu6 = j3*q.u6;
	T j3qu7 = j3*q.u7;
	T j4qi = j4*q.i;
	T j4qi1 = j4*q.i1;
	T j4qi2 = j4*q.i2;
	T j4qi3 = j4*q.i3;
	T j4qi6 = j4*q.i6;
	T j4qi7 = j4*q.i7;
	T j4qj1 = j4*q.j1;
	T j4qj2 = j4*q.j2;
	T j4qj3 = j4*q.j3;
	T j4qj4 = j4*q.j4;
	T j4qj7 = j4*q.j7;
	T j4qk = j4*q.k;
	T j4qk1 = j4*q.k1;
	T j4qk2 = j4*q.k2;
	T j4qk3 = j4*q.k3;
	T j4qk6 = j4*q.k6;
	T j4qk7 = j4*q.k7;
	T j4qu1 = j4*q.u1;
	T j4qu2 = j4*q.u2;
	T j4qu3 = j4*q.u3;
	T j4qu7 = j4*q.u7;
	T j5qi = j5*q.i;
	T j5qi1 = j5*q.i1;
	T j5qi2 = j5*q.i2;
	T j5qi3 = j5*q.i3;
	T j5qi6 = j5*q.i6;
	T j5qi7 = j5*q.i7;
	T j5qj = j5*q.j;
	T j5qj2 = j5*q.j2;
	T j5qj3 = j5*q.j3;
	T j5qj5 = j5*q.j5;
	T j5qj6 = j5*q.j6;
	T j5qk = j5*q.k;
	T j5qk1 = j5*q.k1;
	T j5qk2 = j5*q.k2;
	T j5qk3 = j5*q.k3;
	T j5qk6 = j5*q.k6;
	T j5qk7 = j5*q.k7;
	T j5qu1 = j5*q.u1;
	T j5qu2 = j5*q.u2;
	T j5qu3 = j5*q.u3;
	T j5qu7 = j5*q.u7;
	T j6qi = j6*q.i;
	T j6qi1 = j6*q.i1;
	T j6qi2 = j6*q.i2;
	T j6qi3 = j6*q.i3;
	T j6qi4 = j6*q.i4;
	T j6qi5 = j6*q.i5;
	T j6qj = j6*q.j;
	T j6qj1 = j6*q.j1;
	T j6qj3 = j6*q.j3;
	T j6qj5 = j6*q.j5;
	T j6qj6 = j6*q.j6;
	T j6qk = j6*q.k;
	T j6qk1 = j6*q.k1;
	T j6qk2 = j6*q.k2;
	T j6qk3 = j6*q.k3;
	T j6qk4 = j6*q.k4;
	T j6qk5 = j6*q.k5;
	T j6qu1 = j6*q.u1;
	T j6qu2 = j6*q.u2;
	T j6qu3 = j6*q.u3;
	T j6qu5 = j6*q.u5;
	T j7qi = j7*q.i;
	T j7qi1 = j7*q.i1;
	T j7qi2 = j7*q.i2;
	T j7qi3 = j7*q.i3;
	T j7qi4 = j7*q.i4;
	T j7qi5 = j7*q.i5;
	T j7qj = j7*q.j;
	T j7qj1 = j7*q.j1;
	T j7qj2 = j7*q.j2;
	T j7qj4 = j7*q.j4;
	T j7qj7 = j7*q.j7;
	T j7qk = j7*q.k;
	T j7qk1 = j7*q.k1;
	T j7qk2 = j7*q.k2;
	T j7qk3 = j7*q.k3;
	T j7qk4 = j7*q.k4;
	T j7qk5 = j7*q.k5;
	T j7qu1 = j7*q.u1;
	T j7qu2 = j7*q.u2;
	T j7qu3 = j7*q.u3;
	T j7qu5 = j7*q.u5;
	T kqi = k*q.i;
	T kqi2 = k*q.i2;
	T kqi3 = k*q.i3;
	T kqi4 = k*q.i4;
	T kqi5 = k*q.i5;
	T kqi6 = k*q.i6;
	T kqi7 = k*q.i7;
	T kqj = k*q.j;
	T kqj2 = k*q.j2;
	T kqj3 = k*q.j3;
	T kqj4 = k*q.j4;
	T kqj5 = k*q.j5;
	T kqj6 = k*q.j6;
	T kqj7 = k*q.j7;
	T kqk = k*q.k;
	T kqk3 = k*q.k3;
	T kqk5 = k*q.k5;
	T kqk6 = k*q.k6;
	T kqk7 = k*q.k7;
	T kqu3 = k*q.u3;
	T kqu5 = k*q.u5;
	T kqu6 = k*q.u6;
	T kqu7 = k*q.u7;
	T k1qi = k1*q.i;
	T k1qi2 = k1*q.i2;
	T k1qi3 = k1*q.i3;
	T k1qi4 = k1*q.i4;
	T k1qi5 = k1*q.i5;
	T k1qi6 = k1*q.i6;
	T k1qi7 = k1*q.i7;
	T k1qj = k1*q.j;
	T k1qj2 = k1*q.j2;
	T k1qj3 = k1*q.j3;
	T k1qj4 = k1*q.j4;
	T k1qj5 = k1*q.j5;
	T k1qj6 = k1*q.j6;
	T k1qj7 = k1*q.j7;
	T k1qk = k1*q.k;
	T k1qk1 = k1*q.k1;
	T k1qk2 = k1*q.k2;
	T k1qk4 = k1*q.k4;
	T k1qk6 = k1*q.k6;
	T k1qk7 = k1*q.k7;
	T k1qu3 = k1*q.u3;
	T k1qu5 = k1*q.u5;
	T k1qu6 = k1*q.u6;
	T k1qu7 = k1*q.u7;
	T k2qi = k2*q.i;
	T k2qi1 = k2*q.i1;
	T k2qi4 = k2*q.i4;
	T k2qi5 = k2*q.i5;
	T k2qi6 = k2*q.i6;
	T k2qi7 = k2*q.i7;
	T k2qj = k2*q.j;
	T k2qj1 = k2*q.j1;
	T k2qj4 = k2*q.j4;
	T k2qj5 = k2*q.j5;
	T k2qj6 = k2*q.j6;
	T k2qj7 = k2*q.j7;
	T k2qk = k2*q.k;
	T k2qk1 = k2*q.k1;
	T k2qk2 = k2*q.k2;
	T k2qk4 = k2*q.k4;
	T k2qk5 = k2*q.k5;
	T k2qk7 = k2*q.k7;
	T k2qu1 = k2*q.u1;
	T k2qu5 = k2*q.u5;
	T k2qu6 = k2*q.u6;
	T k2qu7 = k2*q.u7;
	T k3qi = k3*q.i;
	T k3qi1 = k3*q.i1;
	T k3qi4 = k3*q.i4;
	T k3qi5 = k3*q.i5;
	T k3qi6 = k3*q.i6;
	T k3qi7 = k3*q.i7;
	T k3qj = k3*q.j;
	T k3qj1 = k3*q.j1;
	T k3qj4 = k3*q.j4;
	T k3qj5 = k3*q.j5;
	T k3qj6 = k3*q.j6;
	T k3qj7 = k3*q.j7;
	T k3qk = k3*q.k;
	T k3qk3 = k3*q.k3;
	T k3qk4 = k3*q.k4;
	T k3qk5 = k3*q.k5;
	T k3qk6 = k3*q.k6;
	T k3qu1 = k3*q.u1;
	T k3qu5 = k3*q.u5;
	T k3qu6 = k3*q.u6;
	T k3qu7 = k3*q.u7;
	T k4qi = k4*q.i;
	T k4qi1 = k4*q.i1;
	T k4qi2 = k4*q.i2;
	T k4qi3 = k4*q.i3;
	T k4qi6 = k4*q.i6;
	T k4qi7 = k4*q.i7;
	T k4qj = k4*q.j;
	T k4qj1 = k4*q.j1;
	T k4qj2 = k4*q.j2;
	T k4qj3 = k4*q.j3;
	T k4qj6 = k4*q.j6;
	T k4qj7 = k4*q.j7;
	T k4qk1 = k4*q.k1;
	T k4qk2 = k4*q.k2;
	T k4qk3 = k4*q.k3;
	T k4qk4 = k4*q.k4;
	T k4qk7 = k4*q.k7;
	T k4qu1 = k4*q.u1;
	T k4qu2 = k4*q.u2;
	T k4qu3 = k4*q.u3;
	T k4qu7 = k4*q.u7;
	T k5qi = k5*q.i;
	T k5qi1 = k5*q.i1;
	T k5qi2 = k5*q.i2;
	T k5qi3 = k5*q.i3;
	T k5qi6 = k5*q.i6;
	T k5qi7 = k5*q.i7;
	T k5qj = k5*q.j;
	T k5qj1 = k5*q.j1;
	T k5qj2 = k5*q.j2;
	T k5qj3 = k5*q.j3;
	T k5qj6 = k5*q.j6;
	T k5qj7 = k5*q.j7;
	T k5qk = k5*q.k;
	T k5qk2 = k5*q.k2;
	T k5qk3 = k5*q.k3;
	T k5qk5 = k5*q.k5;
	T k5qk6 = k5*q.k6;
	T k5qu1 = k5*q.u1;
	T k5qu2 = k5*q.u2;
	T k5qu3 = k5*q.u3;
	T k5qu7 = k5*q.u7;
	T k6qi = k6*q.i;
	T k6qi1 = k6*q.i1;
	T k6qi2 = k6*q.i2;
	T k6qi3 = k6*q.i3;
	T k6qi4 = k6*q.i4;
	T k6qi5 = k6*q.i5;
	T k6qj = k6*q.j;
	T k6qj1 = k6*q.j1;
	T k6qj2 = k6*q.j2;
	T k6qj3 = k6*q.j3;
	T k6qj4 = k6*q.j4;
	T k6qj5 = k6*q.j5;
	T k6qk = k6*q.k;
	T k6qk1 = k6*q.k1;
	T k6qk3 = k6*q.k3;
	T k6qk5 = k6*q.k5;
	T k6qk6 = k6*q.k6;
	T k6qu1 = k6*q.u1;
	T k6qu2 = k6*q.u2;
	T k6qu3 = k6*q.u3;
	T k6qu5 = k6*q.u5;
	T k7qi = k7*q.i;
	T k7qi1 = k7*q.i1;
	T k7qi2 = k7*q.i2;
	T k7qi3 = k7*q.i3;
	T k7qi4 = k7*q.i4;
	T k7qi5 = k7*q.i5;
	T k7qj = k7*q.j;
	T k7qj1 = k7*q.j1;
	T k7qj2 = k7*q.j2;
	T k7qj3 = k7*q.j3;
	T k7qj4 = k7*q.j4;
	T k7qj5 = k7*q.j5;
	T k7qk = k7*q.k;
	T k7qk1 = k7*q.k1;
	T k7qk2 = k7*q.k2;
	T k7qk4 = k7*q.k4;
	T k7qk7 = k7*q.k7;
	T k7qu1 = k7*q.u1;
	T k7qu2 = k7*q.u2;
	T k7qu3 = k7*q.u3;
	T k7qu5 = k7*q.u5;

	T sum = rr-ii-jj-kk-u1u1-i1i1-j1j1-k1k1-u2u2-i2i2-j2j2-k2k2-u3u3-i3i3-j3j3-k3k3-u4u4-i4i4-j4j4-k4k4-u5u5-i5i5-j5j5-k5k5-u6u6-i6i6-j6j6-k6k6-u7u7-i7i7-j7j7-k7k7;

	return Trigintaduonion<T>(
		// Real
		( ( r == 0 ) ? r : r*(rr+ii+jj+kk+u1u1+i1i1+j1j1+k1k1+u2u2+i2i2+j2j2+k2k2+u3u3+i3i3+j3j3+k3k3+u4u4+i4i4+j4j4+k4k4+u5u5+i5i5+j5j5+k5k5+u6u6+i6i6+j6j6+k6k6+u7u7+i7i7+j7j7+k7k7) ),
		// I
		(sum + 2*ii)*i+2*(
			(jqj+kqk+j1qj1+u1qu1+k1qk1+i1qi1+i2qi2+u2qu2+k3qk3+j3qj3+i3qi3+u3qu3+k2qk2+j2qj2+k4qk4+j4qj4+i4qi4+u4qu4+k6qk6+j6qj6+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+u5qu5+k7qk7+j7qj7+i7qi7+u7qu7)*q.i
			-(u2*q.i2-kqj-u3*q.i3+k1*q.j1+u4*q.i4-j2*q.k2-i2*q.u2+k2*q.j2+j3*q.k3-k3*q.j3-j4*q.k4+i5*q.u5+j5*q.k5-k5*q.j5+u7*q.i7+k7*q.j7-j7*q.k7-i7*q.u7+jqk+i6*q.u6-k6*q.j6+i3*q.u3-u6*q.i6+j6*q.k6+u1*q.i1-i1*q.u1-u5*q.i5+k4*q.j4-i4*q.u4-j1*q.k1)*q.r
			+(-i6qj1+u6qk1-k5qu2+j5qi2-i5qj2-u5qk2+k4qu3+j4qi3-i3qj4-u3qk4+k2qu5+j2qi5)*q.j7
			+(-u6qj1-i6qk1+j5qu2+k5qi2+u5qj2-i5qk2-j4qu3+k4qi3+u3qj4-i3qk4-j2qu5+k2qi5)*q.k7
			+(-i6qu1-u6qi1+i5qu2+u5qi2-k5qj2+j5qk2-k4qj3+j4qk3-k3qj4+j3qk4-k2qj5+j2qk5)*q.u7
			+(u6qu1-i6qi1-u5qu2+i5qi2+j5qj2+k5qk2-j4qj3-k4qk3+j3qj4+k3qk4-j2qj5-k2qk5)*q.i7
			+(-i6qj-u6qk+k4qu2+j4qi2+k5qu3-j5qi3+i5qj3-u5qk3+k3qu5-j3qi5+i3qj5-u3qk5)*q.j6
			+(u6qj-i6qk-j4qu2+k4qi2-j5qu3-k5qi3+u5qj3+i5qk3-j3qu5-k3qi5+u3qj5+i3qk5)*q.k6
			+(-k6qj+j6qk-i7qu1+u7qi1+k7qj1-j7qk1-k4qj2+j4qk2-i5qu3+u5qi3+k5qj3-j5qk3)*q.u6
			+(j6qj+k6qk+u7qu1+i7qi1+j7qj1+k7qk1-j4qj2-k4qk2-u5qu3-i5qi3-j5qj3-k5qk3)*q.i6
			+(-i5qj-u5qk+k4qu1+j4qi1-k7qu2-j7qi2+i6qj3+u6qk3)*q.j5
			+(u5qj-i5qk-j4qu1+k4qi1+j7qu2-k7qi2-u6qj3+i6qk3)*q.k5
			+(-k5qj+j5qk-k4qj1+j4qk1+i7qu2-u7qi2+i6qu3-u6qi3)*q.u5
			+(j5qj+k5qk-j4qj1-k4qk1-u7qu2-i7qi2+u6qu3+i6qi3)*q.i5
			+(k5qu1-j5qi1+i5qj1-u5qk1+k6qu2-j6qi2+i6qj2-u6qk2)*q.j4
			+(-j5qu1-k5qi1+u5qj1+i5qk1-j6qu2-k6qi2+u6qj2+i6qk2)*q.k4
			+(-i3qj-u3qk+k2qu1+j2qi1)*q.j3
			+(u3qj+k2qi1-i3qk-j2qu1)*q.k3
			+(-k3qj+j3qk+j2qk1-k2qj1)*q.u3
			+(-j2qj1+j3qj+k3qk-k2qk1)*q.i3
			+(k3qu1+i3qj1-j3qi1-u3qk1)*q.j2
			-q.k2*(-i3qk1-u3qj1+j3qu1+k3qi1)
		),
		// J
		(sum + 2*jj)*j+2*(
			(i1qi1+i2qi2+k3qk3+j3qj3+i3qi3+u3qu3+k2qk2+j2qj2+u5qu5+k4qk4+j4qj4+i4qi4+u4qu4+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+k7qk7+j7qj7+i7qi7+u7qu7+k6qk6+j6qj6+j1qj1+k1qk1+kqk+iqi+u1qu1+u2qu2)*q.j
			-(-iqk+k6*q.i6+i1*q.k1-k1*q.i1+i2*q.k2-i6*q.k6-u3*q.j3-j2*q.u2+kqi-k2*q.i2+u1*q.j1-i3*q.k3+j3*q.u3-j1*q.u1+j6*q.u6+u2*q.j2+i4*q.k4-j4*q.u4-i5*q.k5+j5*q.u5+k5*q.i5-u6*q.j6+u7*q.j7-k7*q.i7+k3*q.i3+u4*q.j4-k4*q.i4-u5*q.j5+i7*q.k7-j7*q.u7)*q.r
			+(-j6qi1-u6qk1+k5qu2-j5qi2+i5qj2+u5qk2-k4qu3+i4qj3-j3qi4+u3qk4-k2qu5+i2qj5)*q.i7
			+(-j6qu1-u6qj1+j5qu2+k5qi2+u5qj2-i5qk2+k4qi3-i4qk3+k3qi4-i3qk4+k2qi5-i2qk5)*q.u7
			+(u6qi1-j6qk1-i5qu2-u5qi2+k5qj2-j5qk2+i4qu3+k4qj3-u3qi4-j3qk4+i2qu5+k2qj5)*q.k7
			+(u6qu1-j6qj1-u5qu2+i5qi2+j5qj2+k5qk2-i4qi3-k4qk3+i3qi4+k3qk4-i2qi5-k2qk5)*q.j7
			+(-j6qi+u6qk-k4qu2+i4qj2-k5qu3+j5qi3-i5qj3+u5qk3-k3qu5+j3qi5-i3qj5+u3qk5)*q.i6
			+(k6qi-i6qk-j7qu1-k7qi1+u7qj1+i7qk1+k4qi2-i4qk2-j5qu3-k5qi3+u5qj3+i5qk3)*q.u6
			+(-u6qi-j6qk+i4qu2+k4qj2+i5qu3-u5qi3-k5qj3+j5qk3+i3qu5-u3qi5-k3qj5+j3qk5)*q.k6
			+(i6qi+k6qk+u7qu1+i7qi1+j7qj1+k7qk1-i4qi2-k4qk2-u5qu3-i5qi3-j5qj3-k5qk3)*q.j6
			+(-j5qi+u5qk-k4qu1+i4qj1+k7qu2-i7qj2+j6qi3-u6qk3)*q.i5
			+(k5qi-i5qk+k4qi1-i4qk1+j7qu2-u7qj2+j6qu3-u6qj3)*q.u5
			+(-u5qi-j5qk+i4qu1+k4qj1-i7qu2-k7qj2+u6qi3+j6qk3)*q.k5
			+(i5qi+k5qk-i4qi1-k4qk1-u7qu2-j7qj2+u6qu3+j6qj3)*q.j5
			+(-k5qu1+j5qi1-i5qj1+u5qk1-k6qu2+j6qi2-i6qj2+u6qk2)*q.i4
			+(i5qu1-u5qi1-k5qj1+j5qk1+i6qu2-u6qi2-k6qj2+j6qk2)*q.k4
			+(-j3qi+u3qk-k2qu1+i2qj1)*q.i3
			+(k3qi-i3qk-i2qk1+k2qi1)*q.u3
			+(-u3qi-j3qk+i2qu1+k2qj1)*q.k3
			+(i3qi-k2qk1+k3qk-i2qi1)*q.j3
			+(j3qi1-i3qj1+u3qk1-k3qu1)*q.i2
			+q.k2*(j3qk1-u3qi1-k3qj1+i3qu1)
		),
		// K
		(sum + 2*kk)*k+2*(
			(k1qk1+iqi+j1qj1+u1qu1+jqj+u3qu3+k2qk2+j2qj2+i2qi2+u2qu2+i1qi1+j3qj3+i3qi3+i4qi4+u4qu4+k3qk3+k4qk4+j4qj4+i5qi5+u5qu5+k5qk5+j5qj5+k6qk6+j6qj6+i6qi6+u6qu6+j7qj7+i7qi7+u7qu7+k7qk7)*q.k
			-(-j3*q.i3+k3*q.u3+u4*q.k4+i6*q.j6-u6*q.k6-k1*q.u1-u3*q.k3+i3*q.j3-j6*q.i6-j5*q.i5+j4*q.i4-k4*q.u4-u5*q.k5+i5*q.j5+k5*q.u5+u7*q.k7+k6*q.u6-i7*q.j7+u1*q.k1-i2*q.j2-k2*q.u2+u2*q.k2+iqj-k7*q.u7+j7*q.i7-i4*q.j4-i1*q.j1+j1*q.i1-jqi+j2*q.i2)*q.r
			+(-k6qu1-u6qk1+k5qu2-j5qi2+i5qj2+u5qk2-j4qi3+i4qj3-j3qi4+i3qj4-j2qi5+i2qj5)*q.u7
			+(-k6qi1+u6qj1-j5qu2-k5qi2-u5qj2+i5qk2+j4qu3+i4qk3-k3qi4-u3qj4+j2qu5+i2qk5)*q.i7
			+(-u6qi1-k6qj1+i5qu2+u5qi2-k5qj2+j5qk2-i4qu3+j4qk3+u3qi4-k3qj4-i2qu5+j2qk5)*q.j7
			+(u6qu1-k6qk1-u5qu2+i5qi2+j5qj2+k5qk2-i4qi3-j4qj3+i3qi4+j3qj4-i2qi5-j2qj5)*q.k7
			+(-j6qi+i6qj-k7qu1+j7qi1-i7qj1+u7qk1-j4qi2+i4qj2-k5qu3+j5qi3-i5qj3+u5qk3)*q.u6
			+(-k6qi-u6qj+j4qu2+i4qk2+j5qu3+k5qi3-u5qj3-i5qk3+j3qu5+k3qi5-u3qj5-i3qk5)*q.i6
			+(u6qi-k6qj-i4qu2+j4qk2-i5qu3+u5qi3+k5qj3-j5qk3-i3qu5+u3qi5+k3qj5-j3qk5)*q.j6
			+(i6qi+j6qj+u7qu1+i7qi1+j7qj1+k7qk1-i4qi2-j4qj2-u5qu3-i5qi3-j5qj3-k5qk3)*q.k6
			+(-j5qi+i5qj-j4qi1+i4qj1+k7qu2-u7qk2+k6qu3-u6qk3)*q.u5
			+(-k5qi-u5qj+j4qu1+i4qk1-j7qu2-i7qk2+k6qi3+u6qj3)*q.i5
			+(u5qi-k5qj-i4qu1+j4qk1+i7qu2-j7qk2-u6qi3+k6qj3)*q.j5
			+(i5qi+j5qj-i4qi1-j4qj1-u7qu2-k7qk2+u6qu3+k6qk3)*q.k5
			+(j5qu1+k5qi1-u5qj1-i5qk1+j6qu2+k6qi2-u6qj2-i6qk2)*q.i4
			+(-i5qu1+u5qi1+k5qj1-j5qk1-i6qu2+u6qi2+k6qj2-j6qk2)*q.j4
			+(-j3qi+i3qj-j2qi1+i2qj1)*q.u3
			+(-k3qi-u3qj+j2qu1+i2qk1)*q.i3
			+(u3qi+j2qk1-k3qj-i2qu1)*q.j3
			+(i3qi-i2qi1+j3qj-j2qj1)*q.k3
			+(k3qi1-u3qj1+j3qu1-i3qk1)*q.i2
			+q.j2*(k3qj1-i3qu1+u3qi1-j3qk1)
		),
		// U1
		(sum + 2*u1u1)*u1+2*(
			(iqi+u2qu2+k1qk1+i1qi1+j1qj1+kqk+jqj+j2qj2+i2qi2+i3qi3+u3qu3+k2qk2+u4qu4+k3qk3+j3qj3+k4qk4+j4qj4+i4qi4+j5qj5+i5qi5+u5qu5+j6qj6+i6qi6+u6qu6+k5qk5+k7qk7+j7qj7+i7qi7+u7qu7+k6qk6)*q.u1
			-(j4*q.j5-i6*q.i7-k6*q.k7-i*q.i1-j5*q.j4-u5*q.u4+i1qi+j7*q.j6+k4*q.k5+u2*q.u3+i2*q.i3+j2*q.j3-k*q.k1-j*q.j1+j1qj+k1qk-k3*q.k2-i5*q.i4-k5*q.k4+k2*q.k3-j3*q.j2-u3*q.u2-i3*q.i2+u4*q.u5+i4*q.i5-u6*q.u7-j6*q.j7+u7*q.u6+k7*q.k6+i7*q.i6)*q.r
			+(-u7qk+u6qk1-k5qu2+j5qi2-i5qj2+k4qu3+j4qi3-i4qj3+j3qi4-i3qj4+j2qi5-i2qj5)*q.k6
			+(-u7qj+u6qj1-j5qu2-k5qi2+i5qk2+j4qu3-k4qi3+i4qk3-k3qi4+i3qk4-k2qi5+i2qk5)*q.j6
			+(-u7qi+u6qi1-i5qu2+k5qj2-j5qk2+i4qu3+k4qj3-j4qk3+k3qj4-j3qk4+k2qj5-j2qk5)*q.i6
			+(i7qi+j7qj+k7qk-i6qi1-j6qj1-k6qk1+i5qi2+j5qj2+k5qk2-i4qi3-j4qj3-k4qk3)*q.u6
			+(-u6qk-u7qk1+k4qu2+j4qi2-i4qj2+k5qu3-j5qi3+i5qj3+j2qi4-i2qj4-j3qi5+i3qj5)*q.k7
			+(-u6qj-u7qj1+j4qu2-k4qi2+i4qk2+j5qu3+k5qi3-i5qk3-k2qi4+i2qk4+k3qi5-i3qk5)*q.j7
			+(-u6qi-u7qi1+i4qu2+k4qj2-j4qk2+i5qu3-k5qj3+j5qk3+k2qj4-j2qk4-k3qj5+j3qk5)*q.i7
			+(i6qi+j6qj+k6qk+i7qi1+j7qj1+k7qk1-i4qi2-j4qj2-k4qk2-i5qi3-j5qj3-k5qk3)*q.u7
			+(j5qi-i5qj+j4qi1-i4qj1-k7qu2+u7qk2-k6qu3+u6qk3)*q.k4
			+(-k5qi+i5qk-k4qi1+i4qk1-j7qu2+u7qj2-j6qu3+u6qj3)*q.j4
			+(k5qj-j5qk+k4qj1-j4qk1-i7qu2+u7qi2-i6qu3+u6qi3)*q.i4
			+(j4qi-i4qj-j5qi1+i5qj1+k6qu2-u6qk2-k7qu3+u7qk3)*q.k5
			+(-k4qi+i4qk+k5qi1-i5qk1+j6qu2-u6qj2-j7qu3+u7qj3)*q.j5
			+(k4qj-j4qk-k5qj1+j5qk1+i6qu2-u6qi2-i7qu3+u7qi3)*q.i5
			+(j3qi-i3qj+j2qi1-i2qj1)*q.k2
			+(-k2qi1-k3qi+i3qk+i2qk1)*q.j2
			+(-j2qk1+k2qj1+k3qj-j3qk)*q.i2
			+(j2qi-j3qi1-i2qj+i3qj1)*q.k3
			+(-k2qi+k3qi1-i3qk1+i2qk)*q.j3
			+(-k3qj1-j2qk+j3qk1+k2qj)*q.i3
		),
		// I1
		(sum + 2*i1i1)*i1+2*(
			(kqk+jqj+iqi+u2qu2+k1qk1+u1qu1+j1qj1+i3qi3+u3qu3+k2qk2+j2qj2+i2qi2+j3qj3+u4qu4+k3qk3+u5qu5+k4qk4+j4qj4+i4qi4+i7qi7+u7qu7+k6qk6+j6qj6+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+k7qk7+j7qj7)*q.i1
			-(-j*q.k1-j7*q.k6+u2*q.i3-j1qk-i2*q.u3+u4*q.i5-i4*q.u5+k*q.j1-j6*q.k7+k6*q.j7+i*q.u1+i7*q.u6+k7*q.j6-i3*q.u2+u5*q.i4+u3*q.i2+k1qj-i5*q.u4-u1qi+j2*q.k3-k2*q.j3+j3*q.k2-k3*q.j2+j4*q.k5-k4*q.j5+j5*q.k4-k5*q.j4-u6*q.i7+i6*q.u7-u7*q.i6)*q.r
			+(-i7qj+u6qk1-k5qu2+j5qi2-u5qk2+k4qu3+j4qi3-i4qj3+j3qi4-u3qk4+k2qu5-i2qj5)*q.j6
			+(-i7qk-u6qj1+j5qu2+k5qi2+u5qj2-j4qu3+k4qi3-i4qk3+k3qi4+u3qj4-j2qu5-i2qk5)*q.k6
			+(-u7qi+k7qj-j7qk+i6qu1+k6qj1-j6qk1-u5qi2+k5qj2-j5qk2+i4qu3+k4qj3-j4qk3)*q.u6
			+(-i7qi-u6qu1+u5qu2-j5qj2-k5qk2+i4qi3+j4qj3+k4qk3-j3qj4-k3qk4+j2qj5+k2qk5)*q.i6
			+(u6qk-i7qj1-k4qu2-j4qi2+i4qj2-k5qu3+j5qi3+u5qk3-j2qi4+i2qj4-k3qu5+u3qk5)*q.j7
			+(-u6qj-i7qk1+j4qu2-k4qi2+i4qk2+j5qu3+k5qi3-u5qj3-k2qi4+i2qk4+j3qu5-u3qj5)*q.k7
			+(u6qi-i7qu1-i4qu2-k4qj2+j4qk2+u5qi3+k5qj3-j5qk3-k2qj4+j2qk4+k3qj5-j3qk5)*q.u7
			+(i6qi+j6qj+k6qk+u7qu1+j7qj1+k7qk1-i4qi2-j4qj2-k4qk2-u5qu3-j5qj3-k5qk3)*q.i7
			+(j5qi-u5qk+k4qu1-i4qj1-k7qu2+i7qj2-j6qi3+u6qk3)*q.j4
			+(k5qi+u5qj-j4qu1-i4qk1+j7qu2+i7qk2-k6qi3-u6qj3)*q.k4
			+(-j5qj-k5qk+j4qj1+k4qk1+u7qu2+i7qi2-u6qu3-i6qi3)*q.i4
			+(-j4qi+i4qj-k5qu1+u5qk1-k6qu2+u6qk2-j7qi3+i7qj3)*q.j5
			+(-k4qi+i4qk+j5qu1-u5qj1+j6qu2-u6qj2-k7qi3+i7qk3)*q.k5
			+(-k4qj+j4qk+k5qj1-j5qk1-i6qu2+u6qi2+i7qu3-u7qi3)*q.u5
			+(j3qi-u3qk+k2qu1-i2qj1)*q.j2
			+(u3qj+k3qi-j2qu1-i2qk1)*q.k2
			+(k2qk1-j3qj+j2qj1-k3qk)*q.i2
			+(-j2qi-k3qu1+i2qj+u3qk1)*q.j3
			+(i2qk-k2qi+j3qu1-u3qj1)*q.k3
			-(k2qj+j3qk1-j2qk-k3qj1)*q.u3
		),
		// J1
		(sum + 2*j1j1)*j1+2*(
			(u1qu1+u3qu3+k2qk2+j2qj2+u4qu4+k3qk3+j3qj3+i3qi3+i5qi5+u5qu5+k4qk4+j4qj4+i4qi4+j5qj5+k5qk5+u6qu6+u7qu7+k6qk6+j6qj6+i6qi6+k7qk7+j7qj7+i7qi7+i1qi1+kqk+jqj+iqi+i2qi2+u2qu2+k1qk1)*q.j1
			-(i*q.k1-j2*q.u3-u7*q.j6-j3*q.u2+k3*q.i2-i4*q.k5-u6*q.j7+i6*q.k7-k6*q.i7+k2*q.i3+u3*q.j2+u5*q.j4-j4*q.u5+k4*q.i5-i5*q.k4-j5*q.u4+k5*q.i4-i2*q.k3-k7*q.i6+i7*q.k6-k1qi+j6*q.u7+j7*q.u6+i1qk-k*q.i1+u2*q.j3+u4*q.j5-i3*q.k2+j*q.u1-u1qj)*q.r
			+(-j7qi-u6qk1+k5qu2+i5qj2+u5qk2-k4qu3-j4qi3+i4qj3+i3qj4+u3qk4-k2qu5-j2qi5)*q.i6
			+(-k7qi-u7qj+i7qk+j6qu1-k6qi1+i6qk1-k5qi2-u5qj2+i5qk2+j4qu3-k4qi3+i4qk3)*q.u6
			+(-j7qk+u6qi1-i5qu2-u5qi2+k5qj2+i4qu3+k4qj3-j4qk3-u3qi4+k3qj4+i2qu5-j2qk5)*q.k6
			+(-j7qj-u6qu1+u5qu2-i5qi2-k5qk2+i4qi3+j4qj3+k4qk3-i3qi4-k3qk4+i2qi5+k2qk5)*q.j6
			+(-u6qk-j7qi1+k4qu2+j4qi2-i4qj2+k5qu3+i5qj3-u5qk3+j2qi4-i2qj4+k3qu5-u3qk5)*q.i7
			+(u6qj-j7qu1-j4qu2+k4qi2-i4qk2-k5qi3+u5qj3+i5qk3+k2qi4-i2qk4-k3qi5+i3qk5)*q.u7
			+(u6qi-j7qk1-i4qu2-k4qj2+j4qk2-i5qu3+u5qi3+k5qj3-k2qj4+j2qk4-i3qu5+u3qi5)*q.k7
			+(i6qi+j6qj+k6qk+u7qu1+i7qi1+k7qk1-i4qi2-j4qj2-k4qk2-u5qu3-i5qi3-k5qk3)*q.j7
			+(i5qj+u5qk-k4qu1-j4qi1+k7qu2+j7qi2-i6qj3-u6qk3)*q.i4
			+(-u5qi+k5qj+i4qu1-j4qk1-i7qu2+j7qk2+u6qi3-k6qj3)*q.k4
			+(-i5qi-k5qk+i4qi1+k4qk1+u7qu2+j7qj2-u6qu3-j6qj3)*q.j4
			+(j4qi-i4qj+k5qu1-u5qk1+k6qu2-u6qk2+j7qi3-i7qj3)*q.i5
			+(k4qi-i4qk-k5qi1+i5qk1-j6qu2+u6qj2+j7qu3-u7qj3)*q.u5
			+(-k4qj+j4qk-i5qu1+u5qi1-i6qu2+u6qi2-k7qj3+j7qk3)*q.k5
			+(i3qj+u3qk-k2qu1-j2qi1)*q.i2
			+(-u3qi+k3qj-j2qk1+i2qu1)*q.k2
			+(-i3qi+i2qi1-k3qk+k2qk1)*q.j2
			+(j2qi-i2qj+k3qu1-u3qk1)*q.i3
			+(k2qi+i3qk1-k3qi1-i2qk)*q.u3
			-(-u3qi1-j2qk+i3qu1+k2qj)*q.k3
		),
		// K1
		(sum + 2*k1k1)*k1+2*(
			(i7qi7+u7qu7+k6qk6+j6qj6+k7qk7+j7qj7+i6qi6+u6qu6+k5qk5+j5qj5+iqi+u1qu1+i1qi1+kqk+jqj+k2qk2+j2qj2+i2qi2+u2qu2+j1qj1+u4qu4+k3qk3+j3qj3+i3qi3+u3qu3+u5qu5+k4qk4+j4qj4+i4qi4+i5qi5)*q.k1
			-(-i*q.j1-j3*q.i2+k*q.u1-u1qk+i2*q.j3-j2*q.i3+i3*q.j2+i4*q.j5-j4*q.i5+i5*q.j4-j5*q.i4-u6*q.k7+k6*q.u7-u7*q.k6+k7*q.u6+j*q.i1+j1qi-k4*q.u5+j7*q.i6-i1qj-k2*q.u3+u3*q.k2+u2*q.k3-k3*q.u2+u4*q.k5+u5*q.k4-k5*q.u4-i6*q.j7+j6*q.i7-i7*q.j6)*q.r
			+(j7qi-i7qj-u7qk+k6qu1+j6qi1-i6qj1+j5qi2-i5qj2-u5qk2+k4qu3+j4qi3-i4qj3)*q.u6
			+(-k7qi+u6qj1-j5qu2-u5qj2+i5qk2+j4qu3-k4qi3+i4qk3-u3qj4+i3qk4+j2qu5-k2qi5)*q.i6
			+(-k7qj-u6qi1+i5qu2+u5qi2+j5qk2-i4qu3-k4qj3+j4qk3+u3qi4+j3qk4-i2qu5-k2qj5)*q.j6
			+(-k7qk-u6qu1+u5qu2-i5qi2-j5qj2+i4qi3+j4qj3+k4qk3-i3qi4-j3qj4+i2qi5+j2qj5)*q.k6
			+(u6qk-k7qu1-k4qu2-j4qi2+i4qj2+j5qi3-i5qj3+u5qk3-j2qi4+i2qj4+j3qi5-i3qj5)*q.u7
			+(u6qj-k7qi1-j4qu2+k4qi2-i4qk2-j5qu3+u5qj3+i5qk3+k2qi4-i2qk4-j3qu5+u3qj5)*q.i7
			+(-u6qi-k7qj1+i4qu2+k4qj2-j4qk2+i5qu3-u5qi3+j5qk3+k2qj4-j2qk4+i3qu5-u3qi5)*q.j7
			+(i6qi+j6qj+k6qk+u7qu1+i7qi1+j7qj1-i4qi2-j4qj2-k4qk2-u5qu3-i5qi3-j5qj3)*q.k7
			+(-u5qj+i5qk+j4qu1-k4qi1-j7qu2+k7qi2+u6qj3-i6qk3)*q.i4
			+(u5qi+j5qk-i4qu1-k4qj1+i7qu2+k7qj2-u6qi3-j6qk3)*q.j4
			+(-i5qi-j5qj+i4qi1+j4qj1+u7qu2+k7qk2-u6qu3-k6qk3)*q.k4
			+(-j4qi+i4qj+j5qi1-i5qj1-k6qu2+u6qk2+k7qu3-u7qk3)*q.u5
			+(k4qi-i4qk-j5qu1+u5qj1-j6qu2+u6qj2+k7qi3-i7qk3)*q.i5
			+(k4qj-j4qk+i5qu1-u5qi1+i6qu2-u6qi2+k7qj3-j7qk3)*q.j5
			+(-u3qj+i3qk+j2qu1-k2qi1)*q.i2
			+(u3qi+j3qk-i2qu1-k2qj1)*q.j2
			+(-i3qi+i2qi1-j3qj+j2qj1)*q.k2
			+(-j2qi+j3qi1+i2qj-i3qj1)*q.u3
			+(-i2qk+k2qi+u3qj1-j3qu1)*q.i3
			-(-k2qj+j2qk-i3qu1+u3qi1)*q.j3
		),
		// U2
		(sum + 2*u2u2)*u2+2*(
			(j7qj7+iqi+i1qi1+kqk+jqj+i3qi3+u3qu3+k2qk2+j2qj2+i2qi2+k1qk1+j1qj1+u1qu1+j4qj4+i4qi4+u4qu4+k3qk3+j3qj3+j6qj6+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+u5qu5+k4qk4+u7qu7+k6qk6+k7qk7+i7qi7)*q.u2
			-(j4*q.j6+u5*q.u7-u6*q.u4-j*q.j2-k1*q.k3-i6*q.i4+k4*q.k6+u3*q.u1+j3*q.j1+i4*q.i6+i5*q.i7+j5*q.j7-j6*q.j4-i7*q.i5-k7*q.k5-k6*q.k4-i*q.i2-j7*q.j5+k5*q.k7+k3*q.k1-u1*q.u3-i1*q.i3+i3*q.i1-j1*q.j3+j2qj-u7*q.u5+u4*q.u6-k*q.k2+i2qi+k2qk)*q.r
			+(-j7qi+i7qj+u7qk-k6qu1-j6qi1+i6qj1-j5qi2+i5qj2+u5qk2-k4qu3-j4qi3+i4qj3)*q.k5
			+(k7qi+u7qj-i7qk-j6qu1+k6qi1-i6qk1+k5qi2+u5qj2-i5qk2-j4qu3+k4qi3-i4qk3)*q.j5
			+(u7qi-k7qj+j7qk-i6qu1-k6qj1+j6qk1+u5qi2-k5qj2+j5qk2-i4qu3-k4qj3+j4qk3)*q.i5
			+(-i7qi-j7qj-k7qk+i6qi1+j6qj1+k6qk1-i5qi2-j5qj2-k5qk2+i4qi3+j4qj3+k4qk3)*q.u5
			+(j6qi-i6qj+k7qu1-j7qi1+i7qj1-u7qk1+j4qi2-i4qj2+k5qu3-j5qi3+i5qj3-u5qk3)*q.k4
			+(-k6qi+i6qk+j7qu1+k7qi1-u7qj1-i7qk1-k4qi2+i4qk2+j5qu3+k5qi3-u5qj3-i5qk3)*q.j4
			+(k6qj-j6qk+i7qu1-u7qi1-k7qj1+j7qk1+k4qj2-j4qk2+i5qu3-u5qi3-k5qj3+j5qk3)*q.i4
			+(-j5qi+i5qj+u5qk-k4qu1-j4qi1+i4qj1+j7qi2-i7qj2-u7qk2+k6qu3+j6qi3-i6qj3)*q.k7
			+(k5qi+u5qj-i5qk-j4qu1+k4qi1-i4qk1-k7qi2-u7qj2+i7qk2+j6qu3-k6qi3+i6qk3)*q.j7
			+(u5qi-k5qj+j5qk-i4qu1-k4qj1+j4qk1-u7qi2+k7qj2-j7qk2+i6qu3+k6qj3-j6qk3)*q.i7
			+(-i5qi-j5qj-k5qk+i4qi1+j4qj1+k4qk1+i7qi2+j7qj2+k7qk2-i6qi3-j6qj3-k6qk3)*q.u7
			+(j4qi-i4qj+k5qu1-j5qi1+i5qj1-u5qk1-j6qi2+i6qj2-k7qu3+j7qi3-i7qj3+u7qk3)*q.k6
			+(i4qk-k4qi+j5qu1-u5qj1+k5qi1-i5qk1-i6qk2+k6qi2+i7qk3+u7qj3-j7qu3-k7qi3)*q.j6
			-(-j5qk1+j4qk-i5qu1-k4qj+k6qj2+u5qi1-j6qk2-k7qj3+i7qu3+k5qj1-u7qi3+j7qk3)*q.i6
		),
		// I2
		(sum + 2*i2i2)*i2+2*(
			-(j1qj+k1qk)*q.i3
			+(j6qi-u6qk+k7qu1-j7qi1+i7qj1-u7qk1+k4qu2-i4qj2-u1qk7+i1qj7-j1qi7+k1qu7)*q.j4
			+(k6qi+u6qj-j7qu1-k7qi1+u7qj1+i7qk1-j4qu2-i4qk2+u1qj7+i1qk7-j1qu7-k1qi7)*q.k4
			+(-j6qj-k6qk-u7qu1-i7qi1-j7qj1-k7qk1+j4qj2+k4qk2+u5qu3+i5qi3+j5qj3+k5qk3)*q.i4
			+(j7qi-i7qj-u7qk+k6qu1+j6qi1+u6qk1-k5qu2-i4qj3+jqi7+kqu7-u1qk6-i1qj6)*q.j5
			+(u7qi-k7qj+j7qk-u6qi1-k6qj1+j6qk1+i5qu2-i4qu3+jqk7-kqj7+j1qk6-k1qj6)*q.u5
			+(k7qi+u7qj-i7qk-j6qu1+k6qi1-u6qj1+j5qu2-i4qk3-jqu7+kqi7+u1qj6-i1qk6)*q.k5
			+(i7qi+j7qj+k7qk+u6qu1-j6qj1-k6qk1-u5qu2-i4qi3-jqj7-kqk7+j1qj6+k1qk6)*q.i5
			+(-j5qi+i4qj1+k7qu2-i7qj2-u7qk2+k6qu3+j6qi3-u6qk3)*q.j7
			+(-i5qi+i4qi1+u7qu2+j7qj2+k7qk2-u6qu3-j6qj3-k6qk3)*q.i7
			-(-u3qi1+j3qk1-k3qj1+j4qk6-k4qj6-j5qk7+k5qj7+j6qk4-k6qj4-j7qk5+k7qj5-i6*q.u4+u6*q.i4-i7qu5-u1qi3-u2qi-i4*q.u6+i3qu1+u5qi7+u4*q.i6-i5qu7+u7qi5+i*q.u2+kqj2+i1qu3-jqk2+j1qk3-k1qj3-j2qk+k2qj)*q.r
			+(-u5qi+i4qu1-i7qu2-k7qj2+j7qk2+u6qi3-k6qj3+j6qk3)*q.u7
			+(k6qk6+i6qi6+u6qu6+k5qk5+i7qi7+u7qu7+k7qk7+j7qj7+u2qu2+j6qj6+kqk+jqj+iqi+i3qi3+u3qu3+k2qk2+j2qj2+k1qk1+j1qj1+u1qu1+i1qi1+j5qj5+i5qi5+u5qu5+k4qk4+j4qj4+i4qi4+u4qu4+k3qk3+j3qj3)*q.i2
			+(kqi3+u1qj2-jqu3-i1qk2)*q.k1
			+(-k1qj2+jqk3-kqj3+j1qk2)*q.u1
			+(j1qj2+k1qk2-jqj3-kqk3)*q.i1
			+(i1qj+u1qk)*q.j3
			+(-u1qj+i1qk)*q.k3
			+(k1qj-j1qk)*q.u3
			+(-k4qj+j4qk-i5qu1+u5qi1+k5qj1-j5qk1+k6qj2-j6qk2+i7qu3-u7qi3-k7qj3+j7qk3)*q.u6
			+(-u1qk2+kqu3-i1qj2+jqi3)*q.j1
			+(-j4qi+i4qj-k6qu2+u6qk2+k7qu3-j7qi3+i7qj3-u7qk3)*q.j6
			+(-k4qi+i4qk+j6qu2-u6qj2-j7qu3-k7qi3+u7qj3+i7qk3)*q.k6
			+(-k5qi+i4qk1-j7qu2+u7qj2-i7qk2-j6qu3+k6qi3+u6qj3)*q.k7
		),
		// J2
		(sum + 2*j2j2)*j2+2*(
			(-u7qi+k7qj-j7qk+i6qu1+u6qi1+k6qj1-i5qu2-j4qk3+iqu7+kqj7-u1qi6-j1qk6)*q.k5
			+(i1qk-k1qi)*q.u3
			+(j1qi-u1qk)*q.i3
			+(u1qi+j1qk)*q.k3
			+(i7qi+j7qj+k7qk+u6qu1-i6qi1-k6qk1-u5qu2-j4qj3-iqi7-kqk7+i1qi6+k1qk6)*q.j5
			+(i6qj+u6qk-k7qu1+j7qi1-i7qj1+u7qk1-k4qu2-j4qi2+u1qk7-i1qj7+j1qi7-k1qu7)*q.i4
			+(-u6qi+k6qj+i7qu1-u7qi1-k7qj1+j7qk1+i4qu2-j4qk2-u1qi7+i1qu7+j1qk7-k1qj7)*q.k4
			+(-k4qj+j4qk-i6qu2+u6qi2+i7qu3-u7qi3-k7qj3+j7qk3)*q.k6
			+(-k5qj+j4qk1+i7qu2-u7qi2-j7qk2+i6qu3-u6qi3+k6qj3)*q.k7
			+(-j1qi2+iqj3-kqu3+u1qk2)*q.i1
			+(-iqk3+kqi3-i1qk2+k1qi2)*q.u1
			+(-i5qj+j4qi1-k7qu2-j7qi2+u7qk2-k6qu3+i6qj3+u6qk3)*q.i7
			+(iqu3-j1qk2+kqj3-u1qi2)*q.k1
			+(k1qk2-iqi3-kqk3+i1qi2)*q.j1
			+(k7qk7+j7qj7+i7qi7+u7qu7+k6qk6+j6qj6+i6qi6+u2qu2+jqj+iqi+u1qu1+i1qi1+kqk+u3qu3+k2qk2+i2qi2+k1qk1+j1qj1+k3qk3+j3qj3+i3qi3+i4qi4+u4qu4+j4qj4+k4qk4+u6qu6+k5qk5+j5qj5+i5qi5+u5qu5)*q.j2
			-(i1qi+k1qk)*q.j3
			+(j4qi-i4qj+k6qu2-u6qk2-k7qu3+j7qi3-i7qj3+u7qk3)*q.i6
			+(k7qi+u7qj-i7qk+k6qi1-u6qj1-i6qk1+j5qu2-j4qu3-iqk7+kqi7-i1qk6+k1qi6)*q.u5
			+(k4qi-i4qk-j5qu1-k5qi1+u5qj1+i5qk1-k6qi2+i6qk2+j7qu3+k7qi3-u7qj3-i7qk3)*q.u6
			+(-i6qi-k6qk-u7qu1-i7qi1-j7qj1-k7qk1+i4qi2+k4qk2+u5qu3+i5qi3+j5qj3+k5qk3)*q.j4
			+(-j5qj+j4qj1+u7qu2+i7qi2+k7qk2-u6qu3-i6qi3-k6qk3)*q.j7
			-(-j4*q.u6-k7qi5+i2qk+iqk2-u1qj3-kqi2+j1qu3+k1qi3-k2qi-u3qj1-i3qk1+j3qu1-i4qk6+k4qi6+u5qj7-j5qu7+k6qi4+u7qj5-j7qu5+i7qk5+j*q.u2-i1qk3+k3qi1+u6*q.j4-u2qj+u4*q.j6+i5qk7-k5qi7-j6*q.u4-i6qk4)*q.r
			+(-u5qj+j4qu1-j7qu2+k7qi2-i7qk2+k6qi3+u6qj3-i6qk3)*q.u7
			+(-j7qi+i7qj+u7qk-k6qu1+i6qj1-u6qk1+k5qu2-j4qi3+iqj7-kqu7+u1qk6-j1qi6)*q.i5
		),
		// K2
		(sum + 2*k2k2)*k2+2*(
			(-i6qi-j6qj-u7qu1-i7qi1-j7qj1-k7qk1+i4qi2+j4qj2+u5qu3+i5qi3+j5qj3+k5qk3)*q.k4
			+(-k7qi-u7qj+i7qk+j6qu1+u6qj1+i6qk1-j5qu2-k4qi3+iqk7+jqu7-u1qj6-k1qi6)*q.i5
			+(-j5qk+k4qj1-i7qu2+u7qi2-k7qj2-i6qu3+u6qi3+j6qk3)*q.j7
			+(u7qi-k7qj+j7qk-i6qu1-u6qi1+j6qk1+i5qu2-k4qj3-iqu7+jqk7+u1qi6-k1qj6)*q.j5
			+(i7qi+j7qj+k7qk+u6qu1-i6qi1-j6qj1-u5qu2-k4qk3-iqi7-jqj7+i1qi6+j1qj6)*q.k5
			+(-u6qj+i6qk+j7qu1+k7qi1-u7qj1-i7qk1+j4qu2-k4qi2-u1qj7-i1qk7+j1qu7+k1qi7)*q.i4
			+(-k5qk+k4qk1+u7qu2+i7qi2+j7qj2-u6qu3-i6qi3-j6qj3)*q.k7
			+(u6qi+j6qk-i7qu1+u7qi1+k7qj1-j7qk1-i4qu2-k4qj2+u1qi7-i1qu7-j1qk7+k1qj7)*q.j4
			+(-j4qi+i4qj-k5qu1+j5qi1-i5qj1+u5qk1+j6qi2-i6qj2+k7qu3-j7qi3+i7qj3-u7qk3)*q.u6
			+(k4qi-i4qk-j6qu2+u6qj2+j7qu3+k7qi3-u7qj3-i7qk3)*q.i6
			+(k4qj-j4qk+i6qu2-u6qi2-i7qu3+u7qi3+k7qj3-j7qk3)*q.j6
			+(u1qu1+i1qi1+kqk+jqj+iqi+u2qu2+k1qk1+j1qj1+i3qi3+u3qu3+j2qj2+i2qi2+i4qi4+u4qu4+k3qk3+j3qj3+k4qk4+j4qj4+u5qu5+j5qj5+i5qi5+k5qk5+j7qj7+i7qi7+u7qu7+k6qk6+j6qj6+i6qi6+u6qu6+k7qk7)*q.k2
			+(iqj3-jqi3+i1qj2-j1qi2)*q.u1
			+(jqu3-u1qj2-k1qi2+iqk3)*q.i1
			+(-k1qj2+jqk3+u1qi2-iqu3)*q.j1
			+(j1qi-i1qj)*q.u3
			+(-iqi3+i1qi2-jqj3+j1qj2)*q.k1
			+(k1qi+u1qj)*q.i3
			+(-u1qi+k1qj)*q.j3
			+(-i5qk+k4qi1+j7qu2-k7qi2-u7qj2+j6qu3-u6qj3+i6qk3)*q.i7
			-(-j1qi3-i2qj-iqj2+jqi2-u1qk3+i1qj3+k1qu3+j2qi-u3qk1-j3qi1+k3qu1+i4qj6-j4qi6-k4*q.u6+u5qk7+j5qi7-k5qu7+u6*q.k4+i6qj4-j6qi4+i3qj1-k7qu5+u7qk5-i7qj5+k*q.u2+j7qi5-u2qk+u4*q.k6-k6*q.u4-i5qj7)*q.r
			+(-j7qi+i7qj+u7qk-j6qi1+i6qj1-u6qk1+k5qu2-k4qu3+iqj7-jqi7+i1qj6-j1qi6)*q.u5
			+(-u5qk+k4qu1-k7qu2-j7qi2+i7qj2-j6qi3+i6qj3+u6qk3)*q.u7
			-(i1qi+j1qj)*q.k3
		),
		// U3
		(sum + 2*u3u3)*u3+2*(
			(-i6qi-j6qj-k6qk-i7qi1-j7qj1-k7qk1+i4qi2+j4qj2+k4qk2+i5qi3+j5qj3+k5qk3)*q.u5
			+(-u6qi+k6qj-j6qk+i7qu1-k7qj1+j7qk1+i4qu2-u5qi3-jqk6+kqj6+j1qk7-k1qj7)*q.i5
			+(k7qj-j7qk+i6qu1+u6qi1+k6qj1-j6qk1-i5qu2-u5qi2-jqk7+kqj7-j1qk6+k1qj6)*q.i4
			+(-k5qu1+u5qk1-k6qu2+j6qi2-i6qj2+u6qk2-j7qi3+i7qj3)*q.k7
			+(j6qi-i6qj-u6qk+k7qu1-j7qi1+i7qj1+k4qu2-u5qk3-iqj6+jqi6+i1qj7-j1qi7)*q.k5
			+(-k6qi-u6qj+i6qk+j7qu1+k7qi1-i7qk1+j4qu2-u5qj3+iqk6-kqi6-i1qk7+k1qi7)*q.j5
			+(i5qi+j5qj+k5qk-i4qi1-j4qj1-k4qk1-i7qi2-j7qj2-k7qk2+i6qi3+j6qj3+k6qk3)*q.u6
			-(-j3qj-k3qk-i4qi7-j4qj7-k4qk7-u5*q.u6+j5qj6+k5qk6+u6*q.u5-j6qj5-k6qk5+i7qi4+j7qj4+k7qk4+u1*q.u2+u4*q.u7+iqi3-u2*q.u1-i3qi-i6qi5+i5qi6-k1qk2-u7*q.u4+jqj3+kqk3-i1qi2-j1qj2+i2qi1+j2qj1+k2qk1)*q.r
			+(u5qi-i4qu1+i7qu2+k7qj2-j7qk2-u6qi3+k6qj3-j6qk3)*q.i6
			+(j7qi-i7qj+k6qu1+j6qi1-i6qj1+u6qk1-k5qu2-u5qk2-iqj7+jqi7-i1qj6+j1qi6)*q.k4
			+(-k7qi+i7qk+j6qu1-k6qi1+u6qj1+i6qk1-j5qu2-u5qj2+iqk7-kqi7+i1qk6-k1qi6)*q.j4
			+(u5qk-k4qu1+k7qu2+j7qi2-i7qj2+j6qi3-i6qj3-u6qk3)*q.k6
			+(-j5qu1+u5qj1-j6qu2-k6qi2+u6qj2+i6qk2+k7qi3-i7qk3)*q.j7
			+(jqj+iqi+k2qk2+j2qj2+i2qi2+k1qk1+j1qj1+u1qu1+i1qi1+kqk+j4qj4+i4qi4+u4qu4+k3qk3+j3qj3+i3qi3+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+u5qu5+k4qk4+j6qj6+k7qk7+j7qj7+i7qi7+u7qu7+k6qk6+u2qu2)*q.u3
			+(-i5qu1+u5qi1-i6qu2+u6qi2+k6qj2-j6qk2-k7qj3+j7qk3)*q.i7
			+(u5qj-j4qu1+j7qu2-k7qi2+i7qk2-k6qi3-u6qj3+i6qk3)*q.j6
			+(-j1qi3-iqj2+jqi2+i1qj3)*q.k1
			+(-i1qk3+k1qi3+iqk2-kqi2)*q.j1
			+(-k1qj3-jqk2+kqj2+j1qk3)*q.i1
			+(-j1qi+i1qj)*q.k2
			+(-i1qk+k1qi)*q.j2
			+(-k1qj+j1qk)*q.i2
			+(-jqi+iqj)*q.k3
			+q.i3*(jqk-kqj)
			+(kqi-iqk)*q.j3
		),
		// I3
		(sum + 2*i3i3)*i3+2*(
			(i5qi-i4qi1-u7qu2-j7qj2-k7qk2+u6qu3+j6qj3+k6qk3)*q.i6
			+(u6qi-k6qj+j6qk+u7qi1+k7qj1-j7qk1-i4qu2-i5qu3+jqk6-kqj6-j1qk7+k1qj7)*q.u5
			+(-i6qi-j6qj-k6qk-u7qu1-j7qj1-k7qk1+i4qi2+j4qj2+k4qk2+u5qu3+j5qj3+k5qk3)*q.i5
			+(jqk2-kqj2-j1qk3+k1qj3)*q.u1
			+(-j1qi+u1qk)*q.j2
			+(-k1qi-u1qj)*q.k2
			+(j1qj+k1qk)*q.i2
			+(kqi-iqk)*q.k3
			+(jqi-iqj)*q.j3
			-(-j5qk6+u1qi2-k1qj2-kqj3+jqk3+u6*q.i5+i6qu5-j6qk5-u7qi4+k7qj4-j7qk4+k5qj6+j1qk2-i2qu1+j2qk1-k2qj1+u3qi-k3qj+i4qu7-j4qk7+k4qj7-u5qi6-i5*q.u6+i1*q.u2+u4*q.i7-iqu3+k6qj5-i7*q.u4+j3qk-u2*q.i1)*q.r
			+(i5qj-j4qi1+k7qu2+j7qi2-u7qk2+k6qu3-i6qj3-u6qk3)*q.j6
			+(j7qi-u7qk+k6qu1+j6qi1-i6qj1+u6qk1-k5qu2-i5qj2-iqj7+kqu7-u1qk6+j1qi6)*q.j4
			+(k7qi+u7qj-j6qu1+k6qi1-u6qj1-i6qk1+j5qu2-i5qk2-iqk7-jqu7+u1qj6+k1qi6)*q.k4
			+(-j7qj-k7qk-u6qu1+i6qi1+j6qj1+k6qk1+u5qu2-i5qi2+jqj7+kqk7-j1qj6-k1qk6)*q.i4
			+(-u5qi+k5qj-j5qk+i4qu1+k4qj1-j4qk1+u7qi2-k7qj2+j7qk2-i6qu3-k6qj3+j6qk3)*q.u6
			+(-j6qi+i6qj+u6qk-k7qu1+j7qi1+u7qk1-k4qu2-i5qj3+iqj6-jqi6+u1qk7-k1qu7)*q.j5
			+(-k6qi-u6qj+i6qk+j7qu1+k7qi1-u7qj1+j4qu2-i5qk3+iqk6-kqi6-u1qj7+j1qu7)*q.k5
			+(i5qk-k4qi1-j7qu2+k7qi2+u7qj2-j6qu3+u6qj3-i6qk3)*q.k6
			+(-j5qi1+i5qj1+k6qu2-j6qi2+i6qj2-u6qk2-k7qu3+u7qk3)*q.j7
			+(-k5qi1+i5qk1-j6qu2-k6qi2+u6qj2+i6qk2+j7qu3-u7qj3)*q.k7
			+(i5qu1-u5qi1+i6qu2-u6qi2-k6qj2+j6qk2+k7qj3-j7qk3)*q.u7
			+(iqi+jqj+kqk+u1qu1+i1qi1+u2qu2+k1qk1+j1qj1+k2qk2+j2qj2+i2qi2+j3qj3+u3qu3+k3qk3+i5qi5+u5qu5+k4qk4+j4qj4+i4qi4+u4qu4+u6qu6+k5qk5+j5qj5+u7qu7+k6qk6+j6qj6+i6qi6+k7qk7+j7qj7+i7qi7)*q.i3
			+(u1qk3-k1qu3+iqj2-jqi2)*q.j1
			+(-u1qj3+j1qu3+iqk2-kqi2)*q.k1
			-q.u3*(jqk-kqj)
		),
		// J3
		(sum + 2*j3j3)*j3+2*(
			(u6qi-k6qj+j6qk-i7qu1+u7qi1+k7qj1-i4qu2-j5qk3+jqk6-kqj6+u1qi7-i1qu7)*q.k5
			+(-i6qi-j6qj-k6qk-u7qu1-i7qi1-k7qk1+i4qi2+j4qj2+k4qk2+u5qu3+i5qi3+k5qk3)*q.j5
			+(i7qj+u7qk-k6qu1-j6qi1+i6qj1-u6qk1+k5qu2-j5qi2-jqi7-kqu7+u1qk6+i1qj6)*q.i4
			+(-u7qi+k7qj+i6qu1+u6qi1+k6qj1-j6qk1-i5qu2-j5qk2+iqu7-jqk7-u1qi6+k1qj6)*q.k4
			+(j6qi-i6qj-u6qk+k7qu1+i7qj1-u7qk1+k4qu2-j5qi3-iqj6+jqi6-u1qk7+k1qu7)*q.i5
			+(j5qj-j4qj1-u7qu2-i7qi2-k7qk2+u6qu3+i6qi3+k6qk3)*q.j6
			+(j5qi-i4qj1-k7qu2+i7qj2+u7qk2-k6qu3-j6qi3+u6qk3)*q.i6
			+(k6qi+u6qj-i6qk-k7qi1+u7qj1+i7qk1-j4qu2-j5qu3-iqk6+kqi6+i1qk7-k1qi7)*q.u5
			-(-k6qi5+u6*q.j5+u1qj2+i6qk5-i2qk1+k2qi1+u3qj+k3qi+j4qu7-k4qi7+i4qk7+u4*q.j7+i5qk6-j5*q.u6-k5qi6+j6qu5+i7qk4-j7*q.u4-k7qi4-i1qk2-u7qj4-j2qu1+k1qi2+j1*q.u2-u2*q.j1-jqu3-i3qk-iqk3+kqi3-u5qj6)*q.r
			+(-k5qi-u5qj+i5qk+j4qu1-k4qi1+i4qk1+k7qi2+u7qj2-i7qk2-j6qu3+k6qi3-i6qk3)*q.u6
			+(u1qi-k1qj)*q.k2
			+(-i7qi-k7qk-u6qu1+i6qi1+j6qj1+k6qk1+u5qu2-j5qj2+iqi7+kqk7-i1qi6-k1qk6)*q.j4
			+(j5qk-k4qj1+i7qu2-u7qi2+k7qj2+i6qu3-u6qi3-j6qk3)*q.k6
			+(-i1qj-u1qk)*q.i2
			+(-jqi+iqj)*q.i3
			+(i1qi+k1qk)*q.j2
			+(-kqi+iqk)*q.u3
			-q.k3*(-kqj+jqk)
			+(j5qi1-i5qj1-k6qu2+j6qi2-i6qj2+u6qk2+k7qu3-u7qk3)*q.i7
			+(j5qu1-u5qj1+j6qu2+k6qi2-u6qj2-i6qk2-k7qi3+i7qk3)*q.u7
			+(i2qi2+u2qu2+k1qk1+j1qj1+u1qu1+i1qi1+kqk+jqj+iqi+k3qk3+i3qi3+u3qu3+k2qk2+j2qj2+i5qi5+u5qu5+k4qk4+j4qj4+i4qi4+u4qu4+j5qj5+u6qu6+k5qk5+k6qk6+j6qj6+i6qi6+j7qj7+i7qi7+u7qu7+k7qk7)*q.j3
			+(-k5qj1+j5qk1+i6qu2-u6qi2-k6qj2+j6qk2-i7qu3+u7qi3)*q.k7
			+(-iqj2+jqi2-u1qk3+k1qu3)*q.i1
			+(-iqk2+kqi2+i1qk3-k1qi3)*q.u1
			+(-i1qu3+jqk2-kqj2+u1qi3)*q.k1
		),
		// K3
		(sum + 2*k3k3)*k3+2*(
			(-u6qi+k6qj-j6qk+i7qu1-u7qi1+j7qk1+i4qu2-k5qj3-jqk6+kqj6-u1qi7+i1qu7)*q.j5
			+(j5qi-i5qj-u5qk+k4qu1+j4qi1-i4qj1-j7qi2+i7qj2+u7qk2-k6qu3-j6qi3+i6qj3)*q.u6
			+(-j6qi+i6qj+u6qk+j7qi1-i7qj1+u7qk1-k4qu2-k5qu3+iqj6-jqi6-i1qj7+j1qi7)*q.u5
			+(k5qk-k4qk1-u7qu2-i7qi2-j7qj2+u6qu3+i6qi3+j6qj3)*q.k6
			+(-u7qj+i7qk+j6qu1-k6qi1+u6qj1+i6qk1-j5qu2-k5qi2+jqu7-kqi7-u1qj6+i1qk6)*q.i4
			+(k6qi+u6qj-i6qk-j7qu1+u7qj1+i7qk1-j4qu2-k5qi3-iqk6+kqi6+u1qj7-j1qu7)*q.i5
			+(u7qi+j7qk-i6qu1-u6qi1-k6qj1+j6qk1+i5qu2-k5qj2-iqu7-kqj7+u1qi6+j1qk6)*q.j4
			+(-i7qi-j7qj-u6qu1+i6qi1+j6qj1+k6qk1+u5qu2-k5qk2+iqi7+jqj7-i1qi6-j1qj6)*q.k4
			+(-i6qi-j6qj-k6qk-u7qu1-i7qi1-j7qj1+i4qi2+j4qj2+k4qk2+u5qu3+i5qi3+j5qj3)*q.k5
			+(k5qj1-j5qk1-i6qu2+u6qi2+k6qj2-j6qk2+i7qu3-u7qi3)*q.j7
			+(iqj2-jqi2-i1qj3+j1qi3)*q.u1
			+(i1qi1+kqk+jqj+iqi+u2qu2+k1qk1+j1qj1+u1qu1+i4qi4+u4qu4+j3qj3+i3qi3+u3qu3+k2qk2+j2qj2+i2qi2+u5qu5+k4qk4+j4qj4+k6qk6+j6qj6+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+k7qk7+j7qj7+i7qi7+u7qu7)*q.k3
			+(k5qi-i4qk1+j7qu2-u7qj2+i7qk2+j6qu3-k6qi3-u6qj3)*q.i6
			+(-jqk2+kqj2-u1qi3+i1qu3)*q.j1
			+(-iqk2+kqi2+u1qj3-j1qu3)*q.i1
			+(u1qj-i1qk)*q.i2
			+(-u1qi-j1qk)*q.j2
			+(j1qj+i1qi)*q.k2
			+(jqi-iqj)*q.u3
			+(-kqi+iqk)*q.i3
			+q.j3*(jqk-kqj)
			-(i3qj+iqj3-k7*q.u4-u7qk4-jqi3-u2*q.k1+i1qj2-kqu3+u1qk2-j1qi2-k5*q.u6-k2qu1+u3qk+j7qi4-j3qi-i4qj7+k4qu7-u5qk6+j5qi6-i6qj5+k6qu5-j2qi1+j4qi7+k1*q.u2+j6qi5+u4*q.k7+i2qj1-i5qj6+u6*q.k5-i7qj4)*q.r
			+(k5qj-j4qk1-i7qu2+u7qi2+j7qk2-i6qu3+u6qi3-k6qj3)*q.j6
			+(k5qu1-u5qk1+k6qu2-j6qi2+i6qj2-u6qk2+j7qi3-i7qj3)*q.u7
			+(k5qi1-i5qk1+j6qu2+k6qi2-u6qj2-i6qk2-j7qu3+u7qj3)*q.i7
		),
		// U4
		(sum + 2*u4u4)*u4+2*(
			(k3qk3+i4qi4+j4qj4+k4qk4+u5qu5+j5qj5+k5qk5+u6qu6+i6qi6+j6qj6+k6qk6+u7qu7+i7qi7+j7qj7+k7qk7+iqi+jqj+kqk+u1qu1+j1qj1+k2qk2+u3qu3+i3qi3+j3qj3+k1qk1+u2qu2+i2qi2+j2qj2+i1qi1+i5qi5)*q.u4
			+ q.r*(i3*q.i7+u3*q.u7+k2*q.k6+j2*q.j6+i2*q.i6+u2*q.u6+k1*q.k5+j1*q.j5+u1*q.u5+k*q.k4+j*q.j4+i*q.i4-u5*q.u1+k3*q.k7+j3*q.j7-k7*q.k3-j4*q.j-i4*q.i-k4*q.k+i1*q.i5-i5*q.i1-j5*q.j1-k5*q.k1-u6*q.u2-i6*q.i2-j6*q.j2-k6*q.k2-u7*q.u3-i7*q.i3-j7*q.j3)
		),
		// I4
		(sum + 2*i4i4)*i4+2*(
			-(i*q.u4-u5qi1+i1qu5+kqj4+j1qk5-k1qj5-u2qi6+i2qu6+j2qk6-k2qj6+u3qi7-i3qu7-j3qk7+k3qj7+k4qj+j5qk1-k5qj1-u6qi2+i6qu2+j6qk2-k6qj2+u7qi3-i7qu3-j7qk3+k7qj3-u1qi5-j4qk-u4*q.i-jqk4+i5qu1)*q.r
			-(j1qj+k1qk)*q.i5
			+(-jqj5-kqk5+j1qj4+k1qk4+u2qu7+i2qi7-u3qu6-i3qi6)*q.i1
			+(jqi5+kqu5-u1qk4-i1qj4+u2qk7+i2qj7-j3qi6-k3qu6)*q.j1
			+(-i7qj4-u7qk4+k6qu5+j6qi5)*q.j7
			+(u7qj4-i7qk4-j6qu5+k6qi5)*q.k7
			+(-j7qu1-k7qi1+u7qj1+i7qk1-jqu6+kqi6+u1qj7+i1qk7-j1qu7-k1qi7+u2qj4-i2qk4)*q.k2
			+(jqk5-kqj5+j1qk4-k1qj4-u2qi7+i2qu7-u3qi6+i3qu6)*q.u1
			+(jqk6-kqj6+u1qi7-i1qu7-j1qk7+k1qj7+j2qk4-k2qj4+u3qi5-i3qu5-j3qk5+k3qj5)*q.u2
			+(-i7qj-u7qk+k6qu1+j6qi1+jqi7+kqu7-u1qk6-i1qj6+j1qi6-k1qu6+u2qk5-i2qj5)*q.j3
			+(u7qj-i7qk-j6qu1+k6qi1-jqu7+kqi7+u1qj6-i1qk6+j1qu6+k1qi6-u2qj5-i2qk5)*q.k3
			+(-k7qj+j7qk-k6qj1+j6qk1+jqk7-kqj7+u1qi6+i1qu6+j1qk6-k1qj6-u2qi5-i2qu5)*q.u3
			+(-jqu5+kqi5+u1qj4-i1qk4-u2qj7+i2qk7+j3qu6-k3qi6)*q.k1
			+(j7qj+k7qk-j6qj1-k6qk1-jqj7-kqk7-u1qu6+i1qi6+j1qj6+k1qk6+u2qu5-i2qi5)*q.i3
			+(-jqj6-kqk6-u1qu7-i1qi7-j1qj7-k1qk7+j2qj4+k2qk4+u3qu5+i3qi5+j3qj5+k3qk5)*q.i2
			+(k7qu1-j7qi1+i7qj1-u7qk1+jqi6+kqu6-u1qk7+i1qj7-j1qi7+k1qu7-u2qk4-i2qj4)*q.j2
			+(-u1qj+i1qk)*q.k5
			+(k1qj-j1qk)*q.u5
			+(-k7qj4+j7qk4-k6qj5+j6qk5)*q.u7
			+(-u2qj-j7qu5+i2qk-k7qi5+u7qj5+i7qk5)*q.k6
			+(-j2qj-k2qk)*q.i6
			+(i1qj+u1qk)*q.j5
			+(k2qj-j2qk)*q.u6
			+(k3qk3+kqk+jqj+iqi+u2qu2+k1qk1+j1qj1+i3qi3+u3qu3+k2qk2+j2qj2+i2qi2+u6qu6+k5qk5+j5qj5+i5qi5+u5qu5+k4qk4+j4qj4+u4qu4+u7qu7+k6qk6+j6qj6+i6qi6+j7qj7+i7qi7+k7qk7+u1qu1+i1qi1+j3qj3)*q.i4
			+(j7qj4+k7qk4-j6qj5-k6qk5)*q.i7
			+(i2qj+u2qk-j7qi5+k7qu5+i7qj5-u7qk5)*q.j6
		),
		// J4
		(sum + 2*j4j4)*j4+2*(
			-(-u5qj1+j5qu1+k5qi1-u6qj2-i6qk2+j6qu2+k6qi2+u7qj3+i7qk3-j7qu3-k7qi3-i5qk1+k1qi5+j*q.u4+iqk4-u4*q.j-kqi4-i1qk5-u1qj5+j1qu5-u2qj6-i2qk6+j2qu6+k2qi6+u3qj7+i3qk7-j3qu7-k3qi7+i4qk-k4qi)*q.r
			+(iqj5-kqu5+u1qk4-j1qi4-u2qk7+j2qi7-i3qj6+k3qu6)*q.i1
			+(-j7qi+u7qk-k6qu1+i6qj1+iqj7-kqu7+u1qk6+i1qj6-j1qi6+k1qu6-u2qk5-j2qi5)*q.i3
			+(-iqi6-kqk6-u1qu7-i1qi7-j1qj7-k1qk7+i2qi4+k2qk4+u3qu5+i3qi5+j3qj5+k3qk5)*q.j2
			+(k7qi-i7qk+k6qi1-i6qk1-iqk7+kqi7+u1qj6-i1qk6+j1qu6+k1qi6-u2qj5-j2qu5)*q.u3
			+(j2qi-u2qk-k7qu5+j7qi5-i7qj5+u7qk5)*q.i6
			+(-k2qi+i2qk)*q.u6
			+(u2qi+j2qk-k7qj5+j7qk5+i7qu5-u7qi5)*q.k6
			+(-i2qi-k2qk)*q.j6
			+(j1qi-u1qk)*q.i5
			+(k7qi4-i7qk4+k6qi5-i6qk5)*q.u7
			+(u1qi+j1qk)*q.k5
			+(i7qu1-u7qi1-k7qj1+j7qk1+iqu6+kqj6-u1qi7+i1qu7+j1qk7-k1qj7-u2qi4-j2qk4)*q.k2
			+(-k1qi+i1qk)*q.u5
			+(-u7qi-j7qk+i6qu1+k6qj1+iqu7+kqj7-u1qi6-i1qu6-j1qk6+k1qj6+u2qi5-j2qk5)*q.k3
			+(-iqk5+kqi5-i1qk4+k1qi4-u2qj7+j2qu7-u3qj6+j3qu6)*q.u1
			+(i7qi+k7qk-i6qi1-k6qk1-iqi7-kqk7-u1qu6+i1qi6+j1qj6+k1qk6+u2qu5-j2qj5)*q.j3
			+(-k7qu1+j7qi1-i7qj1+u7qk1+iqj6-kqu6+u1qk7-i1qj7+j1qi7-k1qu7+u2qk4-j2qi4)*q.i2
			+(iqu5+kqj5-u1qi4-j1qk4+u2qi7+j2qk7-i3qu6-k3qj6)*q.k1
			+(-iqk6+kqi6+u1qj7+i1qk7-j1qu7-k1qi7-i2qk4+k2qi4+u3qj5+i3qk5-j3qu5-k3qi5)*q.u2
			+(u5qu5+k4qk4+i4qi4+k6qk6+j6qj6+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+i7qi7+u7qu7+k7qk7+j7qj7+k3qk3+jqj+iqi+k1qk1+j1qj1+u1qu1+i1qi1+kqk+j2qj2+i2qi2+u2qu2+u4qu4+j3qj3+i3qi3+u3qu3+k2qk2)*q.j4
			+(i7qi4+k7qk4-i6qi5-k6qk5)*q.j7
			+(-iqi5-kqk5+i1qi4+k1qk4+u2qu7+j2qj7-u3qu6-j3qj6)*q.j1
			+(-j7qi4+u7qk4-k6qu5+i6qj5)*q.i7
			+(-u7qi4-j7qk4+i6qu5+k6qj5)*q.k7
			-q.j5*(i1qi+k1qk)
		),
		// K4
		(sum + 2*k4k4)*k4+2*(
			(j7qu1+k7qi1-u7qj1-i7qk1+iqk6+jqu6-u1qj7-i1qk7+j1qu7+k1qi7-u2qj4-k2qi4)*q.i2
			+(-iqu5+jqk5+u1qi4-k1qj4-u2qi7+k2qj7+i3qu6-j3qk6)*q.j1
			+(-k7qi-u7qj+j6qu1+i6qk1+iqk7+jqu7-u1qj6+i1qk6-j1qu6-k1qi6+u2qj5-k2qi5)*q.i3
			+(iqk5+jqu5-u1qj4-k1qi4+u2qj7+k2qi7-i3qk6-j3qu6)*q.i1
			+(u7qi-k7qj-i6qu1+j6qk1-iqu7+jqk7+u1qi6+i1qu6+j1qk6-k1qj6-u2qi5-k2qj5)*q.j3
			+(i7qi+j7qj-i6qi1-j6qj1-iqi7-jqj7-u1qu6+i1qi6+j1qj6+k1qk6+u2qu5-k2qk5)*q.k3
			+(iqj6-jqi6+u1qk7-i1qj7+j1qi7-k1qu7+i2qj4-j2qi4+u3qk5-i3qj5+j3qi5-k3qu5)*q.u2
			+(j1qi-i1qj)*q.u5
			+(k1qi+u1qj)*q.i5
			+(-u1qi+k1qj)*q.j5
			-(k1qu5+j4qi-iqj4+jqi4-u1qk5+i1qj5-j1qi5-u2qk6+i2qj6-j2qi6+k2qu6+u3qk7-i3qj7+j3qi7-k3qu7-i4qj+i5qj1-j5qi1+k5qu1-u6qk2+i6qj2-j6qi2+u7qk3-i7qj3+j7qi3+k6qu2-k7qu3+k*q.u4-u4*q.k-u5qk1)*q.r
			+(-j7qi+i7qj-j6qi1+i6qj1+iqj7-jqi7+u1qk6+i1qj6-j1qi6+k1qu6-u2qk5-k2qu5)*q.u3
			-q.k5*(i1qi+j1qj)
			+(iqj5-jqi5+i1qj4-j1qi4-u2qk7+k2qu7-u3qk6+k3qu6)*q.u1
			+(-iqi6-jqj6-u1qu7-i1qi7-j1qj7-k1qk7+i2qi4+j2qj4+u3qu5+i3qi5+j3qj5+k3qk5)*q.k2
			+(-iqi5-jqj5+i1qi4+j1qj4+u2qu7+k2qk7-u3qu6-k3qk6)*q.k1
			+(-j7qi4+i7qj4-j6qi5+i6qj5)*q.u7
			+(-k7qi4-u7qj4+j6qu5+i6qk5)*q.i7
			+(u7qi4-k7qj4-i6qu5+j6qk5)*q.j7
			+(i7qi4+j7qj4-i6qi5-j6qj5)*q.k7
			+(j2qi-i2qj)*q.u6
			+(i1qi1+kqk+jqj+iqi+u2qu2+k1qk1+u1qu1+i3qi3+u3qu3+k2qk2+j2qj2+i2qi2+u4qu4+k3qk3+j1qj1+j3qj3+i4qi4+j4qj4+j5qj5+i5qi5+u5qu5+k5qk5+k6qk6+j6qj6+i6qi6+u6qu6+k7qk7+j7qj7+i7qi7+u7qu7)*q.k4
			+(j7qu5+k2qi+u2qj-u7qj5+k7qi5-i7qk5)*q.i6
			+(-u2qi+k7qj5+u7qi5-i7qu5+k2qj-j7qk5)*q.j6
			+(-i2qi-j2qj)*q.k6
			+(-i7qu1+u7qi1+k7qj1-j7qk1-iqu6+jqk6+u1qi7-i1qu7-j1qk7+k1qj7+u2qi4-k2qj4)*q.j2
		),
		// U5
		(sum + 2*u5u5)*u5+2*(
			-q.i5*(kqj-jqk)
			+(-k6qi+i6qk+k7qi1-i7qk1+iqk6+jqu6-kqi6-i1qk7+j1qu7+k1qi7-u2qj4-u3qj5)*q.j3
			+(k6qj-j6qk-k7qj1+j7qk1+iqu6-jqk6+kqj6+i1qu7+j1qk7-k1qj7-u2qi4-u3qi5)*q.i3
			+(u3qk+j7qi4-i7qj4+j6qi5-i6qj5)*q.k6
			+(-k7qi+i7qk-k6qi1+i6qk1+iqk7+jqu7-kqi7+i1qk6-j1qu6-k1qi6+u2qj5-u3qj4)*q.j2
			+(iqk4-kqi4-i1qk5+k1qi5-u2qj6+j2qu6+u3qj7-j3qu7)*q.j1
			+(k7qj-j7qk+k6qj1-j6qk1+iqu7-jqk7+kqj7-i1qu6-j1qk6+k1qj6+u2qi5-u3qi4)*q.i2
			+(-iqi6-jqj6-kqk6-i1qi7-j1qj7-k1qk7+i2qi4+j2qj4+k2qk4+i3qi5+j3qj5+k3qk5)*q.u3
			+(-jqk4+kqj4+j1qk5-k1qj5-u2qi6+i2qu6+u3qi7-i3qu7)*q.i1
			+(j6qi-i6qj-j7qi1+i7qj1-iqj6+jqi6+kqu6+i1qj7-j1qi7+k1qu7-u2qk4-u3qk5)*q.k3
			-(-j6qj3+u7qu2+i7qi2+j7qj2+k7qk2+u1*q.u4-u4*q.u1-i5qi-k6qk3+iqi5+j4qj1+jqj5+kqk5-i1qi4-j1qj4-k1qk4-u2qu7-i2qi7-j2qj7-k2qk7+u3qu6+i3qi6+j3qj6+k3qk6+i4qi1+k4qk1-j5qj-k5qk-u6qu3-i6qi3)*q.r
			+(j7qi-i7qj+j6qi1-i6qj1-iqj7+jqi7+kqu7-i1qj6+j1qi6-k1qu6+u2qk5-u3qk4)*q.k2
			+(u2qj-k6qi4+i6qk4+k7qi5-i7qk5)*q.j7
			+(u2qi+k6qj4-k7qj5-j6qk4+j7qk5)*q.i7
			+(-iqi7-jqj7-kqk7+i1qi6+j1qj6+k1qk6-i2qi5-j2qj5-k2qk5+i3qi4+j3qj4+k3qk4)*q.u2
			+(-j1qi+i1qj)*q.k4
			+(-i2qi-j2qj-k2qk)*q.u7
			+(k1qi-i1qk)*q.j4
			+(-k1qj+j1qk)*q.i4
			+(-jqi+iqj)*q.k5
			+(-iqk+kqi)*q.j5
			+(u3qj-k7qi4+i7qk4-k6qi5+i6qk5)*q.j6
			+(u3qi+k7qj4-j7qk4+k6qj5-j6qk5)*q.i6
			+(-i3qi-j3qj-k3qk)*q.u6
			+(u2qk+j6qi4-i6qj4-j7qi5+i7qj5)*q.k7
			+(k3qk3+kqk+jqj+u3qu3+k2qk2+j2qj2+i2qi2+u2qu2+k1qk1+j1qj1+u1qu1+i1qi1+j3qj3+i3qi3+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+k4qk4+j4qj4+i4qi4+u4qu4+k6qk6+j6qj6+iqi+k7qk7+j7qj7+i7qi7+u7qu7)*q.u5
			+(-iqj4+jqi4+i1qj5-j1qi5-u2qk6+k2qu6+u3qk7-k3qu7)*q.k1
		),
		// I5
		(sum + 2*i5i5)*i5+2*(
			(-k6qj+j6qk+k7qj1-j7qk1-iqu6+jqk6-kqj6+u1qi7-j1qk7+k1qj7+u2qi4-i3qu5)*q.u3
			+(-j7qj-k7qk+j6qj1+k6qk1+iqi7+jqj7+kqk7+u1qu6-j1qj6-k1qk6-u2qu5-i3qi4)*q.i2
			+(-j6qi+i6qj-k7qu1+u7qk1+iqj6-jqi6-kqu6+u1qk7+j1qi7-k1qu7+u2qk4-i3qj5)*q.j3
			+(-k6qi+i6qk+j7qu1-u7qj1+iqk6+jqu6-kqi6-u1qj7+j1qu7+k1qi7-u2qj4-i3qk5)*q.k3
			+(jqk4-kqj4-j1qk5+k1qj5+u2qi6-i2qu6-u3qi7+i3qu7)*q.u1
			+(i3qj+j7qi4-u7qk4+k6qu5-i6qj5)*q.j6
			+(k7qi+u7qj-j6qu1-i6qk1-iqk7-jqu7+kqi7+u1qj6+j1qu6+k1qi6-u2qj5-i3qk4)*q.k2
			+(iqu7-jqk7+kqj7-u1qi6-j1qk6+k1qj6+i2qu5-j2qk5+k2qj5-u3qi4-j3qk4+k3qj4)*q.u2
			+(-iqi6-jqj6-kqk6-u1qu7-j1qj7-k1qk7+i2qi4+j2qj4+k2qk4+u3qu5+j3qj5+k3qk5)*q.i3
			+(jqi-iqj)*q.j5
			+(kqi-iqk)*q.k5
			+(iqj4-jqi4+u1qk5-k1qu5+u2qk6-k2qu6+i3qj7-j3qi7)*q.j1
			-(-iqu5+k7qj2-k1qj4-u6qi3+i1*q.u4-u4*q.i1+j1qk4-kqj5-u2qi7+i2qu7-j2qk7+k2qj7+u1qi4+u5qi-j7qk2+jqk5-u3qi6+i3qu6-i4qu1+i6qu3+i7qu2+j4qk1-k4qj1+j5qk-j3qk6-k5qj-j6qk3+k6qj3-u7qi2+k3qj6)*q.r
			+(iqk4-kqi4-u1qj5+j1qu5-u2qj6+j2qu6+i3qk7-k3qi7)*q.k1
			+(j7qi-u7qk+k6qu1-i6qj1-iqj7+jqi7+kqu7-u1qk6+j1qi6-k1qu6+u2qk5-i3qj4)*q.j2
			+(i6qi6+u6qu6+k7qk7+i7qi7+u7qu7+k6qk6+k2qk2+j2qj2+k5qk5+j5qj5+u5qu5+k4qk4+j4qj4+i4qi4+u4qu4+j6qj6+j7qj7+iqi+u1qu1+i1qi1+kqk+jqj+i2qi2+u2qu2+k1qk1+j1qj1+k3qk3+j3qj3+i3qi3+u3qu3)*q.i5
			+(u2qj-k6qi4+i6qk4+j7qu5-u7qj5)*q.k7
			+(-u2qi+k7qj5-k6qj4+j6qk4-j7qk5)*q.u7
			+(-i2qi-j2qj-k2qk)*q.i7
			+(-j1qi+u1qk)*q.j4
			+(-k1qi-u1qj)*q.k4
			+(j1qj+k1qk)*q.i4
			-q.u5*(jqk-kqj)
			+(i3qk+k7qi4+u7qj4-j6qu5-i6qk5)*q.k6
			+(u3qi-k3qj+j3qk)*q.u6
			+(i3qi-j7qj4-k7qk4+j6qj5+k6qk5)*q.i6
			+(-u2qk-j6qi4+i6qj4-k7qu5+u7qk5)*q.j7
		),
		// J5
		(sum + 2*j5j5)*j5+2*(
			(i7qj+u7qk-k6qu1-j6qi1+iqj7-jqi7-kqu7+u1qk6+i1qj6+k1qu6-u2qk5-j3qi4)*q.i2
			+(u2qu2+k1qk1+j1qj1+u1qu1+u6qu6+k5qk5+i5qi5+u5qu5+k4qk4+j4qj4+i4qi4+u4qu4+k6qk6+j6qj6+i6qi6+k7qk7+j7qj7+i7qi7+u7qu7+i1qi1+kqk+jqj+iqi+k3qk3+j3qj3+i3qi3+u3qu3+k2qk2+j2qj2+i2qi2)*q.j5
			+(-u2qi-k6qj4-i7qu5+u7qi5+j6qk4)*q.k7
			+(-i2qi-j2qj-k2qk)*q.j7
			+(-i1qj-u1qk)*q.i4
			+(u1qi-k1qj)*q.k4
			+(i1qi+k1qk)*q.j4
			+(-jqi+iqj)*q.i5
			+(iqk-kqi)*q.u5
			+q.k5*(-jqk+kqj)
			-(u1qj4-j4qu1+j7qu2-iqk5+kqi5-i1qk4+k1qi4-u2qj7+i2qk7+j2qu7-k2qi7-u3qj6+i3qk6+j3qu6-k3qi6-i4qk1+k4qi1-i5qk+k5qi-u6qj3+i6qk3+j6qu3-k6qi3-u7qj2+i7qk2-k7qi2-jqu5+j1*q.u4-u4*q.j1+u5qj)*q.r
			+(-i7qi-k7qk+i6qi1+k6qk1+iqi7+jqj7+kqk7+u1qu6-i1qi6-k1qk6-u2qu5-j3qj4)*q.j2
			+(k6qi-i6qk-k7qi1+i7qk1-iqk6-jqu6+kqi6+u1qj7+i1qk7-k1qi7+u2qj4-j3qu5)*q.u3
			+(j6qi-i6qj+k7qu1-u7qk1-iqj6+jqi6+kqu6-u1qk7+i1qj7+k1qu7-u2qk4-j3qi5)*q.i3
			+(iqk7+jqu7-kqi7-u1qj6+i1qk6-k1qi6+i2qk5+j2qu5-k2qi5-u3qj4+i3qk4-k3qi4)*q.u2
			+(-u7qi+k7qj+i6qu1-j6qk1+iqu7-jqk7+kqj7-u1qi6-i1qu6+k1qj6+u2qi5-j3qk4)*q.k2
			+(-iqi6-jqj6-kqk6-u1qu7-i1qi7-k1qk7+i2qi4+j2qj4+k2qk4+u3qu5+i3qi5+k3qk5)*q.j3
			+(-iqk4+kqi4+i1qk5-k1qi5+u2qj6-j2qu6-u3qj7+j3qu7)*q.u1
			+(jqk4-kqj4+u1qi5-i1qu5+u2qi6-i2qu6+j3qk7-k3qj7)*q.k1
			+(-k6qj+j6qk-i7qu1+u7qi1-iqu6+jqk6-kqj6+u1qi7-i1qu7+k1qj7+u2qi4-j3qk5)*q.k3
			+(j3qi+i7qj4+u7qk4-k6qu5-j6qi5)*q.i6
			+(-iqj4+jqi4-u1qk5+k1qu5-u2qk6+k2qu6-i3qj7+j3qi7)*q.i1
			+(k3qi+u3qj-i3qk)*q.u6
			+(j3qk-u7qi4+k7qj4+i6qu5-j6qk5)*q.k6
			+(j3qj-i7qi4-k7qk4+i6qi5+k6qk5)*q.j6
			+(u2qk+j6qi4-i6qj4+k7qu5-u7qk5)*q.i7
			+(-u2qj+k6qi4-i6qk4-k7qi5+i7qk5)*q.u7
		),
		// K5
		(sum + 2*k5k5)*k5+2*(
			(k6qi-i6qk-j7qu1+u7qj1-iqk6-jqu6+kqi6+u1qj7+i1qk7-j1qu7+u2qj4-k3qi5)*q.i3
			+(k6qj-j6qk+i7qu1-u7qi1+iqu6-jqk6+kqj6-u1qi7+i1qu7+j1qk7-u2qi4-k3qj5)*q.j3
			+(-iqi6-jqj6-kqk6-u1qu7-i1qi7-j1qj7+i2qi4+j2qj4+k2qk4+u3qu5+i3qi5+j3qj5)*q.k3
			+(u7qi+j7qk-i6qu1-k6qj1-iqu7+jqk7-kqj7+u1qi6+i1qu6+j1qk6-u2qi5-k3qj4)*q.j2
			+(-u7qj+i7qk+j6qu1-k6qi1+iqk7+jqu7-kqi7-u1qj6+i1qk6-j1qu6+u2qj5-k3qi4)*q.i2
			+(-iqk4+kqi4+u1qj5-j1qu5+u2qj6-j2qu6-i3qk7+k3qi7)*q.i1
			+(-i7qi-j7qj+i6qi1+j6qj1+iqi7+jqj7+kqk7+u1qu6-i1qi6-j1qj6-u2qu5-k3qk4)*q.k2
			+(-j6qi+i6qj+j7qi1-i7qj1+iqj6-jqi6-kqu6+u1qk7-i1qj7+j1qi7+u2qk4-k3qu5)*q.u3
			+(-jqk4+kqj4-u1qi5+i1qu5-u2qi6+i2qu6-j3qk7+k3qj7)*q.j1
			+(-u1qi-j1qk)*q.j4
			+(iqk-kqi)*q.i5
			+q.j5*(jqk-kqj)
			-(-kqu5+j7qi2+iqj5+u1qk4-jqi5+i1qj4-j1qi4-u2qk7-i2qj7+j2qi7+k2qu7-u3qk6-i3qj6+j3qi6+k3qu6+i4qj1-j4qi1-k4qu1+i5qj-j5qi-u6qk3-i6qj3+j6qi3+k6qu3-u7qk2-i7qj2+k7qu2+u5qk+k1*q.u4-u4*q.k1)*q.r
			+(-iqj7+jqi7+kqu7-u1qk6-i1qj6+j1qi6-i2qj5+j2qi5+k2qu5-u3qk4-i3qj4+j3qi4)*q.u2
			+(-u2qk-j6qi4+i6qj4+j7qi5-i7qj5)*q.u7
			+(-u2qj+k6qi4-i6qk4-j7qu5+u7qj5)*q.i7
			+(u2qi-u7qi5+i7qu5+k6qj4-j6qk4)*q.j7
			+(-i2qi-j2qj-k2qk)*q.k7
			+(u1qj-i1qk)*q.i4
			+(i1qi+j1qj)*q.k4
			+(jqi-iqj)*q.u5
			+(-j3qi+i3qj+u3qk)*q.u6
			+(k3qi-u7qj4+i7qk4+j6qu5-k6qi5)*q.i6
			+(iqj4-jqi4-i1qj5+j1qi5+u2qk6-k2qu6-u3qk7+k3qu7)*q.u1
			+(k3qj+u7qi4+j7qk4-i6qu5-k6qj5)*q.j6
			+(k3qk-i7qi4-j7qj4+i6qi5+j6qj5)*q.k6
			+(kqk+jqj+iqi+i3qi3+u3qu3+k2qk2+j2qj2+u2qu2+j1qj1+i2qi2+k1qk1+u1qu1+i1qi1+i4qi4+u4qu4+k3qk3+j3qj3+u5qu5+k4qk4+j4qj4+u7qu7+k6qk6+j6qj6+i6qi6+u6qu6+j5qj5+i5qi5+k7qk7+j7qj7+i7qi7)*q.k5
		),
		// U6
		(sum + 2*u6u6)*u6+2*(
			(-iqk5-jqu5+kqi5+u1qj4-i1qk4+k1qi4+i2qk7+j2qu7-k2qi7-u3qj6+i3qk6-k3qi6)*q.j3
			+(iqi5+jqj5+kqk5-i1qi4-j1qj4-k1qk4-i2qi7-j2qj7-k2qk7+i3qi6+j3qj6+k3qk6)*q.u3
			+(i3qi3+u3qu3+k2qk2+j5qj5+i5qi5+j7qj7+i7qi7+u7qu7+k6qk6+j6qj6+i6qi6+k5qk5+k7qk7+i1qi1+kqk+jqj+iqi+u2qu2+k1qk1+j1qj1+u1qu1+j2qj2+i2qi2+u5qu5+k4qk4+j4qj4+i4qi4+u4qu4+k3qk3+j3qj3)*q.u6
			+(k2qi-i2qk)*q.j4
			+(-k2qj+j2qk)*q.i4
			+(j1qi-i1qj-u1qk)*q.k7
			+(-k1qi+i1qk-u1qj)*q.j7
			+(-u1qi-j1qk+k1qj)*q.i7
			+(i1qi+k1qk+j1qj)*q.u7
			+(kqi-iqk)*q.j6
			+(-jqi+iqj)*q.k6
			-(-i3qi5+i5qi3+iqi6+j4qj2+u5qu3+jqj6+k4qk2+kqk6+u1qu7+i1qi7+j1qj7+k1qk7-i2qi4-j2qj4-k2qk4-u3qu5-j3qj5-k3qk5+i4qi2+j5qj3+k5qk3-i6qi-j6qj-k6qk-u7qu1-i7qi1-j7qj1-k7qk1-u4*q.u2+u2*q.u4)*q.r
			+(iqk4-kqi4-u1qj5-i1qk5+j1qu5+k1qi5-i2qk6+k2qi6+u3qj7+i3qk7-j3qu7-k3qi7)*q.j2
			+(iqj7-jqi7-kqu7+u1qk6+i1qj6-j1qi6+i2qj5-j2qi5-k2qu5+u3qk4+i3qj4-j3qi4)*q.k1
			+(-iqk7-jqu7+kqi7+u1qj6-i1qk6+k1qi6-i2qk5-j2qu5+k2qi5+u3qj4-i3qk4+k3qi4)*q.j1
			+(-iqu7+jqk7-kqj7+u1qi6+j1qk6-k1qj6-i2qu5+j2qk5-k2qj5+u3qi4+j3qk4-k3qj4)*q.i1
			+(iqi7+jqj7+kqk7-i1qi6-j1qj6-k1qk6+i2qi5+j2qj5+k2qk5-i3qi4-j3qj4-k3qk4)*q.u1
			+(iqj5-jqi5-kqu5+u1qk4+i1qj4-j1qi4-i2qj7+j2qi7+k2qu7-u3qk6-i3qj6+j3qi6)*q.k3
			+(-jqk4+kqj4-u1qi5+i1qu5+j1qk5-k1qj5+j2qk6-k2qj6+u3qi7-i3qu7-j3qk7+k3qj7)*q.i2
			+(j3qi-i3qj-u3qk)*q.k5
			+(-k3qi-u3qj+i3qk)*q.j5
			+(-u3qi+k3qj-j3qk)*q.i5
			+(i3qi+j3qj+k3qk)*q.u5
			+(-j2qi+i2qj)*q.k4
			+(-iqu5+jqk5-kqj5+u1qi4+j1qk4-k1qj4+i2qu7-j2qk7+k2qj7-u3qi6-j3qk6+k3qj6)*q.i3
			-q.i6*(-jqk+kqj)
			+(-iqj4+jqi4-u1qk5+i1qj5-j1qi5+k1qu5+i2qj6-j2qi6+u3qk7-i3qj7+j3qi7-k3qu7)*q.k2
		),
		// I6
		(sum + 2*i6i6)*i6+2*(
			u1qi*q.u7
			+i1qi*q.i7
			+j1qi*q.j7
			+k1qi*q.k7
			-(j2qk4-j1qk7+j6qk-u4*q.i2-k4qj2+jqk6+k1qj7+i2*q.u4-j3qk5+k3qj5-i5qu3-k6qj-j7qk1+j4qk2-k2qj4+k5qj3-kqj6-j5qk3-i3qu5-iqu6+u5qi3-i7qu1+u7qi1+u1qi7-i1qu7+u2qi4-i4qu2+u6qi+u3qi5+k7qj1)*q.r
			+(-iqj7-k1qu6+u2qk5+j2qi5+k2qu5-u3qk4-i3qj4+j3qi4+j4qi3+k4qu3-u5qk2-i5qj2)*q.j1
			+(i5qj+u5qk+iqj5-j1qi4-u2qk7+j2qi7+k2qu7-u3qk6-i3qj6+k3qu6)*q.j3
			+(-iqk7+j1qu6-u2qj5-j2qu5+k2qi5+u3qj4-i3qk4+k3qi4-j4qu3+k4qi3+u5qj2-i5qk2)*q.k1
			+(iqj4-jqi4+u2qk6-k2qu6-u3qk7+i3qj7-j3qi7+k3qu7)*q.j2
			+(-iqu7+i1qu6-u2qi5+j2qk5-k2qj5+u3qi4+j3qk4-k3qj4+j4qk3-k4qj3+j5qk2-k5qj2)*q.u1
			+(-iqi7-u1qu6+u2qu5-j2qj5-k2qk5+i3qi4+j3qj4+k3qk4-j4qj3-k4qk3+j5qj2+k5qk2)*q.i1
			+(jqk4-kqj4+u1qi5-i1qu5-j1qk5+k1qj5-j2qk6+k2qj6-u3qi7+i3qu7+j3qk7-k3qj7)*q.u2
			+(-u5qj+i5qk+iqk5-k1qi4+u2qj7-j2qu7+k2qi7+u3qj6-i3qk6-j3qu6)*q.k3
			+(k5qj-j5qk+iqu5-u1qi4+u2qi7+j2qk7-k2qj7-i3qu6+j3qk6-k3qj6)*q.u3
			+(-j3qi+i3qj+u3qk+j4qi7+k4qu7-u5qk6-i5qj6)*q.j5
			+(-j5qj-k5qk+iqi5-i1qi4-u2qu7-j2qj7-k2qk7+u3qu6+j3qj6+k3qk6)*q.i3
			+(iqk4-kqi4-u2qj6+j2qu6+u3qj7+i3qk7-j3qu7-k3qi7)*q.k2
			+(-i3qi-j3qj-k3qk-j4qj7-k4qk7+j5qj6+k5qk6)*q.i5
			+(jqj+iqi+j2qj2+i2qi2+u2qu2+k1qk1+j1qj1+u1qu1+i1qi1+kqk+j3qj3+i3qi3+u3qu3+k2qk2+u4qu4+k3qk3+u5qu5+k4qk4+j4qj4+i4qi4+i5qi5+j6qj6+u6qu6+k5qk5+j5qj5+k6qk6+u7qu7+k7qk7+j7qj7+i7qi7)*q.i6
			+(-j2qi-j5qi7+u2qk+k5qu7-u5qk7+i5qj7)*q.j4
			+(-k2qi-j5qu7-u2qj-k5qi7+u5qj7+i5qk7)*q.k4
			+(j2qj+k2qk)*q.i4
			+(jqi-iqj)*q.j6
			+(kqi-iqk)*q.k6
			-q.u6*(-kqj+jqk)
			+(-k3qi-u3qj+i3qk-j4qu7+k4qi7+u5qj6-i5qk6)*q.k5
			+(-u3qi+k3qj-j3qk+j4qk7-k4qj7+j5qk6-k5qj6)*q.u5
		),
		// J6
		(sum + 2*j6j6)*j6+2*(
			u1qj*q.u7
			+i1qj*q.i7
			+j1qj*q.j7
			+k1qj*q.k7
			+(-k5qi+i5qk+jqu5-u1qj4+u2qj7-i2qk7+k2qi7-i3qk6-j3qu6+k3qi6)*q.u3
			+(u5qi+j5qk+jqk5-k1qj4-u2qi7+i2qu7+k2qj7-u3qi6+i3qu6-j3qk6)*q.k3
			+(j5qi-u5qk+jqi5-i1qj4+u2qk7+i2qj7-k2qu7+u3qk6-j3qi6-k3qu6)*q.i3
			+(jqk4-kqj4+u2qi6-i2qu6-u3qi7+i3qu7+j3qk7-k3qj7)*q.k2
			+(-jqu7+j1qu6-u2qj5-i2qk5+k2qi5+u3qj4-i3qk4+k3qi4-i4qk3+k4qi3-i5qk2+k5qi2)*q.u1
			+(-iqk4+kqi4+u1qj5+i1qk5-j1qu5-k1qi5+i2qk6-k2qi6-u3qj7-i3qk7+j3qu7+k3qi7)*q.u2
			+(-jqk7-i1qu6+u2qi5+i2qu5+k2qj5-u3qi4-j3qk4+k3qj4+i4qu3+k4qj3-u5qi2-j5qk2)*q.k1
			+(-i5qi-k5qk+jqj5-j1qj4-u2qu7-i2qi7-k2qk7+u3qu6+i3qi6+k3qk6)*q.j3
			+(-jqj7-u1qu6+u2qu5-i2qi5-k2qk5+i3qi4+j3qj4+k3qk4-i4qi3-k4qk3+i5qi2+k5qk2)*q.j1
			+(u3qi-k3qj+j3qk+i4qu7+k4qj7-u5qi6-j5qk6)*q.k5
			+(-jqi7+k1qu6-u2qk5+i2qj5-k2qu5+u3qk4+i3qj4-j3qi4+i4qj3-k4qu3+u5qk2-j5qi2)*q.i1
			+(-i3qi-j3qj-k3qk-i4qi7-k4qk7+i5qi6+k5qk6)*q.j5
			+(i2qi2+u2qu2+k1qk1+u1qu1+i1qi1+kqk+jqj+iqi+i3qi3+u3qu3+k2qk2+j2qj2+u5qu5+k4qk4+j4qj4+i4qi4+u4qu4+k3qk3+j3qj3+k6qk6+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+i7qi7+u7qu7+j7qj7+k7qk7+j1qj1)*q.j6
			+(-i2qj-u2qk+j5qi7-k5qu7+u5qk7-i5qj7)*q.i4
			+(u2qi-k2qj-k5qj7+i5qu7+j5qk7-u5qi7)*q.k4
			+(i2qi+k2qk)*q.j4
			+(-kqi+iqk)*q.u6
			+q.k6*(kqj-jqk)
			+(-jqi+iqj)*q.i6
			-(-iqk6-j4qu2-jqu6+u1qj7-j1qu7+u2qj4-i2qk4+k2qi4+u3qj5+i3qk5-k3qi5-i4qk2+k4qi2+i5qk3-j5qu3-k5qi3+u6qj+u7qj1-j7qu1+i7qk1-u4*q.j2-i6qk+u5qj3+kqi6+i1qk7-k1qi7+j2*q.u4+k6qi-j3qu5-k7qi1)*q.r
			+(-iqj4+jqi4-u2qk6+k2qu6+u3qk7-i3qj7+j3qi7-k3qu7)*q.i2
			+(-k3qi-u3qj+i3qk-i4qk7+k4qi7-i5qk6+k5qi6)*q.u5
			+(j3qi-i3qj-u3qk+i4qj7-k4qu7+u5qk6-j5qi6)*q.i5
		),
		// K6
		(sum + 2*k6k6)*k6+2*(
			u1qk*q.u7
			+i1qk*q.i7
			+j1qk*q.j7
			+k1qk*q.k7
			+(j5qi-i5qj+kqu5-u1qk4+u2qk7+i2qj7-j2qi7+i3qj6-j3qi6-k3qu6)*q.u3
			+(-kqu7+k1qu6-u2qk5+i2qj5-j2qi5+u3qk4+i3qj4-j3qi4+i4qj3-j4qi3+i5qj2-j5qi2)*q.u1
			+(j3qi-i3qj-u3qk+i4qj7-j4qi7+i5qj6-j5qi6)*q.u5
			+(-u5qi+k5qj+kqj5-j1qk4+u2qi7-i2qu7+j2qk7+u3qi6-i3qu6-k3qj6)*q.j3
			+(k5qi+u5qj+kqi5-i1qk4-u2qj7+i2qk7+j2qu7-u3qj6+j3qu6-k3qi6)*q.i3
			+(-i5qi-j5qj+kqk5-k1qk4-u2qu7-i2qi7-j2qj7+u3qu6+i3qi6+j3qj6)*q.k3
			+(-kqi+iqk)*q.i6
			+q.j6*(-kqj+jqk)
			-(i6qj-j4qi2-kqu6-i1qj7+j1qi7+u2qk4-i3qj5-k4qu2+j5qi3+u6qk-i7qj1+j7qi1-j6qi-jqi6-k7qu1+j3qi5-k3qu5+u5qk3+iqj6+u1qk7-k1qu7+i2qj4-j2qi4-i5qj3+k2*q.u4+u3qk5-u4*q.k2+i4qj2+u7qk1-k5qu3)*q.r
			+(-iqk4+kqi4+u2qj6-j2qu6-u3qj7-i3qk7+j3qu7+k3qi7)*q.i2
			+(-kqi7-j1qu6+u2qj5+i2qk5+j2qu5-u3qj4+i3qk4-k3qi4+i4qk3+j4qu3-u5qj2-k5qi2)*q.i1
			+(-jqk4+kqj4-u2qi6+i2qu6+u3qi7-i3qu7-j3qk7+k3qj7)*q.j2
			+(-kqj7+i1qu6-u2qi5-i2qu5+j2qk5+u3qi4+j3qk4-k3qj4-i4qu3+j4qk3+u5qi2-k5qj2)*q.j1
			+(iqj4-jqi4+u1qk5-i1qj5+j1qi5-k1qu5-i2qj6+j2qi6-u3qk7+i3qj7-j3qi7+k3qu7)*q.u2
			+(-kqk7-u1qu6+u2qu5-i2qi5-j2qj5+i3qi4+j3qj4+k3qk4-i4qi3-j4qj3+i5qi2+j5qj2)*q.k1
			+(-i3qi-j3qj-k3qk-i4qi7-j4qj7+i5qi6+j5qj6)*q.k5
			+(u2qj+k5qi7-i2qk+j5qu7-u5qj7-i5qk7)*q.i4
			+(j7qj7+i7qi7+u7qu7+iqi+i1qi1+kqk+jqj+k7qk7+j1qj1+i2qi2+u2qu2+k1qk1+i4qi4+u4qu4+k3qk3+j3qj3+i3qi3+u3qu3+k2qk2+j2qj2+j5qj5+i5qi5+u5qu5+k4qk4+j4qj4+j6qj6+i6qi6+u6qu6+k5qk5+u1qu1)*q.k6
			+(-u2qi-j2qk+u5qi7-i5qu7-j5qk7+k5qj7)*q.j4
			+(i2qi+j2qj)*q.k4
			+(jqi-iqj)*q.u6
			+(k3qi+u3qj-i3qk+i4qk7+j4qu7-u5qj6-k5qi6)*q.i5
			+(-u3qi+k3qj-j3qk-i4qu7+j4qk7+u5qi6-k5qj6)*q.j5
		),
		// U7
		(sum + 2*u7u7)*u7+2*(
			-u1qi*q.i6
			-u1qj*q.j6
			-u1qk*q.k6
			+(-k4qj+j4qk+u1qi5-i1qu5+u2qi6-i2qu6-j2qk6+k2qj6+j3qk7-k3qj7)*q.i3
			+(k5qi-i5qk-jqu5+u1qj4-u2qj7+i2qk7-k2qi7+i3qk6+j3qu6-k3qi6)*q.j2
			+(-k5qj+j5qk-iqu5+u1qi4-u2qi7-j2qk7+k2qj7+i3qu6-j3qk6+k3qj6)*q.i2
			+(kqu6-u1qk7-u2qk4-i2qj4+j2qi4+i3qj5-j3qi5+k3qu5-i4qj2+j4qi2+i5qj3-j5qi3)*q.k1
			+(k4qi-i4qk+u1qj5-j1qu5+u2qj6+i2qk6-j2qu6-k2qi6-i3qk7+k3qi7)*q.j3
			+(jqu6-u1qj7-u2qj4+i2qk4-k2qi4-i3qk5+j3qu5+k3qi5+i4qk2-k4qi2-i5qk3+k5qi3)*q.j1
			+(iqu6-u1qi7-u2qi4-j2qk4+k2qj4+i3qu5+j3qk5-k3qj5-j4qk2+k4qj2+j5qk3-k5qj3)*q.i1
			+(-j5qi+i5qj-kqu5+u1qk4-u2qk7-i2qj7+j2qi7-i3qj6+j3qi6+k3qu6)*q.k2
			-(-iqi7+i5qi2+k7qk-i2qi5+u3*q.u4+k1qk6-jqj7-u4*q.u3+k5qk2-u1qu6+i1qi6+u2qu5-j2qj5-k2qk5+i3qi4+j3qj4+k3qk4-i4qi3-j4qj3-k4qk3-u5qu2+j5qj2+u6qu1-i6qi1+i7qi+j1qj6-j6qj1-k6qk1-kqk7+j7qj)*q.r
			+(k2qi+u2qj+i4qk6-i2qk+k5qi7-k4qi6-i5qk7)*q.j5
			+(u2qi+k4qj6-k2qj+j2qk-k5qj7-j4qk6+j5qk7)*q.i5
			+(i2qi+k2qk+j2qj)*q.u5
			+(-j4qi+i4qj+u1qk5-k1qu5+u2qk6-i2qj6+j2qi6-k2qu6+i3qj7-j3qi7)*q.k3
			-q.u6*(i1qi+j1qj+k1qk)
			+(-iqi5-jqj5-kqk5+i1qi4+j1qj4+k1qk4+i2qi7+j2qj7+k2qk7-i3qi6-j3qj6-k3qk6)*q.u2
			+(iqi6+jqj6+kqk6+i1qi7+j1qj7+k1qk7-i2qi4-j2qj4-k2qk4-i3qi5-j3qj5-k3qk5)*q.u1
			+(-j3qi+i3qj-i4qj7+j4qi7-i5qj6+j5qi6)*q.k4
			+(k3qi-i3qk+i4qk7-k4qi7+i5qk6-k5qi6)*q.j4
			+(-k3qj+j3qk-j4qk7+k4qj7-j5qk6+k5qj6)*q.i4
			+(-j2qi+i2qj+u2qk-i4qj6+j4qi6+i5qj7-j5qi7)*q.k5
			+(u3qu3+k7qk7+i7qi7+j7qj7+kqk+iqi+jqj+j1qj1+i1qi1+u1qu1+j2qj2+i2qi2+u2qu2+k1qk1+i4qi4+u4qu4+k3qk3+j3qj3+i3qi3+k2qk2+j5qj5+i5qi5+u5qu5+k4qk4+j4qj4+u6qu6+k5qk5+k6qk6+j6qj6+i6qi6)*q.u7
		),
		// I7
		(sum + 2*i7i7)*i7+2*(
			-i1qi*q.i6
			-i1qj*q.j6
			-i1qk*q.k6
			+(iqu5-jqk5+kqj5-u1qi4-j1qk4+k1qj4-i2qu7+j2qk7-k2qj7+u3qi6+j3qk6-k3qj6)*q.u2
			+(k4qi-i4qk+i1qk5-k1qi5+u2qj6+i2qk6-j2qu6-k2qi6-u3qj7+j3qu7)*q.k3
			+(-iqu6-i1qu7+u2qi4+j2qk4-k2qj4+u3qi5-j3qk5+k3qj5+j4qk2-k4qj2-j5qk3+k5qj3)*q.u1
			+(jqu6-i1qk7-u2qj4+i2qk4-k2qi4-u3qj5+j3qu5+k3qi5+i4qk2-k4qi2-u5qj3+j5qu3)*q.k1
			+(iqi6+jqj6+kqk6+u1qu7+j1qj7+k1qk7-i2qi4-j2qj4-k2qk4-u3qu5-j3qj5-k3qk5)*q.i1
			+(-j5qi+u5qk-jqi5+i1qj4-u2qk7-i2qj7+k2qu7-u3qk6+j3qi6+k3qu6)*q.j2
			+(j4qi-i4qj+i1qj5-j1qi5-u2qk6+i2qj6-j2qi6+k2qu6+u3qk7-k3qu7)*q.j3
			+(k4qj-j4qk-u1qi5+i1qu5-u2qi6+i2qu6+j2qk6-k2qj6-j3qk7+k3qj7)*q.u3
			+(-k5qi-u5qj-kqi5+i1qk4+u2qj7-i2qk7-j2qu7+u3qj6-j3qu6+k3qi6)*q.k2
			-(kqj7-jqk7-u1qi6-i1qu6-j1qk6+k1qj6-j2qk5+k2qj5-u3qi4-j3qk4+k3qj4+i4qu3-j4qk3+k4qj3-u5qi2-j5qk2+u6qi1+i6qu1-j6qk1+k6qj1-u7qi-j7qk+k7qj+k5qj2+i3*q.u4-u4*q.i3-i5qu2+u2qi5+i2qu5+iqu7)*q.r
			+(j5qj+k5qk-iqi5+i1qi4+u2qu7+j2qj7+k2qk7-u3qu6-j3qj6-k3qk6)*q.i2
			+(-kqu6-i1qj7+u2qk4+i2qj4-j2qi4+u3qk5+j3qi5-k3qu5+i4qj2-j4qi2+u5qk3-k5qu3)*q.j1
			+(-u2qi+j4qk6+k2qj-j2qk+k5qj7-k4qj6-j5qk7)*q.u5
			+(i2qi+k2qk+j2qj)*q.i5
			+(u1qi-k1qj+j1qk)*q.u6
			+(-j3qi+u3qk-i4qj7+k4qu7-u5qk6+j5qi6)*q.j4
			+(-k3qi-u3qj-i4qk7-j4qu7+u5qj6+k5qi6)*q.k4
			+(j3qj+k3qk+j4qj7+k4qk7-j5qj6-k5qk6)*q.i4
			+(u3qu3+j7qj7+kqk+iqi+jqj+u7qu7+j1qj1+i1qi1+u1qu1+u2qu2+k1qk1+j2qj2+i2qi2+j3qj3+i3qi3+k2qk2+u4qu4+k7qk7+k3qk3+u5qu5+k4qk4+j4qj4+i4qi4+j6qj6+i6qi6+u6qu6+k5qk5+j5qj5+i5qi5+k6qk6)*q.i7
			+(j2qi-i2qj-u2qk+i4qj6-j4qi6+u5qk7-k5qu7)*q.j5
			+(k2qi+u2qj-k4qi6-i2qk+j5qu7+i4qk6-u5qj7)*q.k5
		),
		// J7
		(sum + 2*j7j7)*j7+2*(
			(k7qk7+iqi+jqj+i7qi7+u7qu7+u2qu2+k1qk1+j1qj1+u1qu1+k2qk2+j2qj2+i2qi2+i3qi3+u3qu3+i4qi4+u4qu4+k3qk3+j3qj3+j4qj4+k4qk4+k5qk5+j5qj5+i5qi5+u5qu5+k6qk6+j6qj6+i6qi6+u6qu6+i1qi1+kqk)*q.j7
			-(iqk7+j2qu5+i7qk-kqi7+j4qu3-u3qj4+jqu7-u1qj6-j1qu6+u2qj5+i2qk5-k2qi5+i3qk4-k3qi4+i4qk3-k4qi3+i5qk2-j5qu2-k5qi2+u6qj1+j6qu1-u7qj-u4*q.j3-k6qi1+i1qk6+j3*q.u4-k7qi-k1qi6-u5qj2+i6qk1)*q.r
			+(kqu6-j1qi7-u2qk4-i2qj4+j2qi4-u3qk5+i3qj5+k3qu5-i4qj2+j4qi2-u5qk3+k5qu3)*q.i1
			+(-jqu6-j1qu7+u2qj4-i2qk4+k2qi4+u3qj5+i3qk5-k3qi5-i4qk2+k4qi2+i5qk3-k5qi3)*q.u1
			+(-iqu6-j1qk7+u2qi4+j2qk4-k2qj4+u3qi5-i3qu5+k3qj5+j4qk2-k4qj2+u5qi3-i5qu3)*q.k1
			+(iqi6+jqj6+kqk6+u1qu7+i1qi7+k1qk7-i2qi4-j2qj4-k2qk4-u3qu5-i3qi5-k3qk5)*q.j1
			+(-i5qj-u5qk-iqj5+j1qi4+u2qk7-j2qi7-k2qu7+u3qk6+i3qj6-k3qu6)*q.i2
			+(iqk5+jqu5-kqi5-u1qj4+i1qk4-k1qi4-i2qk7-j2qu7+k2qi7+u3qj6-i3qk6+k3qi6)*q.u2
			+(u5qi-k5qj-kqj5+j1qk4-u2qi7+i2qu7-j2qk7-u3qi6+i3qu6+k3qj6)*q.k2
			+(i5qi+k5qk-jqj5+j1qj4+u2qu7+i2qi7+k2qk7-u3qu6-i3qi6-k3qk6)*q.j2
			+(-j4qi+i4qj-i1qj5+j1qi5+u2qk6-i2qj6+j2qi6-k2qu6-u3qk7+k3qu7)*q.i3
			+(-k4qi+i4qk-u1qj5+j1qu5-u2qj6-i2qk6+j2qu6+k2qi6+i3qk7-k3qi7)*q.u3
			+(k4qj-j4qk+j1qk5-k1qj5-u2qi6+i2qu6+j2qk6-k2qj6+u3qi7-i3qu7)*q.k3
			+(-i3qj-u3qk-j4qi7-k4qu7+u5qk6+i5qj6)*q.i4
			+(u3qi-k3qj+i4qu7-j4qk7-u5qi6+k5qj6)*q.k4
			+(i3qi+k3qk+i4qi7+k4qk7-i5qi6-k5qk6)*q.j4
			+(-j2qi+i2qj+u2qk-i4qj6+j4qi6-u5qk7+k5qu7)*q.i5
			+(-k2qi-u2qj-k5qi7+i2qk-i4qk6+k4qi6+i5qk7)*q.u5
			+(-u2qi+j4qk6-i5qu7+k2qj-k4qj6-j2qk+u5qi7)*q.k5
			+(i2qi+k2qk+j2qj)*q.j5-j1qi*q.i6
			+(-i1qk+k1qi+u1qj)*q.u6
			-(q.k6*q.k+q.j*q.j6)*j1
		),
		// K7
		(sum + 2*k7k7)*k7+2*(
			(j7qj7+jqj+i7qi7+u7qu7+kqk+iqi+i1qi1+u1qu1+j1qj1+k2qk2+j2qj2+i2qi2+u2qu2+k1qk1+j4qj4+i4qi4+u4qu4+k3qk3+j3qj3+i3qi3+u3qu3+k5qk5+j5qj5+i5qi5+u5qu5+k4qk4+i6qi6+u6qu6+k6qk6+j6qj6)*q.k7
			-(j4qi3-k5qu2+u6qk1+j6qi1-i7qj+j2qi5-u3qk4+k2qu5+j5qi2+k4qu3+kqu7+j7qi-u1qk6+j1qi6-u5qk2-i5qj2+j3qi4-i4qj3-iqj7+jqi7-i6qj1-u7qk-k1qu6+u2qk5-i3qj4+k3*q.u4+k6qu1-i2qj5-u4*q.k3-i1qj6)*q.r
			+(-kqu6-k1qu7+u2qk4+i2qj4-j2qi4+u3qk5-i3qj5+j3qi5+i4qj2-j4qi2-i5qj3+j5qi3)*q.u1
			+(-jqu6-k1qi7+u2qj4-i2qk4+k2qi4+u3qj5+i3qk5-j3qu5-i4qk2+k4qi2+u5qj3-j5qu3)*q.i1
			+(iqu6-k1qj7-u2qi4-j2qk4+k2qj4-u3qi5+i3qu5+j3qk5-j4qk2+k4qj2-u5qi3+i5qu3)*q.j1
			+(iqi6+jqj6+kqk6+u1qu7+i1qi7+j1qj7-i2qi4-j2qj4-k2qk4-u3qu5-i3qi5-j3qj5)*q.k1
			+(-iqj5+jqi5+kqu5-u1qk4-i1qj4+j1qi4+i2qj7-j2qi7-k2qu7+u3qk6+i3qj6-j3qi6)*q.u2
			+(u5qj-i5qk-iqk5+k1qi4-u2qj7+j2qu7-k2qi7-u3qj6+i3qk6+j3qu6)*q.i2
			+(-u5qi-j5qk-jqk5+k1qj4+u2qi7-i2qu7-k2qj7+u3qi6-i3qu6+j3qk6)*q.j2
			+(i5qi+j5qj-kqk5+k1qk4+u2qu7+i2qi7+j2qj7-u3qu6-i3qi6-j3qj6)*q.k2
			+(j4qi-i4qj-u1qk5+k1qu5-u2qk6+i2qj6-j2qi6+k2qu6-i3qj7+j3qi7)*q.u3
			+(-k4qi+i4qk-i1qk5+k1qi5-u2qj6-i2qk6+j2qu6+k2qi6+u3qj7-j3qu7)*q.i3
			+(-k4qj+j4qk-j1qk5+k1qj5+u2qi6-i2qu6-j2qk6+k2qj6-u3qi7+i3qu7)*q.j3
			+(u3qj-i3qk+j4qu7-k4qi7-u5qj6+i5qk6)*q.i4
			+(-u3qi-j3qk-i4qu7-k4qj7+u5qi6+j5qk6)*q.j4
			+(i3qi+j3qj+i4qi7+j4qj7-i5qi6-j5qj6)*q.k4
			+(j2qi-i2qj-u2qk+i4qj6-j4qi6-i5qj7+j5qi7)*q.u5
			+(-k2qi-u2qj-i4qk6+i2qk-j5qu7+k4qi6+u5qj7)*q.i5
			+(u2qi-j4qk6+i5qu7-u5qi7-k2qj+k4qj6+j2qk)*q.j5
			+(i2qi+k2qk+j2qj)*q.k5
			+(-j1qi+i1qj+u1qk)*q.u6
			-k1*(q.i6*q.i+q.j*q.j6+q.k6*q.k)
		)
	);
}

/******************************************
 * Boolean-valued Functions
 ******************************************/


template <typename T> bool Trigintaduonion<T>::is_imaginary() const {
	return ( r == 0.0 );
}

template <typename T> bool Trigintaduonion<T>::is_inf() const {
  	return
		( r == Support<T>::POS_INF ) ||
		( r == Support<T>::NEG_INF ) ||
		( i == Support<T>::POS_INF ) ||
		( i == Support<T>::NEG_INF ) ||
		( j == Support<T>::POS_INF ) ||
		( j == Support<T>::NEG_INF ) ||
		( k == Support<T>::POS_INF ) ||
		( k == Support<T>::NEG_INF ) ||
		( u1 == Support<T>::POS_INF ) ||
		( u1 == Support<T>::NEG_INF ) ||
		( i1 == Support<T>::POS_INF ) ||
		( i1 == Support<T>::NEG_INF ) ||
		( j1 == Support<T>::POS_INF ) ||
		( j1 == Support<T>::NEG_INF ) ||
		( k1 == Support<T>::POS_INF ) ||
		( k1 == Support<T>::NEG_INF ) ||
		( u2 == Support<T>::POS_INF ) ||
		( u2 == Support<T>::NEG_INF ) ||
		( i2 == Support<T>::POS_INF ) ||
		( i2 == Support<T>::NEG_INF ) ||
		( j2 == Support<T>::POS_INF ) ||
		( j2 == Support<T>::NEG_INF ) ||
		( k2 == Support<T>::POS_INF ) ||
		( k2 == Support<T>::NEG_INF ) ||
		( u3 == Support<T>::POS_INF ) ||
		( u3 == Support<T>::NEG_INF ) ||
		( i3 == Support<T>::POS_INF ) ||
		( i3 == Support<T>::NEG_INF ) ||
		( j3 == Support<T>::POS_INF ) ||
		( j3 == Support<T>::NEG_INF ) ||
		( k3 == Support<T>::POS_INF ) ||
		( k3 == Support<T>::NEG_INF ) ||
		( u4 == Support<T>::POS_INF ) ||
		( u4 == Support<T>::NEG_INF ) ||
		( i4 == Support<T>::POS_INF ) ||
		( i4 == Support<T>::NEG_INF ) ||
		( j4 == Support<T>::POS_INF ) ||
		( j4 == Support<T>::NEG_INF ) ||
		( k4 == Support<T>::POS_INF ) ||
		( k4 == Support<T>::NEG_INF ) ||
		( u5 == Support<T>::POS_INF ) ||
		( u5 == Support<T>::NEG_INF ) ||
		( i5 == Support<T>::POS_INF ) ||
		( i5 == Support<T>::NEG_INF ) ||
		( j5 == Support<T>::POS_INF ) ||
		( j5 == Support<T>::NEG_INF ) ||
		( k5 == Support<T>::POS_INF ) ||
		( k5 == Support<T>::NEG_INF ) ||
		( u6 == Support<T>::POS_INF ) ||
		( u6 == Support<T>::NEG_INF ) ||
		( i6 == Support<T>::POS_INF ) ||
		( i6 == Support<T>::NEG_INF ) ||
		( j6 == Support<T>::POS_INF ) ||
		( j6 == Support<T>::NEG_INF ) ||
		( k6 == Support<T>::POS_INF ) ||
		( k6 == Support<T>::NEG_INF ) ||
		( u7 == Support<T>::POS_INF ) ||
		( u7 == Support<T>::NEG_INF ) ||
		( i7 == Support<T>::POS_INF ) ||
		( i7 == Support<T>::NEG_INF ) ||
		( j7 == Support<T>::POS_INF ) ||
		( j7 == Support<T>::NEG_INF ) ||
		( k7 == Support<T>::POS_INF ) ||
		( k7 == Support<T>::NEG_INF );
}

template <typename T> bool Trigintaduonion<T>::is_nan() const {
  	return ( r != r ) || ( i != i ) || ( j != j ) || ( k != k ) || ( u1 != u1 ) || ( i1 != i1 ) || ( j1 != j1 ) || ( k1 != k1 ) || ( u2 != u2 ) || ( i2 != i2 ) || ( j2 != j2 ) || ( k2 != k2 ) || ( u3 != u3 ) || ( i3 != i3 ) || ( j3 != j3 ) || ( k3 != k3 ) || ( u4 != u4 ) || ( i4 != i4 ) || ( j4 != j4 ) || ( k4 != k4 ) || ( u5 != u5 ) || ( i5 != i5 ) || ( j5 != j5 ) || ( k5 != k5 ) || ( u6 != u6 ) || ( i6 != i6 ) || ( j6 != j6 ) || ( k6 != k6 ) || ( u7 != u7 ) || ( i7 != i7 ) || ( j7 != j7 ) || ( k7 != k7 );
}

template <typename T> bool Trigintaduonion<T>::is_neg_inf() const {
  	return ( r == Support<T>::NEG_INF ) && ( i == 0.0 ) && ( j == 0.0 ) && ( k == 0.0 ) && ( u1 == 0.0 ) && ( i1 == 0.0 ) && ( j1 == 0.0 ) && ( k1 == 0.0 ) && ( u2 == 0.0 ) && ( i2 == 0.0 ) && ( j2 == 0.0 ) && ( k2 == 0.0 ) && ( u3 == 0.0 ) && ( i3 == 0.0 ) && ( j3 == 0.0 ) && ( k3 == 0.0 ) && ( u4 == 0.0 ) && ( i4 == 0.0 ) && ( j4 == 0.0 ) && ( k4 == 0.0 ) && ( u5 == 0.0 ) && ( i5 == 0.0 ) && ( j5 == 0.0 ) && ( k5 == 0.0 ) && ( u6 == 0.0 ) && ( i6 == 0.0 ) && ( j6 == 0.0 ) && ( k6 == 0.0 ) && ( u7 == 0.0 ) && ( i7 == 0.0 ) && ( j7 == 0.0 ) && ( k7 == 0.0 );
}

template <typename T> bool Trigintaduonion<T>::is_pos_inf() const {
  	return ( r == Support<T>::POS_INF ) && ( i == 0.0 ) && ( j == 0.0 ) && ( k == 0.0 ) && ( u1 == 0.0 ) && ( i1 == 0.0 ) && ( j1 == 0.0 ) && ( k1 == 0.0 ) && ( u2 == 0.0 ) && ( i2 == 0.0 ) && ( j2 == 0.0 ) && ( k2 == 0.0 ) && ( u3 == 0.0 ) && ( i3 == 0.0 ) && ( j3 == 0.0 ) && ( k3 == 0.0 ) && ( u4 == 0.0 ) && ( i4 == 0.0 ) && ( j4 == 0.0 ) && ( k4 == 0.0 ) && ( u5 == 0.0 ) && ( i5 == 0.0 ) && ( j5 == 0.0 ) && ( k5 == 0.0 ) && ( u6 == 0.0 ) && ( i6 == 0.0 ) && ( j6 == 0.0 ) && ( k6 == 0.0 ) && ( u7 == 0.0 ) && ( i7 == 0.0 ) && ( j7 == 0.0 ) && ( k7 == 0.0 );
}

template <typename T> bool Trigintaduonion<T>::is_real() const {
	return ( i == 0.0 ) && ( j == 0.0 ) && ( k == 0.0 ) && ( u1 == 0.0 ) && ( i1 == 0.0 ) && ( j1 == 0.0 ) && ( k1 == 0.0 ) && ( u2 == 0.0 ) && ( i2 == 0.0 ) && ( j2 == 0.0 ) && ( k2 == 0.0 ) && ( u3 == 0.0 ) && ( i3 == 0.0 ) && ( j3 == 0.0 ) && ( k3 == 0.0 ) && ( u4 == 0.0 ) && ( i4 == 0.0 ) && ( j4 == 0.0 ) && ( k4 == 0.0 ) && ( u5 == 0.0 ) && ( i5 == 0.0 ) && ( j5 == 0.0 ) && ( k5 == 0.0 ) && ( u6 == 0.0 ) && ( i6 == 0.0 ) && ( j6 == 0.0 ) && ( k6 == 0.0 ) && ( u7 == 0.0 ) && ( i7 == 0.0 ) && ( j7 == 0.0 ) && ( k7 == 0.0 );
}

template <typename T> bool Trigintaduonion<T>::is_real_inf() const {
  	return ( r == Support<T>::POS_INF || r == Support<T>::NEG_INF ) && ( i == 0.0 ) && ( j == 0.0 ) && ( k == 0.0 ) && ( u1 == 0.0 ) && ( i1 == 0.0 ) && ( j1 == 0.0 ) && ( k1 == 0.0 ) && ( u2 == 0.0 ) && ( i2 == 0.0 ) && ( j2 == 0.0 ) && ( k2 == 0.0 ) && ( u3 == 0.0 ) && ( i3 == 0.0 ) && ( j3 == 0.0 ) && ( k3 == 0.0 ) && ( u4 == 0.0 ) && ( i4 == 0.0 ) && ( j4 == 0.0 ) && ( k4 == 0.0 ) && ( u5 == 0.0 ) && ( i5 == 0.0 ) && ( j5 == 0.0 ) && ( k5 == 0.0 ) && ( u6 == 0.0 ) && ( i6 == 0.0 ) && ( j6 == 0.0 ) && ( k6 == 0.0 ) && ( u7 == 0.0 ) && ( i7 == 0.0 ) && ( j7 == 0.0 ) && ( k7 == 0.0 );
}

template <typename T> bool Trigintaduonion<T>::is_zero() const {
	return ( r == 0.0 ) && ( i == 0.0 ) && ( j == 0.0 ) && ( k == 0.0 ) && ( u1 == 0.0 ) && ( i1 == 0.0 ) && ( j1 == 0.0 ) && ( k1 == 0.0 ) && ( u2 == 0.0 ) && ( i2 == 0.0 ) && ( j2 == 0.0 ) && ( k2 == 0.0 ) && ( u3 == 0.0 ) && ( i3 == 0.0 ) && ( j3 == 0.0 ) && ( k3 == 0.0 ) && ( u4 == 0.0 ) && ( i4 == 0.0 ) && ( j4 == 0.0 ) && ( k4 == 0.0 ) && ( u5 == 0.0 ) && ( i5 == 0.0 ) && ( j5 == 0.0 ) && ( k5 == 0.0 ) && ( u6 == 0.0 ) && ( i6 == 0.0 ) && ( j6 == 0.0 ) && ( k6 == 0.0 ) && ( u7 == 0.0 ) && ( i7 == 0.0 ) && ( j7 == 0.0 ) && ( k7 == 0.0 );
}

/******************************************
 * Multiplier Function
 ******************************************/

template <typename T> inline Trigintaduonion<T> Trigintaduonion<T>::multiplier( T r, T mltplr, const Trigintaduonion<T> & q ) {
	if ( Support<T>::is_nan( mltplr ) || Support<T>::is_inf( mltplr ) ) {
		if ( q.i == 0 && q.j == 0 && q.k == 0 ) {
			return Trigintaduonion<T>(
				r,           mltplr*q.i,  mltplr*q.j,  mltplr*q.k,
				mltplr*q.u1, mltplr*q.i1, mltplr*q.j1, mltplr*q.k1,
				mltplr*q.u2, mltplr*q.i2, mltplr*q.j2, mltplr*q.k2,
				mltplr*q.u3, mltplr*q.i3, mltplr*q.j3, mltplr*q.k3,
				mltplr*q.u4, mltplr*q.i4, mltplr*q.j4, mltplr*q.k4,
				mltplr*q.u5, mltplr*q.i5, mltplr*q.j5, mltplr*q.k5,
				mltplr*q.u6, mltplr*q.i6, mltplr*q.j6, mltplr*q.k6,
				mltplr*q.u7, mltplr*q.i7, mltplr*q.j7, mltplr*q.k7
			);
		} else {
			return Trigintaduonion<T>(
				r,
				(q.i == 0) ? Support<T>::sign( mltplr )*q.i : mltplr*q.i,
				(q.j == 0) ? Support<T>::sign( mltplr )*q.j : mltplr*q.j,
				(q.k == 0) ? Support<T>::sign( mltplr )*q.k : mltplr*q.k,
				(q.u1 == 0) ? Support<T>::sign( mltplr )*q.u1 : mltplr*q.u1,
				(q.i1 == 0) ? Support<T>::sign( mltplr )*q.i1 : mltplr*q.i1,
				(q.j1 == 0) ? Support<T>::sign( mltplr )*q.j1 : mltplr*q.j1,
				(q.k1 == 0) ? Support<T>::sign( mltplr )*q.k1 : mltplr*q.k1,
				(q.u2 == 0) ? Support<T>::sign( mltplr )*q.u2 : mltplr*q.u2,
				(q.i2 == 0) ? Support<T>::sign( mltplr )*q.i2 : mltplr*q.i2,
				(q.j2 == 0) ? Support<T>::sign( mltplr )*q.j2 : mltplr*q.j2,
				(q.k2 == 0) ? Support<T>::sign( mltplr )*q.k2 : mltplr*q.k2,
				(q.u3 == 0) ? Support<T>::sign( mltplr )*q.u3 : mltplr*q.u3,
				(q.i3 == 0) ? Support<T>::sign( mltplr )*q.i3 : mltplr*q.i3,
				(q.j3 == 0) ? Support<T>::sign( mltplr )*q.j3 : mltplr*q.j3,
				(q.k3 == 0) ? Support<T>::sign( mltplr )*q.k3 : mltplr*q.k3,
				(q.u4 == 0) ? Support<T>::sign( mltplr )*q.u4 : mltplr*q.u4,
				(q.i4 == 0) ? Support<T>::sign( mltplr )*q.i4 : mltplr*q.i4,
				(q.j4 == 0) ? Support<T>::sign( mltplr )*q.j4 : mltplr*q.j4,
				(q.k4 == 0) ? Support<T>::sign( mltplr )*q.k4 : mltplr*q.k4,
				(q.u5 == 0) ? Support<T>::sign( mltplr )*q.u5 : mltplr*q.u5,
				(q.i5 == 0) ? Support<T>::sign( mltplr )*q.i5 : mltplr*q.i5,
				(q.j5 == 0) ? Support<T>::sign( mltplr )*q.j5 : mltplr*q.j5,
				(q.k5 == 0) ? Support<T>::sign( mltplr )*q.k5 : mltplr*q.k5,
				(q.u6 == 0) ? Support<T>::sign( mltplr )*q.u6 : mltplr*q.u6,
				(q.i6 == 0) ? Support<T>::sign( mltplr )*q.i6 : mltplr*q.i6,
				(q.j6 == 0) ? Support<T>::sign( mltplr )*q.j6 : mltplr*q.j6,
				(q.k6 == 0) ? Support<T>::sign( mltplr )*q.k6 : mltplr*q.k6,
				(q.u7 == 0) ? Support<T>::sign( mltplr )*q.u7 : mltplr*q.u7,
				(q.i7 == 0) ? Support<T>::sign( mltplr )*q.i7 : mltplr*q.i7,
				(q.j7 == 0) ? Support<T>::sign( mltplr )*q.j7 : mltplr*q.j7,
				(q.k7 == 0) ? Support<T>::sign( mltplr )*q.k7 : mltplr*q.k7
			);
		}
	} else {
		return Trigintaduonion<T>(
			r,           mltplr*q.i,  mltplr*q.j,  mltplr*q.k,
			mltplr*q.u1, mltplr*q.i1, mltplr*q.j1, mltplr*q.k1,
			mltplr*q.u2, mltplr*q.i2, mltplr*q.j2, mltplr*q.k2,
			mltplr*q.u3, mltplr*q.i3, mltplr*q.j3, mltplr*q.k3,
			mltplr*q.u4, mltplr*q.i4, mltplr*q.j4, mltplr*q.k4,
			mltplr*q.u5, mltplr*q.i5, mltplr*q.j5, mltplr*q.k5,
			mltplr*q.u6, mltplr*q.i6, mltplr*q.j6, mltplr*q.k6,
			mltplr*q.u7, mltplr*q.i7, mltplr*q.j7, mltplr*q.k7
		);
	}
}

template <typename T> inline Trigintaduonion<T> Trigintaduonion<T>::make_inf( T r, T i ) {
	return Trigintaduonion<T>(
		r, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i
	);
}

template <typename T> inline Trigintaduonion<T> Trigintaduonion<T>::make_i( T r, T i ) {
	return Trigintaduonion<T>(
		r, i, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	);
}

/******************************************
 * Exponential and Logarithmic Functions
 ******************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::exp() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).exp();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::log() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).log();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::log10() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).log10();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::pow( const Trigintaduonion<T> & q ) const {
	return ( log() * q ).exp();
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::pow(T x) const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).pow( x );
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::inverse() const {
	if ( is_zero() ) {
		return Trigintaduonion(
			 Support<T>::sign(  r )*Support<T>::POS_INF,
			-Support<T>::sign(  i )*Support<T>::POS_INF,
			-Support<T>::sign(  j )*Support<T>::POS_INF,
			-Support<T>::sign(  k )*Support<T>::POS_INF,
			-Support<T>::sign( u1 )*Support<T>::POS_INF,
			-Support<T>::sign( i1 )*Support<T>::POS_INF,
			-Support<T>::sign( j1 )*Support<T>::POS_INF,
			-Support<T>::sign( k1 )*Support<T>::POS_INF,
			-Support<T>::sign( u2 )*Support<T>::POS_INF,
			-Support<T>::sign( i2 )*Support<T>::POS_INF,
			-Support<T>::sign( j2 )*Support<T>::POS_INF,
			-Support<T>::sign( k2 )*Support<T>::POS_INF,
			-Support<T>::sign( u3 )*Support<T>::POS_INF,
			-Support<T>::sign( i3 )*Support<T>::POS_INF,
			-Support<T>::sign( j3 )*Support<T>::POS_INF,
			-Support<T>::sign( k3 )*Support<T>::POS_INF,
			-Support<T>::sign( u4 )*Support<T>::POS_INF,
			-Support<T>::sign( i4 )*Support<T>::POS_INF,
			-Support<T>::sign( j4 )*Support<T>::POS_INF,
			-Support<T>::sign( k4 )*Support<T>::POS_INF,
			-Support<T>::sign( u5 )*Support<T>::POS_INF,
			-Support<T>::sign( i5 )*Support<T>::POS_INF,
			-Support<T>::sign( j5 )*Support<T>::POS_INF,
			-Support<T>::sign( k5 )*Support<T>::POS_INF,
			-Support<T>::sign( u6 )*Support<T>::POS_INF,
			-Support<T>::sign( i6 )*Support<T>::POS_INF,
			-Support<T>::sign( j6 )*Support<T>::POS_INF,
			-Support<T>::sign( k6 )*Support<T>::POS_INF,
			-Support<T>::sign( u7 )*Support<T>::POS_INF,
			-Support<T>::sign( i7 )*Support<T>::POS_INF,
			-Support<T>::sign( j7 )*Support<T>::POS_INF,
			-Support<T>::sign( k7 )*Support<T>::POS_INF
		);
	} else if ( is_inf() ) {
		return Trigintaduonion(
			 Support<T>::sign(  r )*0.0,
			-Support<T>::sign(  i )*0.0,
			-Support<T>::sign(  j )*0.0,
			-Support<T>::sign(  k )*0.0,
			-Support<T>::sign( u1 )*0.0,
			-Support<T>::sign( i1 )*0.0,
			-Support<T>::sign( j1 )*0.0,
			-Support<T>::sign( k1 )*0.0,
			-Support<T>::sign( u2 )*0.0,
			-Support<T>::sign( i2 )*0.0,
			-Support<T>::sign( j2 )*0.0,
			-Support<T>::sign( k2 )*0.0,
			-Support<T>::sign( u3 )*0.0,
			-Support<T>::sign( i3 )*0.0,
			-Support<T>::sign( j3 )*0.0,
			-Support<T>::sign( k3 )*0.0,
			-Support<T>::sign( u4 )*0.0,
			-Support<T>::sign( i4 )*0.0,
			-Support<T>::sign( j4 )*0.0,
			-Support<T>::sign( k4 )*0.0,
			-Support<T>::sign( u5 )*0.0,
			-Support<T>::sign( i5 )*0.0,
			-Support<T>::sign( j5 )*0.0,
			-Support<T>::sign( k5 )*0.0,
			-Support<T>::sign( u6 )*0.0,
			-Support<T>::sign( i6 )*0.0,
			-Support<T>::sign( j6 )*0.0,
			-Support<T>::sign( k6 )*0.0,
			-Support<T>::sign( u7 )*0.0,
			-Support<T>::sign( i7 )*0.0,
			-Support<T>::sign( j7 )*0.0,
			-Support<T>::sign( k7 )*0.0
		);
	} else if ( is_nan() ) {
		return Trigintaduonion(
			Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
			Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
			Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
			Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
			Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
			Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
			Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN,
			Support<T>::NaN, Support<T>::NaN, Support<T>::NaN, Support<T>::NaN
		);
	} else {
		T denom = norm();

		return Trigintaduonion(
			  r/denom,  -i/denom,  -j/denom,  -k/denom,
			-u1/denom, -i1/denom, -j1/denom, -k1/denom,
			-u2/denom, -i2/denom, -j2/denom, -k2/denom,
			-u3/denom, -i3/denom, -j3/denom, -k3/denom,
			-u4/denom, -i4/denom, -j4/denom, -k4/denom,
			-u5/denom, -i5/denom, -j5/denom, -k5/denom,
			-u6/denom, -i6/denom, -j6/denom, -k6/denom,
			-u7/denom, -i7/denom, -j7/denom, -k7/denom
		);
	}
}

/********************************************************************
 * Trigonometric and Hyperbolic Functions
 *
 * For each function f:S -> S, we define:
 *
 *             ~                       Im(q)     ~
 *   f(q) = Re(f(Re(q) + i|Im(q)|)) + ------- Im(f(Re(q) + i|Im(q)|))
 *                                    |Im(q)|
 *       ~
 * where f:C -> C is the complex equivalent of the
 * function f.
 ********************************************************************/

/**********************************************************
 * Sine Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::sin() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).sin();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Complementary Sine Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::cos() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).cos();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Tangent Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::tan() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).tan();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Secant Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::sec() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).sec();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Complementary Secant Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::csc() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).csc();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Complementary Tangent Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::cot() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).cot();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Hyperbolic Sine Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::sinh() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).sinh();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Hyperbolic Complementary Sine Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::cosh() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).cosh();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Hyperbolic Tangent Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::tanh() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).tanh();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Hyperbolic Secant Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::sech() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).sech();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Hyperbolic Complementary Secant Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::csch() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).csch();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Hyperbolic Complementary Tangent Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::coth() const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).coth();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

// Real Branch Cut:    (-oo, -1) U (1, oo)

/**********************************************************
 * Inverse Sine Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::asin( const Trigintaduonion<T> & q ) const {
	T absIm = abs_imag();

	// Branch Cuts:   (-oo, -1) U (1, oo)

	if ( absIm == 0 ) {
		if ( r > 1 ) {
			// Branch cut (1, oo)

			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( Support<T>::PI2, std::log( r + std::sqrt( r*r - 1 ) ) );
			} else {
				return multiplier( Support<T>::PI2, std::log( r + std::sqrt( r*r - 1 ) )/absq, q );
			}
		} else if ( r < -1 ) {
			// Branch cut (-oo, -1)

			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( -Support<T>::PI2, std::log( -r + std::sqrt( r*r - 1 ) ) );
			} else {
				return multiplier( -Support<T>::PI2, std::log( -r + std::sqrt( r*r - 1 ) )/absq, q );
			}
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).asin();

	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );

}

// Real Branch Cut:    (-oo, -1) U (1, oo)

/**********************************************************
 * Inverse Complementary Sine Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::acos( const Trigintaduonion<T> & q ) const {
	T absIm = abs_imag();

	// Branch Cuts:   (-oo, -1) U (1, oo)

	if ( absIm == 0 ) {
		if ( r > 1 ) {
			// Branch cut (1, oo)

			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.0, -std::log( r + std::sqrt( r*r - 1 ) ) );
			} else {
				return multiplier( 0.0, -std::log( r + std::sqrt( r*r - 1 ) )/absq, q );
			}
		} else if ( r < -1 ) {
			// Branch cut (-oo, -1)

			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( Support<T>::PI, -std::log( -r + std::sqrt( r*r - 1 ) ) );
			} else {
				return multiplier( Support<T>::PI, -std::log( -r + std::sqrt( r*r - 1 ) )/absq, q );
			}
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).acos();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

// Complex Branch Cut:    (-ooi, -i] U [i, ooi)

/**********************************************************
 * Inverse Tangent Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::atan() const {
	T absIm = abs_imag();

	if ( r == 0 ) {
		if ( absIm == 1 ) {
				return multiplier( Support<T>::NaN, Support<T>::POS_INF, *this );
		} else if ( absIm > 1 ) {
			// Branch cut [ui, Inf)
			//  - ui is a unit purely-imaginary quaternion

			T p = absIm + 1;
			T m = absIm - 1;

			T mltplr = 0.25*std::log( (p*p)/(m*m) )/absIm;

			return multiplier( Support<T>::sign( r )*Support<T>::PI2, mltplr, *this );
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).atan();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Inverse Secant Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::asec( const Trigintaduonion<T> & q ) const {
	T absIm = abs_imag();

	// Branch Cut: (-1, 1)

	if ( absIm == 0 ) {
		if ( r == 0.0 ) {
			return multiplier( Support<T>::POS_INF, Support<T>::POS_INF, q );
		} else if ( r > 0.0 && r < 1.0 ) {
			T absq = q.abs_imag();
			
			if ( q == I || absq == 0 ) {
				return make_i( 0.0, std::log( 1.0/r + std::sqrt( 1.0/(r*r) - 1.0 ) ) );
			} else {
				return multiplier( 0.0, std::log( 1.0/r + std::sqrt( 1.0/(r*r) - 1.0 ) )/absq, q );
			}
		} else if ( r > -1.0 && r < 0.0 ) {
			T absq = q.abs_imag();
			
			if ( q == I || absq == 0 ) {
				return make_i( Support<T>::PI, std::log( -1.0/r + std::sqrt( 1.0/(r*r) - 1.0 ) ) );
			} else {
				return multiplier( Support<T>::PI, std::log( -1.0/r + std::sqrt( 1.0/(r*r) - 1.0 ) )/absq, q );
			}
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).asec();

	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

// Real Branch Cut:    (-1, 1)
/**********************************************************
 * Inverse Complementary Secant Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::acsc( const Trigintaduonion<T> & q ) const {
	T absIm = abs_imag();

	if ( absIm == 0.0 ) {
		if ( r == 0.0 ) {
			return multiplier( Support<T>::POS_INF, Support<T>::POS_INF, q );
		} else if ( r > 0.0 && r <  1.0 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( Support<T>::PI2, -std::log( 1.0/r + std::sqrt( 1.0/(r*r) - 1.0 ) ) );
			} else {
				return multiplier( Support<T>::PI2, -std::log( 1.0/r + std::sqrt( 1.0/(r*r) - 1.0 ) )/absq, q );
			}
		} else if ( r > -1.0 && r < 0.0 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( -Support<T>::PI2, -std::log( -1.0/r + std::sqrt( 1.0/(r*r) - 1.0 ) ) );
			} else {
				return multiplier( -Support<T>::PI2, -std::log( -1.0/r + std::sqrt( 1.0/(r*r) - 1.0 ) )/absq, q );
			}
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).acsc();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}


/**********************************************************
 * Inverse Complementary Tangent Function
 * Complex Branch Cut:    (-i, i)
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::acot() const {
	T absIm = abs_imag();

	if ( r == 0 ) {
		if ( absIm == 0 ) {
			return multiplier( Support<T>::PI2, -1, *this );
		} else if ( absIm < 1 ) {
			// Branch cut [ui, Inf)
			//  - ui is a unit purely-imaginary quaternion

			T p = absIm + 1;
			T m = absIm - 1;

			T mltplr = -0.25*std::log( (p*p)/(m*m) )/absIm;

			return multiplier( Support<T>::PI2, mltplr, *this );
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).acot();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

// Complex Branch Cut:    (-ooi, -i) U (i, ooi)

/**********************************************************
 * Inverse Hyperbolic Sine Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::asinh() const {
	T absIm = abs_imag();

	if ( r == 0 ) {
		if ( absIm > 1 ) {
			return multiplier( Support<T>::sign( r )*std::log( absIm + std::sqrt(absIm*absIm - 1) ), Support<T>::PI2/absIm, *this );
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).asinh();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}


/**********************************************************
 * Inverse Hyperbolic Complementary Sine Function
 * Real Branch Cut:    (-oo, 1)
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::acosh( const Trigintaduonion<T> & q ) const {
	T absIm = abs_imag();

	if ( absIm == 0 ) {
		if ( r < -1 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( std::log(-r + std::sqrt(r*r - 1)), Support<T>::PI*Support<T>::sign(i) );
			} else {
				return multiplier( std::log(-r + std::sqrt(r*r - 1)), Support<T>::PI/absq, q );
			}
		} else if ( r == -1 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.0, Support<T>::PI*Support<T>::sign(i) );
			} else {
				return multiplier( 0.0, Support<T>::PI/absq, q );
			}
		} else if ( r < 0 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.0, std::acos(r)*Support<T>::sign(i) );
			} else {
				return multiplier( 0.0, std::acos(r)/absq, q );
			}
		} else if ( r == 0 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.0, Support<T>::PI2*Support<T>::sign(i) );
			} else {
				return multiplier( 0.0, Support<T>::PI2/absq, q );
			}
		} else if ( r < 1 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.0, std::acos( r )*Support<T>::sign(i) );
			} else {
				return multiplier( 0.0, std::acos(r)/absq, q );
			}
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).acosh();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}


/**********************************************************
 * Inverse Hyperbolic Tangent Function
 * Real Branch Cut:    (-oo, -1] U [1, oo)
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::atanh( const Trigintaduonion<T> & q ) const {
	T absIm = abs_imag();

	if ( absIm == 0 ) {
		if ( r == -1 ) {
			return make_inf( Support<T>::NEG_INF, Support<T>::NaN );
		} else if ( r == 1 ) {
			return make_inf( Support<T>::POS_INF, Support<T>::NaN );
		} else if ( r < -1 || r > 1 ) {
			T p = r + 1;
			T m = r - 1;

			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.25*std::log( (p*p)/(m*m) ), -Support<T>::sign(r)*Support<T>::PI2 );
			} else {
				return multiplier( 0.25*std::log( (p*p)/(m*m) ), -Support<T>::sign(r)*Support<T>::PI2/absq, q );
			}
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).atanh();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}


/**********************************************************
 * Inverse Hyperbolic Secant Function
 * Real Branch Cut:    (-oo, 0] U (1, oo)
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::asech( const Trigintaduonion<T> & q ) const {
	T absIm = abs_imag();

	if ( absIm == 0 ) {
		if ( r < -1 || r > 1 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.0, -std::acos( 1/r )*Support<T>::sign(i) );
			} else {
				return multiplier( 0.0, -std::acos( 1/r )/absq, q );
			}
		} else if ( r == -1 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.0, Support<T>::PI*Support<T>::sign(i) );
			} else {
				return multiplier( 0.0, Support<T>::PI/absq, q );
			}
		} else if ( r < 0 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( std::log( -1/r + std::sqrt( 1/(r*r) - 1 ) ), -Support<T>::PI );
			} else {
				return multiplier( std::log( -1/r + std::sqrt( 1/(r*r) - 1 ) ), -Support<T>::PI/absq, q );
			}
		} else if ( r == 0 ) {
			return make_inf( Support<T>::POS_INF, Support<T>::NaN );
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).asech();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}


/**********************************************************
 * Inverse Hyperbolic Complementary Secant Function
 * Complex Branch Cut:    (-i, i)
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::acsch() const {
	T absIm = abs_imag();

	if ( r == 0 ) {
			if ( absIm == 0 ) {
				return make_inf( Support<T>::NEG_INF, Support<T>::NaN );
			} else if ( absIm < 1 ) {
				return multiplier( Support<T>::sign( r )*std::log( 1/absIm + std::sqrt(1/(absIm*absIm) - 1) ), -Support<T>::PI2/absIm, *this );
			}
	}

	Complex<T> z = Complex<T>( r, absIm ).acsch();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}


/**********************************************************
 * Inverse Hyperbolic Complementary Tangent Function
 * Real Branch Cut:    [-1, 1]
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::acoth( const Trigintaduonion<T> & q ) const {
	T absIm = abs_imag();

	if ( absIm == 0 ) {
		if ( r == -1 ) {
			return make_inf( Support<T>::NEG_INF, Support<T>::NaN );
		} else if ( r == 1 ) {
			return make_inf( Support<T>::POS_INF, Support<T>::NaN );
		} else if ( r == 0 ) {
			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.0, -Support<T>::PI2*Support<T>::sign(i) );
			} else {
				return multiplier( 0.0, -Support<T>::PI2/absq, q );
			}
		} else if ( r > -1 && r < 0 ) {
			T p = r + 1;
			T m = r - 1;

			T absq = q.abs_imag();

			if ( q == I || absq == 0 ) {
				return make_i( 0.25*std::log( (p*p)/(m*m) ), -Support<T>::sign(r)*Support<T>::PI2 );
			} else {
				return multiplier( 0.25*std::log( (p*p)/(m*m) ), -Support<T>::sign(r)*Support<T>::PI2/absq, q );
			}
		}
	}

	Complex<T> z = Complex<T>( r, absIm ).acoth();
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/**********************************************************
 * Bessel J Function
 **********************************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::bessel_J( int n ) const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).bessel_J( n );
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/******************************************
 * Integer Functions
 ******************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::ceil() const {
	return Trigintaduonion<T>(
		std::ceil(r), std::ceil(i), std::ceil(j), std::ceil(k),
		std::ceil(u1), std::ceil(i1), std::ceil(j1), std::ceil(k1),
		std::ceil(u2), std::ceil(i2), std::ceil(j2), std::ceil(k2),
		std::ceil(u3), std::ceil(i3), std::ceil(j3), std::ceil(k3),
		std::ceil(u4), std::ceil(i4), std::ceil(j4), std::ceil(k4),
		std::ceil(u5), std::ceil(i5), std::ceil(j5), std::ceil(k5),
		std::ceil(u6), std::ceil(i6), std::ceil(j6), std::ceil(k6),
		std::ceil(u7), std::ceil(i7), std::ceil(j7), std::ceil(k7)
	);
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::floor() const {
	return Trigintaduonion<T>(
		std::floor(r), std::floor(i), std::floor(j), std::floor(k),
		std::floor(u1), std::floor(i1), std::floor(j1), std::floor(k1),
		std::floor(u2), std::floor(i2), std::floor(j2), std::floor(k2),
		std::floor(u3), std::floor(i3), std::floor(j3), std::floor(k3),
		std::floor(u4), std::floor(i4), std::floor(j4), std::floor(k4),
		std::floor(u5), std::floor(i5), std::floor(j5), std::floor(k5),
		std::floor(u6), std::floor(i6), std::floor(j6), std::floor(k6),
		std::floor(u7), std::floor(i7), std::floor(j7), std::floor(k7)
	);
}

/******************************************
 * Horner's Rule
 *
 *   The polynomial is defined by giving the highest
 *   coefficient first:
 *
 *            n - 1         n - 2
 *      v[0]*q      + v[1]*q      + ... + v[n-2]*q + v[n-1]
 *
 *   This is the same as with Matlab.  Because trigintaduonions are
 *   not commutative, this only makes sense if the coefficients
 *   and offsets are real.
 *
 *        Re(q) + i |Imag(q)|
 *
 ******************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::horner( T * v, unsigned int n ) const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).horner( v, n );
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::horner( T * v, T * c, unsigned int n ) const {
	T absIm = abs_imag();
	Complex<T> z = Complex<T>( r, absIm ).horner( v, c, n );
	
	T mltplr;

	if ( absIm == 0.0 ) {
		mltplr = z.imag_i();
	} else {
		mltplr = z.imag_i() / absIm;
	}

	return multiplier( z.real(), mltplr, *this );
}

/******************************************
 * Random Factories
 ******************************************/


template <typename T> Trigintaduonion<T> Trigintaduonion<T>::random() {
	return Trigintaduonion<T>(
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX
	);
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::random_imag() {
	return Trigintaduonion<T>(
		0.0,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX,
		(static_cast<T>( rand() ))/RAND_MAX
	);
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::random_real() {
	return Trigintaduonion<T>( (static_cast<T>( rand() ))/RAND_MAX, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
}

/******************************************
 * Binary Arithmetic Operators
 ******************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator + ( const Trigintaduonion<T> & z ) const {
	return Trigintaduonion<T>(
		 r + z.r,   i + z.i,   j + z.j,   k + z.k,
		u1 + z.u1, i1 + z.i1, j1 + z.j1, k1 + z.k1,
		u2 + z.u2, i2 + z.i2, j2 + z.j2, k2 + z.k2,
		u3 + z.u3, i3 + z.i3, j3 + z.j3, k3 + z.k3,
		u4 + z.u4, i4 + z.i4, j4 + z.j4, k4 + z.k4,
		u5 + z.u5, i5 + z.i5, j5 + z.j5, k5 + z.k5,
		u6 + z.u6, i6 + z.i6, j6 + z.j6, k6 + z.k6,
		u7 + z.u7, i7 + z.i7, j7 + z.j7, k7 + z.k7
	);
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator + ( T x ) const {
	return Trigintaduonion<T>( r + x, i, j, k, u1, i1, j1, k1, u2, i2, j2, k2, u3, i3, j3, k3, u4, i4, j4, k4, u5, i5, j5, k5, u6, i6, j6, k6, u7, i7, j7, k7 );
}

template <typename T> Trigintaduonion<T> operator + ( T x, const Trigintaduonion<T> & z ) {
	return Trigintaduonion<T>(
		x + z.real(), z.imag_i(), z.imag_j(), z.imag_k(),
		z.imag_u1(), z.imag_i1(), z.imag_j1(), z.imag_k1(),
		z.imag_u2(), z.imag_i2(), z.imag_j2(), z.imag_k2(),
		z.imag_u3(), z.imag_i3(), z.imag_j3(), z.imag_k3(),
		z.imag_u4(), z.imag_i4(), z.imag_j4(), z.imag_k4(),
		z.imag_u5(), z.imag_i5(), z.imag_j5(), z.imag_k5(),
		z.imag_u6(), z.imag_i6(), z.imag_j6(), z.imag_k6(),
		z.imag_u7(), z.imag_i7(), z.imag_j7(), z.imag_k7()
	);
}

template <typename T> Trigintaduonion<T> operator + ( long x, const Trigintaduonion<T> & z ) {
	return operator + ( static_cast<T>( x ), z );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator - ( const Trigintaduonion<T> & z ) const {
	return Trigintaduonion<T>(
		r - z.r, i - z.i, j - z.j, k - z.k,
		u1 - z.u1, i1 - z.i1, j1 - z.j1, k1 - z.k1,
		u2 - z.u2, i2 - z.i2, j2 - z.j2, k2 - z.k2,
		u3 - z.u3, i3 - z.i3, j3 - z.j3, k3 - z.k3,
		u4 - z.u4, i4 - z.i4, j4 - z.j4, k4 - z.k4,
		u5 - z.u5, i5 - z.i5, j5 - z.j5, k5 - z.k5,
		u6 - z.u6, i6 - z.i6, j6 - z.j6, k6 - z.k6,
		u7 - z.u7, i7 - z.i7, j7 - z.j7, k7 - z.k7
	);
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator - ( T x ) const {
	return Trigintaduonion<T>( r - x, i, j, k, u1, i1, j1, k1, u2, i2, j2, k2, u3, i3, j3, k3, u4, i4, j4, k4, u5, i5, j5, k5, u6, i6, j6, k6, u7, i7, j7, k7 );
}

template <typename T> Trigintaduonion<T> operator - ( T x, const Trigintaduonion<T> & z ) {
	return Trigintaduonion<T>(
		x - z.real(), -z.imag_i(), -z.imag_j(), -z.imag_k(),
		-z.imag_u1(), -z.imag_i1(), -z.imag_j1(), -z.imag_k1(),
		-z.imag_u2(), -z.imag_i2(), -z.imag_j2(), -z.imag_k2(),
		-z.imag_u3(), -z.imag_i3(), -z.imag_j3(), -z.imag_k3(),
		-z.imag_u4(), -z.imag_i4(), -z.imag_j4(), -z.imag_k4(),
		-z.imag_u5(), -z.imag_i5(), -z.imag_j5(), -z.imag_k5(),
		-z.imag_u6(), -z.imag_i6(), -z.imag_j6(), -z.imag_k6(),
		-z.imag_u7(), -z.imag_i7(), -z.imag_j7(), -z.imag_k7()
	);
}

template <typename T> Trigintaduonion<T> operator - ( long x, const Trigintaduonion<T> & z ) {
	return operator - ( static_cast<T>( x ), z );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator * ( const Trigintaduonion<T> & q ) const {
	return Trigintaduonion<T>(
		r*q.r - i*q.i - j*q.j - k*q.k - u1*q.u1 - i1*q.i1 - j1*q.j1 - k1*q.k1 - u2*q.u2 - i2*q.i2 - j2*q.j2 - k2*q.k2 - u3*q.u3 - i3*q.i3 - j3*q.j3 - k3*q.k3 - u4*q.u4 - i4*q.i4 - j4*q.j4 - k4*q.k4 - u5*q.u5 - i5*q.i5 - j5*q.j5 - k5*q.k5 - u6*q.u6 - i6*q.i6 - j6*q.j6 - k6*q.k6 - u7*q.u7 - i7*q.i7 - j7*q.j7 - k7*q.k7,
		r*q.i + i*q.r + j*q.k - k*q.j + u1*q.i1 - i1*q.u1 - j1*q.k1 + k1*q.j1 + u2*q.i2 - i2*q.u2 - j2*q.k2 + k2*q.j2 - u3*q.i3 + i3*q.u3 + j3*q.k3 - k3*q.j3 + u4*q.i4 - i4*q.u4 - j4*q.k4 + k4*q.j4 - u5*q.i5 + i5*q.u5 + j5*q.k5 - k5*q.j5 - u6*q.i6 + i6*q.u6 + j6*q.k6 - k6*q.j6 + u7*q.i7 - i7*q.u7 - j7*q.k7 + k7*q.j7,
		r*q.j - i*q.k + j*q.r + k*q.i + u1*q.j1 + i1*q.k1 - j1*q.u1 - k1*q.i1 + u2*q.j2 + i2*q.k2 - j2*q.u2 - k2*q.i2 - u3*q.j3 - i3*q.k3 + j3*q.u3 + k3*q.i3 + u4*q.j4 + i4*q.k4 - j4*q.u4 - k4*q.i4 - u5*q.j5 - i5*q.k5 + j5*q.u5 + k5*q.i5 - u6*q.j6 - i6*q.k6 + j6*q.u6 + k6*q.i6 + u7*q.j7 + i7*q.k7 - j7*q.u7 - k7*q.i7,
		r*q.k + i*q.j - j*q.i + k*q.r + u1*q.k1 - i1*q.j1 + j1*q.i1 - k1*q.u1 + u2*q.k2 - i2*q.j2 + j2*q.i2 - k2*q.u2 - u3*q.k3 + i3*q.j3 - j3*q.i3 + k3*q.u3 + u4*q.k4 - i4*q.j4 + j4*q.i4 - k4*q.u4 - u5*q.k5 + i5*q.j5 - j5*q.i5 + k5*q.u5 - u6*q.k6 + i6*q.j6 - j6*q.i6 + k6*q.u6 + u7*q.k7 - i7*q.j7 + j7*q.i7 - k7*q.u7,
		r*q.u1 - i*q.i1 - j*q.j1 - k*q.k1 + u1*q.r + i1*q.i + j1*q.j + k1*q.k + u2*q.u3 + i2*q.i3 + j2*q.j3 + k2*q.k3 - u3*q.u2 - i3*q.i2 - j3*q.j2 - k3*q.k2 + u4*q.u5 + i4*q.i5 + j4*q.j5 + k4*q.k5 - u5*q.u4 - i5*q.i4 - j5*q.j4 - k5*q.k4 - u6*q.u7 - i6*q.i7 - j6*q.j7 - k6*q.k7 + u7*q.u6 + i7*q.i6 + j7*q.j6 + k7*q.k6,
		r*q.i1 + i*q.u1 - j*q.k1 + k*q.j1 - u1*q.i + i1*q.r - j1*q.k + k1*q.j + u2*q.i3 - i2*q.u3 + j2*q.k3 - k2*q.j3 + u3*q.i2 - i3*q.u2 + j3*q.k2 - k3*q.j2 + u4*q.i5 - i4*q.u5 + j4*q.k5 - k4*q.j5 + u5*q.i4 - i5*q.u4 + j5*q.k4 - k5*q.j4 - u6*q.i7 + i6*q.u7 - j6*q.k7 + k6*q.j7 - u7*q.i6 + i7*q.u6 - j7*q.k6 + k7*q.j6,
		r*q.j1 + i*q.k1 + j*q.u1 - k*q.i1 - u1*q.j + i1*q.k + j1*q.r - k1*q.i + u2*q.j3 - i2*q.k3 - j2*q.u3 + k2*q.i3 + u3*q.j2 - i3*q.k2 - j3*q.u2 + k3*q.i2 + u4*q.j5 - i4*q.k5 - j4*q.u5 + k4*q.i5 + u5*q.j4 - i5*q.k4 - j5*q.u4 + k5*q.i4 - u6*q.j7 + i6*q.k7 + j6*q.u7 - k6*q.i7 - u7*q.j6 + i7*q.k6 + j7*q.u6 - k7*q.i6,
		r*q.k1 - i*q.j1 + j*q.i1 + k*q.u1 - u1*q.k - i1*q.j + j1*q.i + k1*q.r + u2*q.k3 + i2*q.j3 - j2*q.i3 - k2*q.u3 + u3*q.k2 + i3*q.j2 - j3*q.i2 - k3*q.u2 + u4*q.k5 + i4*q.j5 - j4*q.i5 - k4*q.u5 + u5*q.k4 + i5*q.j4 - j5*q.i4 - k5*q.u4 - u6*q.k7 - i6*q.j7 + j6*q.i7 + k6*q.u7 - u7*q.k6 - i7*q.j6 + j7*q.i6 + k7*q.u6,
		r*q.u2 - i*q.i2 - j*q.j2 - k*q.k2 - u1*q.u3 - i1*q.i3 - j1*q.j3 - k1*q.k3 + u2*q.r + i2*q.i + j2*q.j + k2*q.k + u3*q.u1 + i3*q.i1 + j3*q.j1 + k3*q.k1 + u4*q.u6 + i4*q.i6 + j4*q.j6 + k4*q.k6 + u5*q.u7 + i5*q.i7 + j5*q.j7 + k5*q.k7 - u6*q.u4 - i6*q.i4 - j6*q.j4 - k6*q.k4 - u7*q.u5 - i7*q.i5 - j7*q.j5 - k7*q.k5,
		r*q.i2 + i*q.u2 - j*q.k2 + k*q.j2 - u1*q.i3 + i1*q.u3 + j1*q.k3 - k1*q.j3 - u2*q.i + i2*q.r - j2*q.k + k2*q.j - u3*q.i1 + i3*q.u1 + j3*q.k1 - k3*q.j1 + u4*q.i6 - i4*q.u6 + j4*q.k6 - k4*q.j6 + u5*q.i7 - i5*q.u7 - j5*q.k7 + k5*q.j7 + u6*q.i4 - i6*q.u4 + j6*q.k4 - k6*q.j4 + u7*q.i5 - i7*q.u5 - j7*q.k5 + k7*q.j5,
		r*q.j2 + i*q.k2 + j*q.u2 - k*q.i2 - u1*q.j3 - i1*q.k3 + j1*q.u3 + k1*q.i3 - u2*q.j + i2*q.k + j2*q.r - k2*q.i - u3*q.j1 - i3*q.k1 + j3*q.u1 + k3*q.i1 + u4*q.j6 - i4*q.k6 - j4*q.u6 + k4*q.i6 + u5*q.j7 + i5*q.k7 - j5*q.u7 - k5*q.i7 + u6*q.j4 - i6*q.k4 - j6*q.u4 + k6*q.i4 + u7*q.j5 + i7*q.k5 - j7*q.u5 - k7*q.i5,
		r*q.k2 - i*q.j2 + j*q.i2 + k*q.u2 - u1*q.k3 + i1*q.j3 - j1*q.i3 + k1*q.u3 - u2*q.k - i2*q.j + j2*q.i + k2*q.r - u3*q.k1 + i3*q.j1 - j3*q.i1 + k3*q.u1 + u4*q.k6 + i4*q.j6 - j4*q.i6 - k4*q.u6 + u5*q.k7 - i5*q.j7 + j5*q.i7 - k5*q.u7 + u6*q.k4 + i6*q.j4 - j6*q.i4 - k6*q.u4 + u7*q.k5 - i7*q.j5 + j7*q.i5 - k7*q.u5,
		r*q.u3 + i*q.i3 + j*q.j3 + k*q.k3 + u1*q.u2 - i1*q.i2 - j1*q.j2 - k1*q.k2 - u2*q.u1 + i2*q.i1 + j2*q.j1 + k2*q.k1 + u3*q.r - i3*q.i - j3*q.j - k3*q.k + u4*q.u7 - i4*q.i7 - j4*q.j7 - k4*q.k7 - u5*q.u6 + i5*q.i6 + j5*q.j6 + k5*q.k6 + u6*q.u5 - i6*q.i5 - j6*q.j5 - k6*q.k5 - u7*q.u4 + i7*q.i4 + j7*q.j4 + k7*q.k4,
		r*q.i3 - i*q.u3 + j*q.k3 - k*q.j3 + u1*q.i2 + i1*q.u2 + j1*q.k2 - k1*q.j2 - u2*q.i1 - i2*q.u1 + j2*q.k1 - k2*q.j1 + u3*q.i + i3*q.r + j3*q.k - k3*q.j + u4*q.i7 + i4*q.u7 - j4*q.k7 + k4*q.j7 - u5*q.i6 - i5*q.u6 - j5*q.k6 + k5*q.j6 + u6*q.i5 + i6*q.u5 - j6*q.k5 + k6*q.j5 - u7*q.i4 - i7*q.u4 - j7*q.k4 + k7*q.j4,
		r*q.j3 - i*q.k3 - j*q.u3 + k*q.i3 + u1*q.j2 - i1*q.k2 + j1*q.u2 + k1*q.i2 - u2*q.j1 - i2*q.k1 - j2*q.u1 + k2*q.i1 + u3*q.j - i3*q.k + j3*q.r + k3*q.i + u4*q.j7 + i4*q.k7 + j4*q.u7 - k4*q.i7 - u5*q.j6 + i5*q.k6 - j5*q.u6 - k5*q.i6 + u6*q.j5 + i6*q.k5 + j6*q.u5 - k6*q.i5 - u7*q.j4 + i7*q.k4 - j7*q.u4 - k7*q.i4,
		r*q.k3 + i*q.j3 - j*q.i3 - k*q.u3 + u1*q.k2 + i1*q.j2 - j1*q.i2 + k1*q.u2 - u2*q.k1 + i2*q.j1 - j2*q.i1 - k2*q.u1 + u3*q.k + i3*q.j - j3*q.i + k3*q.r + u4*q.k7 - i4*q.j7 + j4*q.i7 + k4*q.u7 - u5*q.k6 - i5*q.j6 + j5*q.i6 - k5*q.u6 + u6*q.k5 - i6*q.j5 + j6*q.i5 + k6*q.u5 - u7*q.k4 - i7*q.j4 + j7*q.i4 - k7*q.u4,
		r*q.u4 - i*q.i4 - j*q.j4 - k*q.k4 - u1*q.u5 - i1*q.i5 - j1*q.j5 - k1*q.k5 - u2*q.u6 - i2*q.i6 - j2*q.j6 - k2*q.k6 - u3*q.u7 - i3*q.i7 - j3*q.j7 - k3*q.k7 + u4*q.r + i4*q.i + j4*q.j + k4*q.k + u5*q.u1 + i5*q.i1 + j5*q.j1 + k5*q.k1 + u6*q.u2 + i6*q.i2 + j6*q.j2 + k6*q.k2 + u7*q.u3 + i7*q.i3 + j7*q.j3 + k7*q.k3,
		r*q.i4 + i*q.u4 - j*q.k4 + k*q.j4 - u1*q.i5 + i1*q.u5 + j1*q.k5 - k1*q.j5 - u2*q.i6 + i2*q.u6 + j2*q.k6 - k2*q.j6 + u3*q.i7 - i3*q.u7 - j3*q.k7 + k3*q.j7 - u4*q.i + i4*q.r - j4*q.k + k4*q.j - u5*q.i1 + i5*q.u1 + j5*q.k1 - k5*q.j1 - u6*q.i2 + i6*q.u2 + j6*q.k2 - k6*q.j2 + u7*q.i3 - i7*q.u3 - j7*q.k3 + k7*q.j3,
		r*q.j4 + i*q.k4 + j*q.u4 - k*q.i4 - u1*q.j5 - i1*q.k5 + j1*q.u5 + k1*q.i5 - u2*q.j6 - i2*q.k6 + j2*q.u6 + k2*q.i6 + u3*q.j7 + i3*q.k7 - j3*q.u7 - k3*q.i7 - u4*q.j + i4*q.k + j4*q.r - k4*q.i - u5*q.j1 - i5*q.k1 + j5*q.u1 + k5*q.i1 - u6*q.j2 - i6*q.k2 + j6*q.u2 + k6*q.i2 + u7*q.j3 + i7*q.k3 - j7*q.u3 - k7*q.i3,
		r*q.k4 - i*q.j4 + j*q.i4 + k*q.u4 - u1*q.k5 + i1*q.j5 - j1*q.i5 + k1*q.u5 - u2*q.k6 + i2*q.j6 - j2*q.i6 + k2*q.u6 + u3*q.k7 - i3*q.j7 + j3*q.i7 - k3*q.u7 - u4*q.k - i4*q.j + j4*q.i + k4*q.r - u5*q.k1 + i5*q.j1 - j5*q.i1 + k5*q.u1 - u6*q.k2 + i6*q.j2 - j6*q.i2 + k6*q.u2 + u7*q.k3 - i7*q.j3 + j7*q.i3 - k7*q.u3,
		r*q.u5 + i*q.i5 + j*q.j5 + k*q.k5 + u1*q.u4 - i1*q.i4 - j1*q.j4 - k1*q.k4 - u2*q.u7 - i2*q.i7 - j2*q.j7 - k2*q.k7 + u3*q.u6 + i3*q.i6 + j3*q.j6 + k3*q.k6 - u4*q.u1 + i4*q.i1 + j4*q.j1 + k4*q.k1 + u5*q.r - i5*q.i - j5*q.j - k5*q.k - u6*q.u3 - i6*q.i3 - j6*q.j3 - k6*q.k3 + u7*q.u2 + i7*q.i2 + j7*q.j2 + k7*q.k2,
		r*q.i5 - i*q.u5 + j*q.k5 - k*q.j5 + u1*q.i4 + i1*q.u4 + j1*q.k4 - k1*q.j4 - u2*q.i7 + i2*q.u7 - j2*q.k7 + k2*q.j7 - u3*q.i6 + i3*q.u6 - j3*q.k6 + k3*q.j6 - u4*q.i1 - i4*q.u1 + j4*q.k1 - k4*q.j1 + u5*q.i + i5*q.r + j5*q.k - k5*q.j - u6*q.i3 + i6*q.u3 - j6*q.k3 + k6*q.j3 - u7*q.i2 + i7*q.u2 - j7*q.k2 + k7*q.j2,
		r*q.j5 - i*q.k5 - j*q.u5 + k*q.i5 + u1*q.j4 - i1*q.k4 + j1*q.u4 + k1*q.i4 - u2*q.j7 + i2*q.k7 + j2*q.u7 - k2*q.i7 - u3*q.j6 + i3*q.k6 + j3*q.u6 - k3*q.i6 - u4*q.j1 - i4*q.k1 - j4*q.u1 + k4*q.i1 + u5*q.j - i5*q.k + j5*q.r + k5*q.i - u6*q.j3 + i6*q.k3 + j6*q.u3 - k6*q.i3 - u7*q.j2 + i7*q.k2 + j7*q.u2 - k7*q.i2,
		r*q.k5 + i*q.j5 - j*q.i5 - k*q.u5 + u1*q.k4 + i1*q.j4 - j1*q.i4 + k1*q.u4 - u2*q.k7 - i2*q.j7 + j2*q.i7 + k2*q.u7 - u3*q.k6 - i3*q.j6 + j3*q.i6 + k3*q.u6 - u4*q.k1 + i4*q.j1 - j4*q.i1 - k4*q.u1 + u5*q.k + i5*q.j - j5*q.i + k5*q.r - u6*q.k3 - i6*q.j3 + j6*q.i3 + k6*q.u3 - u7*q.k2 - i7*q.j2 + j7*q.i2 + k7*q.u2,
		r*q.u6 + i*q.i6 + j*q.j6 + k*q.k6 + u1*q.u7 + i1*q.i7 + j1*q.j7 + k1*q.k7 + u2*q.u4 - i2*q.i4 - j2*q.j4 - k2*q.k4 - u3*q.u5 - i3*q.i5 - j3*q.j5 - k3*q.k5 - u4*q.u2 + i4*q.i2 + j4*q.j2 + k4*q.k2 + u5*q.u3 + i5*q.i3 + j5*q.j3 + k5*q.k3 + u6*q.r - i6*q.i - j6*q.j - k6*q.k - u7*q.u1 - i7*q.i1 - j7*q.j1 - k7*q.k1,
		r*q.i6 - i*q.u6 + j*q.k6 - k*q.j6 + u1*q.i7 - i1*q.u7 - j1*q.k7 + k1*q.j7 + u2*q.i4 + i2*q.u4 + j2*q.k4 - k2*q.j4 + u3*q.i5 - i3*q.u5 - j3*q.k5 + k3*q.j5 - u4*q.i2 - i4*q.u2 + j4*q.k2 - k4*q.j2 + u5*q.i3 - i5*q.u3 - j5*q.k3 + k5*q.j3 + u6*q.i + i6*q.r + j6*q.k - k6*q.j + u7*q.i1 - i7*q.u1 - j7*q.k1 + k7*q.j1,
		r*q.j6 - i*q.k6 - j*q.u6 + k*q.i6 + u1*q.j7 + i1*q.k7 - j1*q.u7 - k1*q.i7 + u2*q.j4 - i2*q.k4 + j2*q.u4 + k2*q.i4 + u3*q.j5 + i3*q.k5 - j3*q.u5 - k3*q.i5 - u4*q.j2 - i4*q.k2 - j4*q.u2 + k4*q.i2 + u5*q.j3 + i5*q.k3 - j5*q.u3 - k5*q.i3 + u6*q.j - i6*q.k + j6*q.r + k6*q.i + u7*q.j1 + i7*q.k1 - j7*q.u1 - k7*q.i1,
		r*q.k6 + i*q.j6 - j*q.i6 - k*q.u6 + u1*q.k7 - i1*q.j7 + j1*q.i7 - k1*q.u7 + u2*q.k4 + i2*q.j4 - j2*q.i4 + k2*q.u4 + u3*q.k5 - i3*q.j5 + j3*q.i5 - k3*q.u5 - u4*q.k2 + i4*q.j2 - j4*q.i2 - k4*q.u2 + u5*q.k3 - i5*q.j3 + j5*q.i3 - k5*q.u3 + u6*q.k + i6*q.j - j6*q.i + k6*q.r + u7*q.k1 - i7*q.j1 + j7*q.i1 - k7*q.u1,
		r*q.u7 - i*q.i7 - j*q.j7 - k*q.k7 - u1*q.u6 + i1*q.i6 + j1*q.j6 + k1*q.k6 + u2*q.u5 - i2*q.i5 - j2*q.j5 - k2*q.k5 + u3*q.u4 + i3*q.i4 + j3*q.j4 + k3*q.k4 - u4*q.u3 - i4*q.i3 - j4*q.j3 - k4*q.k3 - u5*q.u2 + i5*q.i2 + j5*q.j2 + k5*q.k2 + u6*q.u1 - i6*q.i1 - j6*q.j1 - k6*q.k1 + u7*q.r + i7*q.i + j7*q.j + k7*q.k,
		r*q.i7 + i*q.u7 - j*q.k7 + k*q.j7 - u1*q.i6 - i1*q.u6 - j1*q.k6 + k1*q.j6 + u2*q.i5 + i2*q.u5 - j2*q.k5 + k2*q.j5 - u3*q.i4 + i3*q.u4 - j3*q.k4 + k3*q.j4 - u4*q.i3 + i4*q.u3 - j4*q.k3 + k4*q.j3 - u5*q.i2 - i5*q.u2 - j5*q.k2 + k5*q.j2 + u6*q.i1 + i6*q.u1 - j6*q.k1 + k6*q.j1 - u7*q.i + i7*q.r - j7*q.k + k7*q.j,
		r*q.j7 + i*q.k7 + j*q.u7 - k*q.i7 - u1*q.j6 + i1*q.k6 - j1*q.u6 - k1*q.i6 + u2*q.j5 + i2*q.k5 + j2*q.u5 - k2*q.i5 - u3*q.j4 + i3*q.k4 + j3*q.u4 - k3*q.i4 - u4*q.j3 + i4*q.k3 + j4*q.u3 - k4*q.i3 - u5*q.j2 + i5*q.k2 - j5*q.u2 - k5*q.i2 + u6*q.j1 + i6*q.k1 + j6*q.u1 - k6*q.i1 - u7*q.j + i7*q.k + j7*q.r - k7*q.i,
		r*q.k7 - i*q.j7 + j*q.i7 + k*q.u7 - u1*q.k6 - i1*q.j6 + j1*q.i6 - k1*q.u6 + u2*q.k5 - i2*q.j5 + j2*q.i5 + k2*q.u5 - u3*q.k4 - i3*q.j4 + j3*q.i4 + k3*q.u4 - u4*q.k3 - i4*q.j3 + j4*q.i3 + k4*q.u3 - u5*q.k2 - i5*q.j2 + j5*q.i2 - k5*q.u2 + u6*q.k1 - i6*q.j1 + j6*q.i1 + k6*q.u1 - u7*q.k - i7*q.j + j7*q.i + k7*q.r
	);
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator * ( T x ) const {
	if ( Support<T>::is_inf( x ) && norm() > 0 ) {
		return Trigintaduonion<T>(
			( (  r == 0 )?Support<T>::sign(x):x )*r,
			( (  i == 0 )?Support<T>::sign(x):x )*i,
			( (  j == 0 )?Support<T>::sign(x):x )*j,
			( (  k == 0 )?Support<T>::sign(x):x )*k,
			( ( u1 == 0 )?Support<T>::sign(x):x )*u1,
			( ( i1 == 0 )?Support<T>::sign(x):x )*i1,
			( ( j1 == 0 )?Support<T>::sign(x):x )*j1,
			( ( k1 == 0 )?Support<T>::sign(x):x )*k1,
			( ( u2 == 0 )?Support<T>::sign(x):x )*u2,
			( ( i2 == 0 )?Support<T>::sign(x):x )*i2,
			( ( j2 == 0 )?Support<T>::sign(x):x )*j2,
			( ( k2 == 0 )?Support<T>::sign(x):x )*k2,
			( ( u3 == 0 )?Support<T>::sign(x):x )*u3,
			( ( i3 == 0 )?Support<T>::sign(x):x )*i3,
			( ( j3 == 0 )?Support<T>::sign(x):x )*j3,
			( ( k3 == 0 )?Support<T>::sign(x):x )*k3,
			( ( u4 == 0 )?Support<T>::sign(x):x )*u4,
			( ( i4 == 0 )?Support<T>::sign(x):x )*i4,
			( ( j4 == 0 )?Support<T>::sign(x):x )*j4,
			( ( k4 == 0 )?Support<T>::sign(x):x )*k4,
			( ( u5 == 0 )?Support<T>::sign(x):x )*u5,
			( ( i5 == 0 )?Support<T>::sign(x):x )*i5,
			( ( j5 == 0 )?Support<T>::sign(x):x )*j5,
			( ( k5 == 0 )?Support<T>::sign(x):x )*k5,
			( ( u6 == 0 )?Support<T>::sign(x):x )*u6,
			( ( i6 == 0 )?Support<T>::sign(x):x )*i6,
			( ( j6 == 0 )?Support<T>::sign(x):x )*j6,
			( ( k6 == 0 )?Support<T>::sign(x):x )*k6,
			( ( u7 == 0 )?Support<T>::sign(x):x )*u7,
			( ( i7 == 0 )?Support<T>::sign(x):x )*i7,
			( ( j7 == 0 )?Support<T>::sign(x):x )*j7,
			( ( k7 == 0 )?Support<T>::sign(x):x )*k7
		);
	} else {
		return Trigintaduonion<T>( x*r, x*i, x*j, x*k, x*u1, x*i1, x*j1, x*k1, x*u2, x*i2, x*j2, x*k2, x*u3, x*i3, x*j3, x*k3, x*u4, x*i4, x*j4, x*k4, x*u5, x*i5, x*j5, x*k5, x*u6, x*i6, x*j6, x*k6, x*u7, x*i7, x*j7, x*k7 );
	}
}

template <typename T> Trigintaduonion<T> operator * ( T x, const Trigintaduonion<T> & q ) {
	return q.operator * ( x );
}

template <typename T> Trigintaduonion<T> operator * ( long x, const Trigintaduonion<T> & q ) {
	return q.operator * ( static_cast<T>( x ) );
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator / ( const Trigintaduonion<T> & q ) const {
	T denom = q.norm();

	return Trigintaduonion<T>(
		(r*q.r + i*q.i + j*q.j + k*q.k + u1*q.u1 + i1*q.i1 + j1*q.j1 + k1*q.k1 + u2*q.u2 + i2*q.i2 + j2*q.j2 + k2*q.k2 + u3*q.u3 + i3*q.i3 + j3*q.j3 + k3*q.k3 + u4*q.u4 + i4*q.i4 + j4*q.j4 + k4*q.k4 + u5*q.u5 + i5*q.i5 + j5*q.j5 + k5*q.k5 + u6*q.u6 + i6*q.i6 + j6*q.j6 + k6*q.k6 + u7*q.u7 + i7*q.i7 + j7*q.j7 + k7*q.k7)/denom,
		(-r*q.i + i*q.r - j*q.k + k*q.j - u1*q.i1 + i1*q.u1 + j1*q.k1 - k1*q.j1 - u2*q.i2 + i2*q.u2 + j2*q.k2 - k2*q.j2 + u3*q.i3 - i3*q.u3 - j3*q.k3 + k3*q.j3 - u4*q.i4 + i4*q.u4 + j4*q.k4 - k4*q.j4 + u5*q.i5 - i5*q.u5 - j5*q.k5 + k5*q.j5 + u6*q.i6 - i6*q.u6 - j6*q.k6 + k6*q.j6 - u7*q.i7 + i7*q.u7 + j7*q.k7 - k7*q.j7)/denom,
		(-r*q.j + i*q.k + j*q.r - k*q.i - u1*q.j1 - i1*q.k1 + j1*q.u1 + k1*q.i1 - u2*q.j2 - i2*q.k2 + j2*q.u2 + k2*q.i2 + u3*q.j3 + i3*q.k3 - j3*q.u3 - k3*q.i3 - u4*q.j4 - i4*q.k4 + j4*q.u4 + k4*q.i4 + u5*q.j5 + i5*q.k5 - j5*q.u5 - k5*q.i5 + u6*q.j6 + i6*q.k6 - j6*q.u6 - k6*q.i6 - u7*q.j7 - i7*q.k7 + j7*q.u7 + k7*q.i7)/denom,
		(-r*q.k - i*q.j + j*q.i + k*q.r - u1*q.k1 + i1*q.j1 - j1*q.i1 + k1*q.u1 - u2*q.k2 + i2*q.j2 - j2*q.i2 + k2*q.u2 + u3*q.k3 - i3*q.j3 + j3*q.i3 - k3*q.u3 - u4*q.k4 + i4*q.j4 - j4*q.i4 + k4*q.u4 + u5*q.k5 - i5*q.j5 + j5*q.i5 - k5*q.u5 + u6*q.k6 - i6*q.j6 + j6*q.i6 - k6*q.u6 - u7*q.k7 + i7*q.j7 - j7*q.i7 + k7*q.u7)/denom,
		(-r*q.u1 + i*q.i1 + j*q.j1 + k*q.k1 + u1*q.r - i1*q.i - j1*q.j - k1*q.k - u2*q.u3 - i2*q.i3 - j2*q.j3 - k2*q.k3 + u3*q.u2 + i3*q.i2 + j3*q.j2 + k3*q.k2 - u4*q.u5 - i4*q.i5 - j4*q.j5 - k4*q.k5 + u5*q.u4 + i5*q.i4 + j5*q.j4 + k5*q.k4 + u6*q.u7 + i6*q.i7 + j6*q.j7 + k6*q.k7 - u7*q.u6 - i7*q.i6 - j7*q.j6 - k7*q.k6)/denom,
		(-r*q.i1 - i*q.u1 + j*q.k1 - k*q.j1 + u1*q.i + i1*q.r + j1*q.k - k1*q.j - u2*q.i3 + i2*q.u3 - j2*q.k3 + k2*q.j3 - u3*q.i2 + i3*q.u2 - j3*q.k2 + k3*q.j2 - u4*q.i5 + i4*q.u5 - j4*q.k5 + k4*q.j5 - u5*q.i4 + i5*q.u4 - j5*q.k4 + k5*q.j4 + u6*q.i7 - i6*q.u7 + j6*q.k7 - k6*q.j7 + u7*q.i6 - i7*q.u6 + j7*q.k6 - k7*q.j6)/denom,
		(-r*q.j1 - i*q.k1 - j*q.u1 + k*q.i1 + u1*q.j - i1*q.k + j1*q.r + k1*q.i - u2*q.j3 + i2*q.k3 + j2*q.u3 - k2*q.i3 - u3*q.j2 + i3*q.k2 + j3*q.u2 - k3*q.i2 - u4*q.j5 + i4*q.k5 + j4*q.u5 - k4*q.i5 - u5*q.j4 + i5*q.k4 + j5*q.u4 - k5*q.i4 + u6*q.j7 - i6*q.k7 - j6*q.u7 + k6*q.i7 + u7*q.j6 - i7*q.k6 - j7*q.u6 + k7*q.i6)/denom,
		(-r*q.k1 + i*q.j1 - j*q.i1 - k*q.u1 + u1*q.k + i1*q.j - j1*q.i + k1*q.r - u2*q.k3 - i2*q.j3 + j2*q.i3 + k2*q.u3 - u3*q.k2 - i3*q.j2 + j3*q.i2 + k3*q.u2 - u4*q.k5 - i4*q.j5 + j4*q.i5 + k4*q.u5 - u5*q.k4 - i5*q.j4 + j5*q.i4 + k5*q.u4 + u6*q.k7 + i6*q.j7 - j6*q.i7 - k6*q.u7 + u7*q.k6 + i7*q.j6 - j7*q.i6 - k7*q.u6)/denom,
		(-r*q.u2 + i*q.i2 + j*q.j2 + k*q.k2 + u1*q.u3 + i1*q.i3 + j1*q.j3 + k1*q.k3 + u2*q.r - i2*q.i - j2*q.j - k2*q.k - u3*q.u1 - i3*q.i1 - j3*q.j1 - k3*q.k1 - u4*q.u6 - i4*q.i6 - j4*q.j6 - k4*q.k6 - u5*q.u7 - i5*q.i7 - j5*q.j7 - k5*q.k7 + u6*q.u4 + i6*q.i4 + j6*q.j4 + k6*q.k4 + u7*q.u5 + i7*q.i5 + j7*q.j5 + k7*q.k5)/denom,
		(-r*q.i2 - i*q.u2 + j*q.k2 - k*q.j2 + u1*q.i3 - i1*q.u3 - j1*q.k3 + k1*q.j3 + u2*q.i + i2*q.r + j2*q.k - k2*q.j + u3*q.i1 - i3*q.u1 - j3*q.k1 + k3*q.j1 - u4*q.i6 + i4*q.u6 - j4*q.k6 + k4*q.j6 - u5*q.i7 + i5*q.u7 + j5*q.k7 - k5*q.j7 - u6*q.i4 + i6*q.u4 - j6*q.k4 + k6*q.j4 - u7*q.i5 + i7*q.u5 + j7*q.k5 - k7*q.j5)/denom,
		(-r*q.j2 - i*q.k2 - j*q.u2 + k*q.i2 + u1*q.j3 + i1*q.k3 - j1*q.u3 - k1*q.i3 + u2*q.j - i2*q.k + j2*q.r + k2*q.i + u3*q.j1 + i3*q.k1 - j3*q.u1 - k3*q.i1 - u4*q.j6 + i4*q.k6 + j4*q.u6 - k4*q.i6 - u5*q.j7 - i5*q.k7 + j5*q.u7 + k5*q.i7 - u6*q.j4 + i6*q.k4 + j6*q.u4 - k6*q.i4 - u7*q.j5 - i7*q.k5 + j7*q.u5 + k7*q.i5)/denom,
		(-r*q.k2 + i*q.j2 - j*q.i2 - k*q.u2 + u1*q.k3 - i1*q.j3 + j1*q.i3 - k1*q.u3 + u2*q.k + i2*q.j - j2*q.i + k2*q.r + u3*q.k1 - i3*q.j1 + j3*q.i1 - k3*q.u1 - u4*q.k6 - i4*q.j6 + j4*q.i6 + k4*q.u6 - u5*q.k7 + i5*q.j7 - j5*q.i7 + k5*q.u7 - u6*q.k4 - i6*q.j4 + j6*q.i4 + k6*q.u4 - u7*q.k5 + i7*q.j5 - j7*q.i5 + k7*q.u5)/denom,
		(-r*q.u3 - i*q.i3 - j*q.j3 - k*q.k3 - u1*q.u2 + i1*q.i2 + j1*q.j2 + k1*q.k2 + u2*q.u1 - i2*q.i1 - j2*q.j1 - k2*q.k1 + u3*q.r + i3*q.i + j3*q.j + k3*q.k - u4*q.u7 + i4*q.i7 + j4*q.j7 + k4*q.k7 + u5*q.u6 - i5*q.i6 - j5*q.j6 - k5*q.k6 - u6*q.u5 + i6*q.i5 + j6*q.j5 + k6*q.k5 + u7*q.u4 - i7*q.i4 - j7*q.j4 - k7*q.k4)/denom,
		(-r*q.i3 + i*q.u3 - j*q.k3 + k*q.j3 - u1*q.i2 - i1*q.u2 - j1*q.k2 + k1*q.j2 + u2*q.i1 + i2*q.u1 - j2*q.k1 + k2*q.j1 - u3*q.i + i3*q.r - j3*q.k + k3*q.j - u4*q.i7 - i4*q.u7 + j4*q.k7 - k4*q.j7 + u5*q.i6 + i5*q.u6 + j5*q.k6 - k5*q.j6 - u6*q.i5 - i6*q.u5 + j6*q.k5 - k6*q.j5 + u7*q.i4 + i7*q.u4 + j7*q.k4 - k7*q.j4)/denom,
		(-r*q.j3 + i*q.k3 + j*q.u3 - k*q.i3 - u1*q.j2 + i1*q.k2 - j1*q.u2 - k1*q.i2 + u2*q.j1 + i2*q.k1 + j2*q.u1 - k2*q.i1 - u3*q.j + i3*q.k + j3*q.r - k3*q.i - u4*q.j7 - i4*q.k7 - j4*q.u7 + k4*q.i7 + u5*q.j6 - i5*q.k6 + j5*q.u6 + k5*q.i6 - u6*q.j5 - i6*q.k5 - j6*q.u5 + k6*q.i5 + u7*q.j4 - i7*q.k4 + j7*q.u4 + k7*q.i4)/denom,
		(-r*q.k3 - i*q.j3 + j*q.i3 + k*q.u3 - u1*q.k2 - i1*q.j2 + j1*q.i2 - k1*q.u2 + u2*q.k1 - i2*q.j1 + j2*q.i1 + k2*q.u1 - u3*q.k - i3*q.j + j3*q.i + k3*q.r - u4*q.k7 + i4*q.j7 - j4*q.i7 - k4*q.u7 + u5*q.k6 + i5*q.j6 - j5*q.i6 + k5*q.u6 - u6*q.k5 + i6*q.j5 - j6*q.i5 - k6*q.u5 + u7*q.k4 + i7*q.j4 - j7*q.i4 + k7*q.u4)/denom,
		(-r*q.u4 + i*q.i4 + j*q.j4 + k*q.k4 + u1*q.u5 + i1*q.i5 + j1*q.j5 + k1*q.k5 + u2*q.u6 + i2*q.i6 + j2*q.j6 + k2*q.k6 + u3*q.u7 + i3*q.i7 + j3*q.j7 + k3*q.k7 + u4*q.r - i4*q.i - j4*q.j - k4*q.k - u5*q.u1 - i5*q.i1 - j5*q.j1 - k5*q.k1 - u6*q.u2 - i6*q.i2 - j6*q.j2 - k6*q.k2 - u7*q.u3 - i7*q.i3 - j7*q.j3 - k7*q.k3)/denom,
		(-r*q.i4 - i*q.u4 + j*q.k4 - k*q.j4 + u1*q.i5 - i1*q.u5 - j1*q.k5 + k1*q.j5 + u2*q.i6 - i2*q.u6 - j2*q.k6 + k2*q.j6 - u3*q.i7 + i3*q.u7 + j3*q.k7 - k3*q.j7 + u4*q.i + i4*q.r + j4*q.k - k4*q.j + u5*q.i1 - i5*q.u1 - j5*q.k1 + k5*q.j1 + u6*q.i2 - i6*q.u2 - j6*q.k2 + k6*q.j2 - u7*q.i3 + i7*q.u3 + j7*q.k3 - k7*q.j3)/denom,
		(-r*q.j4 - i*q.k4 - j*q.u4 + k*q.i4 + u1*q.j5 + i1*q.k5 - j1*q.u5 - k1*q.i5 + u2*q.j6 + i2*q.k6 - j2*q.u6 - k2*q.i6 - u3*q.j7 - i3*q.k7 + j3*q.u7 + k3*q.i7 + u4*q.j - i4*q.k + j4*q.r + k4*q.i + u5*q.j1 + i5*q.k1 - j5*q.u1 - k5*q.i1 + u6*q.j2 + i6*q.k2 - j6*q.u2 - k6*q.i2 - u7*q.j3 - i7*q.k3 + j7*q.u3 + k7*q.i3)/denom,
		(-r*q.k4 + i*q.j4 - j*q.i4 - k*q.u4 + u1*q.k5 - i1*q.j5 + j1*q.i5 - k1*q.u5 + u2*q.k6 - i2*q.j6 + j2*q.i6 - k2*q.u6 - u3*q.k7 + i3*q.j7 - j3*q.i7 + k3*q.u7 + u4*q.k + i4*q.j - j4*q.i + k4*q.r + u5*q.k1 - i5*q.j1 + j5*q.i1 - k5*q.u1 + u6*q.k2 - i6*q.j2 + j6*q.i2 - k6*q.u2 - u7*q.k3 + i7*q.j3 - j7*q.i3 + k7*q.u3)/denom,
		(-r*q.u5 - i*q.i5 - j*q.j5 - k*q.k5 - u1*q.u4 + i1*q.i4 + j1*q.j4 + k1*q.k4 + u2*q.u7 + i2*q.i7 + j2*q.j7 + k2*q.k7 - u3*q.u6 - i3*q.i6 - j3*q.j6 - k3*q.k6 + u4*q.u1 - i4*q.i1 - j4*q.j1 - k4*q.k1 + u5*q.r + i5*q.i + j5*q.j + k5*q.k + u6*q.u3 + i6*q.i3 + j6*q.j3 + k6*q.k3 - u7*q.u2 - i7*q.i2 - j7*q.j2 - k7*q.k2)/denom,
		(-r*q.i5 + i*q.u5 - j*q.k5 + k*q.j5 - u1*q.i4 - i1*q.u4 - j1*q.k4 + k1*q.j4 + u2*q.i7 - i2*q.u7 + j2*q.k7 - k2*q.j7 + u3*q.i6 - i3*q.u6 + j3*q.k6 - k3*q.j6 + u4*q.i1 + i4*q.u1 - j4*q.k1 + k4*q.j1 - u5*q.i + i5*q.r - j5*q.k + k5*q.j + u6*q.i3 - i6*q.u3 + j6*q.k3 - k6*q.j3 + u7*q.i2 - i7*q.u2 + j7*q.k2 - k7*q.j2)/denom,
		(-r*q.j5 + i*q.k5 + j*q.u5 - k*q.i5 - u1*q.j4 + i1*q.k4 - j1*q.u4 - k1*q.i4 + u2*q.j7 - i2*q.k7 - j2*q.u7 + k2*q.i7 + u3*q.j6 - i3*q.k6 - j3*q.u6 + k3*q.i6 + u4*q.j1 + i4*q.k1 + j4*q.u1 - k4*q.i1 - u5*q.j + i5*q.k + j5*q.r - k5*q.i + u6*q.j3 - i6*q.k3 - j6*q.u3 + k6*q.i3 + u7*q.j2 - i7*q.k2 - j7*q.u2 + k7*q.i2)/denom,
		(-r*q.k5 - i*q.j5 + j*q.i5 + k*q.u5 - u1*q.k4 - i1*q.j4 + j1*q.i4 - k1*q.u4 + u2*q.k7 + i2*q.j7 - j2*q.i7 - k2*q.u7 + u3*q.k6 + i3*q.j6 - j3*q.i6 - k3*q.u6 + u4*q.k1 - i4*q.j1 + j4*q.i1 + k4*q.u1 - u5*q.k - i5*q.j + j5*q.i + k5*q.r + u6*q.k3 + i6*q.j3 - j6*q.i3 - k6*q.u3 + u7*q.k2 + i7*q.j2 - j7*q.i2 - k7*q.u2)/denom,
		(-r*q.u6 - i*q.i6 - j*q.j6 - k*q.k6 - u1*q.u7 - i1*q.i7 - j1*q.j7 - k1*q.k7 - u2*q.u4 + i2*q.i4 + j2*q.j4 + k2*q.k4 + u3*q.u5 + i3*q.i5 + j3*q.j5 + k3*q.k5 + u4*q.u2 - i4*q.i2 - j4*q.j2 - k4*q.k2 - u5*q.u3 - i5*q.i3 - j5*q.j3 - k5*q.k3 + u6*q.r + i6*q.i + j6*q.j + k6*q.k + u7*q.u1 + i7*q.i1 + j7*q.j1 + k7*q.k1)/denom,
		(-r*q.i6 + i*q.u6 - j*q.k6 + k*q.j6 - u1*q.i7 + i1*q.u7 + j1*q.k7 - k1*q.j7 - u2*q.i4 - i2*q.u4 - j2*q.k4 + k2*q.j4 - u3*q.i5 + i3*q.u5 + j3*q.k5 - k3*q.j5 + u4*q.i2 + i4*q.u2 - j4*q.k2 + k4*q.j2 - u5*q.i3 + i5*q.u3 + j5*q.k3 - k5*q.j3 - u6*q.i + i6*q.r - j6*q.k + k6*q.j - u7*q.i1 + i7*q.u1 + j7*q.k1 - k7*q.j1)/denom,
		(-r*q.j6 + i*q.k6 + j*q.u6 - k*q.i6 - u1*q.j7 - i1*q.k7 + j1*q.u7 + k1*q.i7 - u2*q.j4 + i2*q.k4 - j2*q.u4 - k2*q.i4 - u3*q.j5 - i3*q.k5 + j3*q.u5 + k3*q.i5 + u4*q.j2 + i4*q.k2 + j4*q.u2 - k4*q.i2 - u5*q.j3 - i5*q.k3 + j5*q.u3 + k5*q.i3 - u6*q.j + i6*q.k + j6*q.r - k6*q.i - u7*q.j1 - i7*q.k1 + j7*q.u1 + k7*q.i1)/denom,
		(-r*q.k6 - i*q.j6 + j*q.i6 + k*q.u6 - u1*q.k7 + i1*q.j7 - j1*q.i7 + k1*q.u7 - u2*q.k4 - i2*q.j4 + j2*q.i4 - k2*q.u4 - u3*q.k5 + i3*q.j5 - j3*q.i5 + k3*q.u5 + u4*q.k2 - i4*q.j2 + j4*q.i2 + k4*q.u2 - u5*q.k3 + i5*q.j3 - j5*q.i3 + k5*q.u3 - u6*q.k - i6*q.j + j6*q.i + k6*q.r - u7*q.k1 + i7*q.j1 - j7*q.i1 + k7*q.u1)/denom,
		(-r*q.u7 + i*q.i7 + j*q.j7 + k*q.k7 + u1*q.u6 - i1*q.i6 - j1*q.j6 - k1*q.k6 - u2*q.u5 + i2*q.i5 + j2*q.j5 + k2*q.k5 - u3*q.u4 - i3*q.i4 - j3*q.j4 - k3*q.k4 + u4*q.u3 + i4*q.i3 + j4*q.j3 + k4*q.k3 + u5*q.u2 - i5*q.i2 - j5*q.j2 - k5*q.k2 - u6*q.u1 + i6*q.i1 + j6*q.j1 + k6*q.k1 + u7*q.r - i7*q.i - j7*q.j - k7*q.k)/denom,
		(-r*q.i7 - i*q.u7 + j*q.k7 - k*q.j7 + u1*q.i6 + i1*q.u6 + j1*q.k6 - k1*q.j6 - u2*q.i5 - i2*q.u5 + j2*q.k5 - k2*q.j5 + u3*q.i4 - i3*q.u4 + j3*q.k4 - k3*q.j4 + u4*q.i3 - i4*q.u3 + j4*q.k3 - k4*q.j3 + u5*q.i2 + i5*q.u2 + j5*q.k2 - k5*q.j2 - u6*q.i1 - i6*q.u1 + j6*q.k1 - k6*q.j1 + u7*q.i + i7*q.r + j7*q.k - k7*q.j)/denom,
		(-r*q.j7 - i*q.k7 - j*q.u7 + k*q.i7 + u1*q.j6 - i1*q.k6 + j1*q.u6 + k1*q.i6 - u2*q.j5 - i2*q.k5 - j2*q.u5 + k2*q.i5 + u3*q.j4 - i3*q.k4 - j3*q.u4 + k3*q.i4 + u4*q.j3 - i4*q.k3 - j4*q.u3 + k4*q.i3 + u5*q.j2 - i5*q.k2 + j5*q.u2 + k5*q.i2 - u6*q.j1 - i6*q.k1 - j6*q.u1 + k6*q.i1 + u7*q.j - i7*q.k + j7*q.r + k7*q.i)/denom,
		(-r*q.k7 + i*q.j7 - j*q.i7 - k*q.u7 + u1*q.k6 + i1*q.j6 - j1*q.i6 + k1*q.u6 - u2*q.k5 + i2*q.j5 - j2*q.i5 - k2*q.u5 + u3*q.k4 + i3*q.j4 - j3*q.i4 - k3*q.u4 + u4*q.k3 + i4*q.j3 - j4*q.i3 - k4*q.u3 + u5*q.k2 + i5*q.j2 - j5*q.i2 + k5*q.u2 - u6*q.k1 + i6*q.j1 - j6*q.i1 - k6*q.u1 + u7*q.k + i7*q.j - j7*q.i + k7*q.r)/denom
	);
}

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator / ( T x ) const {
	if ( x == 0.0 && !is_zero() ) {
		return Trigintaduonion<T>(
			 r / ( (  r == 0 )?Support<T>::sign( x ):x ),
			 i / ( (  i == 0 )?Support<T>::sign( x ):x ),
			 j / ( (  j == 0 )?Support<T>::sign( x ):x ),
			 k / ( (  k == 0 )?Support<T>::sign( x ):x ),
			u1 / ( ( u1 == 0 )?Support<T>::sign( x ):x ),
			i1 / ( ( i1 == 0 )?Support<T>::sign( x ):x ),
			j1 / ( ( j1 == 0 )?Support<T>::sign( x ):x ),
			k1 / ( ( k1 == 0 )?Support<T>::sign( x ):x ),
			u2 / ( ( u2 == 0 )?Support<T>::sign( x ):x ),
			i2 / ( ( i2 == 0 )?Support<T>::sign( x ):x ),
			j2 / ( ( j2 == 0 )?Support<T>::sign( x ):x ),
			k2 / ( ( k2 == 0 )?Support<T>::sign( x ):x ),
			u3 / ( ( u3 == 0 )?Support<T>::sign( x ):x ),
			i3 / ( ( i3 == 0 )?Support<T>::sign( x ):x ),
			j3 / ( ( j3 == 0 )?Support<T>::sign( x ):x ),
			k3 / ( ( k3 == 0 )?Support<T>::sign( x ):x ),
			u4 / ( ( u4 == 0 )?Support<T>::sign( x ):x ),
			i4 / ( ( i4 == 0 )?Support<T>::sign( x ):x ),
			j4 / ( ( j4 == 0 )?Support<T>::sign( x ):x ),
			k4 / ( ( k4 == 0 )?Support<T>::sign( x ):x ),
			u5 / ( ( u5 == 0 )?Support<T>::sign( x ):x ),
			i5 / ( ( i5 == 0 )?Support<T>::sign( x ):x ),
			j5 / ( ( j5 == 0 )?Support<T>::sign( x ):x ),
			k5 / ( ( k5 == 0 )?Support<T>::sign( x ):x ),
			u6 / ( ( u6 == 0 )?Support<T>::sign( x ):x ),
			i6 / ( ( i6 == 0 )?Support<T>::sign( x ):x ),
			j6 / ( ( j6 == 0 )?Support<T>::sign( x ):x ),
			k6 / ( ( k6 == 0 )?Support<T>::sign( x ):x ),
			u7 / ( ( u7 == 0 )?Support<T>::sign( x ):x ),
			i7 / ( ( i7 == 0 )?Support<T>::sign( x ):x ),
			j7 / ( ( j7 == 0 )?Support<T>::sign( x ):x ),
			k7 / ( ( k7 == 0 )?Support<T>::sign( x ):x )
		);
	} else {
		return Trigintaduonion<T>( r/x, i/x, j/x, k/x, u1/x, i1/x, j1/x, k1/x, u2/x, i2/x, j2/x, k2/x, u3/x, i3/x, j3/x, k3/x, u4/x, i4/x, j4/x, k4/x, u5/x, i5/x, j5/x, k5/x, u6/x, i6/x, j6/x, k6/x, u7/x, i7/x, j7/x, k7/x );
	}
}

template <typename T> Trigintaduonion<T> operator / ( T x, const Trigintaduonion<T> & q ) {
	T mltplr = x/q.norm();

	return Trigintaduonion<T>(
		mltplr*q.real(),
		-mltplr*q.imag_i(),  -mltplr*q.imag_j(),  -mltplr*q.imag_k(),
		-mltplr*q.imag_u1(), -mltplr*q.imag_i1(), -mltplr*q.imag_j1(), -mltplr*q.imag_k1(),
		-mltplr*q.imag_u2(), -mltplr*q.imag_i2(), -mltplr*q.imag_j2(), -mltplr*q.imag_k2(),
		-mltplr*q.imag_u3(), -mltplr*q.imag_i3(), -mltplr*q.imag_j3(), -mltplr*q.imag_k3(),
		-mltplr*q.imag_u4(), -mltplr*q.imag_i4(), -mltplr*q.imag_j4(), -mltplr*q.imag_k4(),
		-mltplr*q.imag_u5(), -mltplr*q.imag_i5(), -mltplr*q.imag_j5(), -mltplr*q.imag_k5(),
		-mltplr*q.imag_u6(), -mltplr*q.imag_i6(), -mltplr*q.imag_j6(), -mltplr*q.imag_k6(),
		-mltplr*q.imag_u7(), -mltplr*q.imag_i7(), -mltplr*q.imag_j7(), -mltplr*q.imag_k7()
	);
}

template <typename T> Trigintaduonion<T> operator / ( long x, const Trigintaduonion<T> & q ) {
	return operator / ( static_cast<T>( x ), q );
}

/******************************************
 * Unary Arithmetic Operators
 ******************************************/

template <typename T> Trigintaduonion<T> Trigintaduonion<T>::operator - () const {
	return Trigintaduonion<T>( -r, -i, -j, -k, -u1, -i1, -j1, -k1, -u2, -i2, -j2, -k2, -u3, -i3, -j3, -k3, -u4, -i4, -j4, -k4, -u5, -i5, -j5, -k5, -u6, -i6, -j6, -k6, -u7, -i7, -j7, -k7 );
}

/******************************************
 * Binary Boolean Operators
 ******************************************/

template <typename T> bool Trigintaduonion<T>::operator == ( const Trigintaduonion<T> & q ) const {
	return
		( r == q.r ) && ( i == q.i ) && ( j == q.j ) && ( k == q.k ) &&
		( u1 == q.u1 ) && ( i1 == q.i1 ) && ( j1 == q.j1 ) && ( k1 == q.k1 ) &&
		( u2 == q.u2 ) && ( i2 == q.i2 ) && ( j2 == q.j2 ) && ( k2 == q.k2 ) &&
		( u3 == q.u3 ) && ( i3 == q.i3 ) && ( j3 == q.j3 ) && ( k3 == q.k3 ) &&
		( u4 == q.u4 ) && ( i4 == q.i4 ) && ( j4 == q.j4 ) && ( k4 == q.k4 ) &&
		( u5 == q.u5 ) && ( i5 == q.i5 ) && ( j5 == q.j5 ) && ( k5 == q.k5 ) &&
		( u6 == q.u6 ) && ( i6 == q.i6 ) && ( j6 == q.j6 ) && ( k6 == q.k6 ) &&
		( u7 == q.u7 ) && ( i7 == q.i7 ) && ( j7 == q.j7 ) && ( k7 == q.k7 );
}

template <typename T> bool Trigintaduonion<T>::operator == ( T x ) const {
	return
		( r == x ) && ( i == 0.0 ) && ( j == 0.0 ) && ( k == 0.0 ) &&
		( u1 == 0.0 ) && ( i1 == 0.0 ) && ( j1 == 0.0 ) && ( k1 == 0.0 ) &&
		( u2 == 0.0 ) && ( i2 == 0.0 ) && ( j2 == 0.0 ) && ( k2 == 0.0 ) &&
		( u3 == 0.0 ) && ( i3 == 0.0 ) && ( j3 == 0.0 ) && ( k3 == 0.0 ) &&
		( u4 == 0.0 ) && ( i4 == 0.0 ) && ( j4 == 0.0 ) && ( k4 == 0.0 ) &&
		( u5 == 0.0 ) && ( i5 == 0.0 ) && ( j5 == 0.0 ) && ( k5 == 0.0 ) &&
		( u6 == 0.0 ) && ( i6 == 0.0 ) && ( j6 == 0.0 ) && ( k6 == 0.0 ) &&
		( u7 == 0.0 ) && ( i7 == 0.0 ) && ( j7 == 0.0 ) && ( k7 == 0.0 );
}

template <typename T> bool operator == ( T x, const Trigintaduonion<T> & q ) {
	return q.operator == ( x );
}

template <typename T> bool operator == ( long x, const Trigintaduonion<T> & q ) {
	return q.operator == ( static_cast<T>( x ) );
}

template <typename T> bool Trigintaduonion<T>::operator != ( const Trigintaduonion<T> & q ) const {
	return
		( r != q.r ) || ( i != q.i ) || ( j != q.j ) || ( k != q.k ) ||
		( u1 != q.u1 ) && ( i1 != q.i1 ) && ( j1 != q.j1 ) && ( k1 != q.k1 ) ||
		( u2 != q.u2 ) && ( i2 != q.i2 ) && ( j2 != q.j2 ) && ( k2 != q.k2 ) ||
		( u3 != q.u3 ) && ( i3 != q.i3 ) && ( j3 != q.j3 ) && ( k3 != q.k3 ) ||
		( u4 != q.u4 ) && ( i4 != q.i4 ) && ( j4 != q.j4 ) && ( k4 != q.k4 ) ||
		( u5 != q.u5 ) && ( i5 != q.i5 ) && ( j5 != q.j5 ) && ( k5 != q.k5 ) ||
		( u6 != q.u6 ) && ( i6 != q.i6 ) && ( j6 != q.j6 ) && ( k6 != q.k6 ) ||
		( u7 != q.u7 ) && ( i7 != q.i7 ) && ( j7 != q.j7 ) && ( k7 != q.k7 );
}

template <typename T> bool Trigintaduonion<T>::operator != ( T x ) const {
	return
		( r != x ) || ( i != 0.0 ) || ( j != 0.0 ) || ( k != 0.0 ) ||
		( u1 != 0.0 ) && ( i1 != 0.0 ) && ( j1 != 0.0 ) && ( k1 != 0.0 ) ||
		( u2 != 0.0 ) && ( i2 != 0.0 ) && ( j2 != 0.0 ) && ( k2 != 0.0 ) ||
		( u3 != 0.0 ) && ( i3 != 0.0 ) && ( j3 != 0.0 ) && ( k3 != 0.0 ) ||
		( u4 != 0.0 ) && ( i4 != 0.0 ) && ( j4 != 0.0 ) && ( k4 != 0.0 ) ||
		( u5 != 0.0 ) && ( i5 != 0.0 ) && ( j5 != 0.0 ) && ( k5 != 0.0 ) ||
		( u6 != 0.0 ) && ( i6 != 0.0 ) && ( j6 != 0.0 ) && ( k6 != 0.0 ) ||
		( u7 != 0.0 ) && ( i7 != 0.0 ) && ( j7 != 0.0 ) && ( k7 != 0.0 );
}

template <typename T> bool operator != ( T x, const Trigintaduonion<T> & q ) {
	return q.operator != ( x );
}

template <typename T> bool operator != ( long x, const Trigintaduonion<T> & q ) {
	return q.operator != ( static_cast<T>( x ) );
}

/******************************************
 * IO Stream Operators
 ******************************************/

template <typename T> std::ostream & operator << ( std::ostream & out, const Trigintaduonion<T> & z ) {
	Support<T>::print_real( z.real(), out );
	Support<T>::print_imaginary( z.imag_i(), 'i', out );
	Support<T>::print_imaginary( z.imag_j(), 'j', out );
	Support<T>::print_imaginary( z.imag_k(), 'k', out );
	Support<T>::print_imaginary( z.imag_u1(), "u1", out );
	Support<T>::print_imaginary( z.imag_i1(), "i1", out );
	Support<T>::print_imaginary( z.imag_j1(), "j1", out );
	Support<T>::print_imaginary( z.imag_k1(), "k1", out );
	Support<T>::print_imaginary( z.imag_u2(), "u2", out );
	Support<T>::print_imaginary( z.imag_i2(), "i2", out );
	Support<T>::print_imaginary( z.imag_j2(), "j2", out );
	Support<T>::print_imaginary( z.imag_k2(), "k2", out );
	Support<T>::print_imaginary( z.imag_u3(), "u3", out );
	Support<T>::print_imaginary( z.imag_i3(), "i3", out );
	Support<T>::print_imaginary( z.imag_j3(), "j3", out );
	Support<T>::print_imaginary( z.imag_k3(), "k3", out );
	Support<T>::print_imaginary( z.imag_u4(), "u4", out );
	Support<T>::print_imaginary( z.imag_i4(), "i4", out );
	Support<T>::print_imaginary( z.imag_j4(), "j4", out );
	Support<T>::print_imaginary( z.imag_k4(), "k4", out );
	Support<T>::print_imaginary( z.imag_u5(), "u5", out );
	Support<T>::print_imaginary( z.imag_i5(), "i5", out );
	Support<T>::print_imaginary( z.imag_j5(), "j5", out );
	Support<T>::print_imaginary( z.imag_k5(), "k5", out );
	Support<T>::print_imaginary( z.imag_u6(), "u6", out );
	Support<T>::print_imaginary( z.imag_i6(), "i6", out );
	Support<T>::print_imaginary( z.imag_j6(), "j6", out );
	Support<T>::print_imaginary( z.imag_k6(), "k6", out );
	Support<T>::print_imaginary( z.imag_u7(), "u7", out );
	Support<T>::print_imaginary( z.imag_i7(), "i7", out );
	Support<T>::print_imaginary( z.imag_j7(), "j7", out );
	Support<T>::print_imaginary( z.imag_k7(), "k7", out );

	return out;
}

/******************************************
 * ************************************** *
 * *                                    * *
 * *      Procedural Functions          * *
 * *                                    * *
 * ************************************** *
 ******************************************/

/******************************************
 * Real-valued Functions
 ******************************************/

template <typename T> T real( const Trigintaduonion<T> & q ) {
	return q.real();
}

template <typename T> T imag_i( const Trigintaduonion<T> & q ) {
	return q.imag_i();
}

template <typename T> T imag_j( const Trigintaduonion<T> & q ) {
	return q.imag_j();
}

template <typename T> T imag_k( const Trigintaduonion<T> & q ) {
	return q.imag_k();
}

template <typename T> T imag_u1( const Trigintaduonion<T> & q ) {
	return q.imag_u1();
}

template <typename T> T imag_i1( const Trigintaduonion<T> & q ) {
	return q.imag_i1();
}

template <typename T> T imag_j1( const Trigintaduonion<T> & q ) {
	return q.imag_j1();
}

template <typename T> T imag_k1( const Trigintaduonion<T> & q ) {
	return q.imag_k1();
}

template <typename T> T imag_u2( const Trigintaduonion<T> & q ) {
	return q.imag_u2();
}

template <typename T> T imag_i2( const Trigintaduonion<T> & q ) {
	return q.imag_i2();
}

template <typename T> T imag_j2( const Trigintaduonion<T> & q ) {
	return q.imag_j2();
}

template <typename T> T imag_k2( const Trigintaduonion<T> & q ) {
	return q.imag_k2();
}

template <typename T> T imag_u3( const Trigintaduonion<T> & q ) {
	return q.imag_u3();
}

template <typename T> T imag_i3( const Trigintaduonion<T> & q ) {
	return q.imag_i3();
}

template <typename T> T imag_j3( const Trigintaduonion<T> & q ) {
	return q.imag_j3();
}

template <typename T> T imag_k3( const Trigintaduonion<T> & q ) {
	return q.imag_k3();
}

template <typename T> T imag_u4( const Trigintaduonion<T> & q ) {
	return q.imag_u4();
}

template <typename T> T imag_i4( const Trigintaduonion<T> & q ) {
	return q.imag_i4();
}

template <typename T> T imag_j4( const Trigintaduonion<T> & q ) {
	return q.imag_j4();
}

template <typename T> T imag_k4( const Trigintaduonion<T> & q ) {
	return q.imag_k4();
}

template <typename T> T imag_u5( const Trigintaduonion<T> & q ) {
	return q.imag_u5();
}

template <typename T> T imag_i5( const Trigintaduonion<T> & q ) {
	return q.imag_i5();
}

template <typename T> T imag_j5( const Trigintaduonion<T> & q ) {
	return q.imag_j5();
}

template <typename T> T imag_k5( const Trigintaduonion<T> & q ) {
	return q.imag_k5();
}

template <typename T> T imag_u6( const Trigintaduonion<T> & q ) {
	return q.imag_u6();
}

template <typename T> T imag_i6( const Trigintaduonion<T> & q ) {
	return q.imag_i6();
}

template <typename T> T imag_j6( const Trigintaduonion<T> & q ) {
	return q.imag_j6();
}

template <typename T> T imag_k6( const Trigintaduonion<T> & q ) {
	return q.imag_k6();
}

template <typename T> T imag_u7( const Trigintaduonion<T> & q ) {
	return q.imag_u7();
}

template <typename T> T imag_i7( const Trigintaduonion<T> & q ) {
	return q.imag_i7();
}

template <typename T> T imag_j7( const Trigintaduonion<T> & q ) {
	return q.imag_j7();
}

template <typename T> T imag_k7( const Trigintaduonion<T> & q ) {
	return q.imag_k7();
}

template <typename T> T csgn( const Trigintaduonion<T> & q ) {
	return q.csgn();
}

template <typename T> T abs( const Trigintaduonion<T> & q ) {
	return q.abs();
}

template <typename T> T norm( const Trigintaduonion<T> & q ) {
	return q.norm();
}

template <typename T> T abs_imag( const Trigintaduonion<T> & z ) {
	return z.abs_imag();
}

template <typename T> T norm_imag( const Trigintaduonion<T> & z ) {
	return z.norm_imag();
}

/******************************************
 * Trigintaduonion-valued Functions
 ******************************************/

template <typename T> Trigintaduonion<T> imag( const Trigintaduonion<T> & q ) {
	return q.imag();
}

template <typename T> Trigintaduonion<T> conj( const Trigintaduonion<T> & q ) {
	return q.conj();
}

template <typename T> Trigintaduonion<T> signum( const Trigintaduonion<T> & q ) {
	return q.signum();
}

template <typename T> Trigintaduonion<T> sqr( const Trigintaduonion<T> & q ) {
	return q.sqr();
}

template <typename T> Trigintaduonion<T> sqrt( const Trigintaduonion<T> & q ) {
	return q.sqrt();
}

template <typename T> Trigintaduonion<T> rotate( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return q.rotate( p );
}

/******************************************
 * Exponential and Logarithmic Functions
 ******************************************/

template <typename T> Trigintaduonion<T> exp( const Trigintaduonion<T> & q ) {
	return q.exp();
}

template <typename T> Trigintaduonion<T> log( const Trigintaduonion<T> & q ) {
	return q.log();
}

template <typename T> Trigintaduonion<T> log10( const Trigintaduonion<T> & q ) {
	return q.log10();
}

template <typename T> Trigintaduonion<T> pow( const Trigintaduonion<T> & q, const Trigintaduonion<T> & w ) {
	return q.pow( w );
}

template <typename T> Trigintaduonion<T> pow( const Trigintaduonion<T> & q, T x ) {
	return q.pow( x );
}

template <typename T> Trigintaduonion<T> inverse( const Trigintaduonion<T> & q ) {
	return q.inverse();
}

/******************************************
 * Trigonometric and Hyperbolic Functions
 ******************************************/

template <typename T> Trigintaduonion<T> sin( const Trigintaduonion<T> & q ) {
	return q.sin();
}

template <typename T> Trigintaduonion<T> cos( const Trigintaduonion<T> & q ) {
	return q.cos();
}

template <typename T> Trigintaduonion<T> tan( const Trigintaduonion<T> & q ) {
	return q.tan();
}

template <typename T> Trigintaduonion<T> sec( const Trigintaduonion<T> & q ) {
	return q.sec();
}

template <typename T> Trigintaduonion<T> csc( const Trigintaduonion<T> & q ) {
	return q.csc();
}

template <typename T> Trigintaduonion<T> cot( const Trigintaduonion<T> & q ) {
	return q.cot();
}

template <typename T> Trigintaduonion<T> sinh( const Trigintaduonion<T> & q ) {
	return q.sinh();
}

template <typename T> Trigintaduonion<T> cosh( const Trigintaduonion<T> & q ) {
	return q.cosh();
}

template <typename T> Trigintaduonion<T> tanh( const Trigintaduonion<T> & q ) {
	return q.tanh();
}

template <typename T> Trigintaduonion<T> sech( const Trigintaduonion<T> & q ) {
	return q.sech();
}

template <typename T> Trigintaduonion<T> csch( const Trigintaduonion<T> & q ) {
	return q.csch();
}

template <typename T> Trigintaduonion<T> coth( const Trigintaduonion<T> & q ) {
	return q.coth();
}

template <typename T> Trigintaduonion<T> asin( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return q.asin( p );
}

template <typename T> Trigintaduonion<T> acos( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return q.acos( p );
}

template <typename T> Trigintaduonion<T> atan( const Trigintaduonion<T> & q ) {
	return q.atan();
}

template <typename T> Trigintaduonion<T> asec( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return q.asec( p );
}

template <typename T> Trigintaduonion<T> acsc( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return p.acsc( p );
}

template <typename T> Trigintaduonion<T> acot( const Trigintaduonion<T> & q ) {
	return q.acot();
}

template <typename T> Trigintaduonion<T> asinh( const Trigintaduonion<T> & q ) {
	return q.asinh();
}

template <typename T> Trigintaduonion<T> acosh( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return q.acosh( p );
}

template <typename T> Trigintaduonion<T> atanh( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return q.atanh( p );
}

template <typename T> Trigintaduonion<T> asech( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return q.asech( p );
}

template <typename T> Trigintaduonion<T> acsch( const Trigintaduonion<T> & q ) {
	return q.acsch();
}

template <typename T> Trigintaduonion<T> acoth( const Trigintaduonion<T> & q, const Trigintaduonion<T> & p ) {
	return q.acoth( p );
}

template <typename T> Trigintaduonion<T> bessel_J( int n, const Trigintaduonion<T> & z ) {
	return z.bessel_J( n );
}

/******************************************
 * Integer Functions
 ******************************************/

template <typename T> Trigintaduonion<T> floor( const Trigintaduonion<T> & q ) {
	return q.floor();
}

template <typename T> Trigintaduonion<T> ceil( const Trigintaduonion<T> & q ) {
	return q.ceil();
}

/******************************************
 * Horner's Rule
 ******************************************/

template <typename T> Trigintaduonion<T> horner( const Trigintaduonion<T> & q, T * v, unsigned int n ) {
	return q.horner( v, n );
}

template <typename T> Trigintaduonion<T> horner( const Trigintaduonion<T> & q, T * v, T * c, unsigned int n ) {
	return q.horner( v, c, n );
}

/**************************************************
 * ********************************************** *
 * *                                            * *
 * *    Double-precision Floating-point         * *
 * *    Instance of Template                    * *
 * *                                            * *
 * ********************************************** *
 **************************************************/

template class Trigintaduonion<double>;

template std::ostream & operator << ( std::ostream &, const Trigintaduonion<double> & );

template Trigintaduonion<double> operator + ( double, const Trigintaduonion<double> & );
template Trigintaduonion<double> operator + ( long, const Trigintaduonion<double> & );
template Trigintaduonion<double> operator - ( double, const Trigintaduonion<double> & );
template Trigintaduonion<double> operator - ( long, const Trigintaduonion<double> & );
template Trigintaduonion<double> operator * ( double, const Trigintaduonion<double> & );
template Trigintaduonion<double> operator * ( long, const Trigintaduonion<double> & );
template Trigintaduonion<double> operator / ( double, const Trigintaduonion<double> & );
template Trigintaduonion<double> operator / ( long, const Trigintaduonion<double> & );

template bool operator == ( double, const Trigintaduonion<double> & );
template bool operator == ( long, const Trigintaduonion<double> & );
template bool operator != ( double, const Trigintaduonion<double> & );
template bool operator != ( long, const Trigintaduonion<double> & );

template <> const Trigintaduonion<double> Trigintaduonion<double>::ZERO = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>  Trigintaduonion<double>::ONE = Trigintaduonion<double>( 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>    Trigintaduonion<double>::I = Trigintaduonion<double>( 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>    Trigintaduonion<double>::J = Trigintaduonion<double>( 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>    Trigintaduonion<double>::K = Trigintaduonion<double>( 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::U1 = Trigintaduonion<double>( 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::I1 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::J1 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::K1 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::U2 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::I2 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::J2 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::K2 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::U3 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::I3 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::J3 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::K3 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::U4 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::I4 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::J4 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::K4 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::U5 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::I5 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::J5 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::K5 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::U6 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::I6 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::J6 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::K6 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::U7 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::I7 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::J7 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 );
template <> const Trigintaduonion<double>   Trigintaduonion<double>::K7 = Trigintaduonion<double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 );

template <> const Trigintaduonion<double> Trigintaduonion<double>::UNITS[32] = {
	Trigintaduonion<double>::ONE,
	Trigintaduonion<double>::I,
	Trigintaduonion<double>::J,
	Trigintaduonion<double>::K,
	Trigintaduonion<double>::U1,
	Trigintaduonion<double>::I1,
	Trigintaduonion<double>::J1,
	Trigintaduonion<double>::K1,
	Trigintaduonion<double>::U2,
	Trigintaduonion<double>::I2,
	Trigintaduonion<double>::J2,
	Trigintaduonion<double>::K2,
	Trigintaduonion<double>::U3,
	Trigintaduonion<double>::I3,
	Trigintaduonion<double>::J3,
	Trigintaduonion<double>::K3,
	Trigintaduonion<double>::U4,
	Trigintaduonion<double>::I4,
	Trigintaduonion<double>::J4,
	Trigintaduonion<double>::K4,
	Trigintaduonion<double>::U5,
	Trigintaduonion<double>::I5,
	Trigintaduonion<double>::J5,
	Trigintaduonion<double>::K5,
	Trigintaduonion<double>::U6,
	Trigintaduonion<double>::I6,
	Trigintaduonion<double>::J6,
	Trigintaduonion<double>::K6,
	Trigintaduonion<double>::U7,
	Trigintaduonion<double>::I7,
	Trigintaduonion<double>::J7,
	Trigintaduonion<double>::K7
};

template double real( const Trigintaduonion<double> & );
template double imag_i( const Trigintaduonion<double> & );
template double imag_j( const Trigintaduonion<double> & );
template double imag_k( const Trigintaduonion<double> & );
template double imag_u1( const Trigintaduonion<double> & );
template double imag_i1( const Trigintaduonion<double> & );
template double imag_j1( const Trigintaduonion<double> & );
template double imag_k1( const Trigintaduonion<double> & );
template double imag_u2( const Trigintaduonion<double> & );
template double imag_i2( const Trigintaduonion<double> & );
template double imag_j2( const Trigintaduonion<double> & );
template double imag_k2( const Trigintaduonion<double> & );
template double imag_u3( const Trigintaduonion<double> & );
template double imag_i3( const Trigintaduonion<double> & );
template double imag_j3( const Trigintaduonion<double> & );
template double imag_k3( const Trigintaduonion<double> & );
template double imag_u4( const Trigintaduonion<double> & );
template double imag_i4( const Trigintaduonion<double> & );
template double imag_j4( const Trigintaduonion<double> & );
template double imag_k4( const Trigintaduonion<double> & );
template double imag_u5( const Trigintaduonion<double> & );
template double imag_i5( const Trigintaduonion<double> & );
template double imag_j5( const Trigintaduonion<double> & );
template double imag_k5( const Trigintaduonion<double> & );
template double imag_u6( const Trigintaduonion<double> & );
template double imag_i6( const Trigintaduonion<double> & );
template double imag_j6( const Trigintaduonion<double> & );
template double imag_k6( const Trigintaduonion<double> & );
template double imag_u7( const Trigintaduonion<double> & );
template double imag_i7( const Trigintaduonion<double> & );
template double imag_j7( const Trigintaduonion<double> & );
template double imag_k7( const Trigintaduonion<double> & );
template double csgn( const Trigintaduonion<double> & );
template double abs( const Trigintaduonion<double> & );
template double norm( const Trigintaduonion<double> & );
template double abs_imag( const Trigintaduonion<double> & );
template double norm_imag( const Trigintaduonion<double> & );
template Trigintaduonion<double> imag( const Trigintaduonion<double> & );
template Trigintaduonion<double> conj( const Trigintaduonion<double> & );
template Trigintaduonion<double> signum( const Trigintaduonion<double> & );
template Trigintaduonion<double> sqr( const Trigintaduonion<double> & );
template Trigintaduonion<double> sqrt( const Trigintaduonion<double> & );
template Trigintaduonion<double> rotate( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> exp( const Trigintaduonion<double> & );
template Trigintaduonion<double> log( const Trigintaduonion<double> & );
template Trigintaduonion<double> log10( const Trigintaduonion<double> & );
template Trigintaduonion<double> pow( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> pow( const Trigintaduonion<double> &, double );
template Trigintaduonion<double> inverse( const Trigintaduonion<double> & );
template Trigintaduonion<double> sin( const Trigintaduonion<double> & );
template Trigintaduonion<double> cos( const Trigintaduonion<double> & );
template Trigintaduonion<double> tan( const Trigintaduonion<double> & );
template Trigintaduonion<double> sec( const Trigintaduonion<double> & );
template Trigintaduonion<double> csc( const Trigintaduonion<double> & );
template Trigintaduonion<double> cot( const Trigintaduonion<double> & );
template Trigintaduonion<double> sinh( const Trigintaduonion<double> & );
template Trigintaduonion<double> cosh( const Trigintaduonion<double> & );
template Trigintaduonion<double> tanh( const Trigintaduonion<double> & );
template Trigintaduonion<double> sech( const Trigintaduonion<double> & );
template Trigintaduonion<double> csch( const Trigintaduonion<double> & );
template Trigintaduonion<double> coth( const Trigintaduonion<double> & );
template Trigintaduonion<double> asin( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> acos( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> atan( const Trigintaduonion<double> & );
template Trigintaduonion<double> asec( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> acsc( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> acot( const Trigintaduonion<double> & );
template Trigintaduonion<double> asinh( const Trigintaduonion<double> & );
template Trigintaduonion<double> acosh( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> atanh( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> asech( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> acsch( const Trigintaduonion<double> & );
template Trigintaduonion<double> acoth( const Trigintaduonion<double> &, const Trigintaduonion<double> & );
template Trigintaduonion<double> bessel_J( int, const Trigintaduonion<double> & );
template Trigintaduonion<double> floor( const Trigintaduonion<double> & );
template Trigintaduonion<double> ceil( const Trigintaduonion<double> & );
template Trigintaduonion<double> horner( const Trigintaduonion<double> &, double *, unsigned int );
template Trigintaduonion<double> horner( const Trigintaduonion<double> &, double *, double *, unsigned int );

/**************************************************
 * ********************************************** *
 * *                                            * *
 * *    Floating-point Instance of Template     * *
 * *                                            * *
 * ********************************************** *
 **************************************************/

template class Trigintaduonion<float>;
template std::ostream & operator << ( std::ostream &, const Trigintaduonion<float> & );

template Trigintaduonion<float> operator + ( float, const Trigintaduonion<float> & );
template Trigintaduonion<float> operator + ( long, const Trigintaduonion<float> & );
template Trigintaduonion<float> operator - ( float, const Trigintaduonion<float> & );
template Trigintaduonion<float> operator - ( long, const Trigintaduonion<float> & );
template Trigintaduonion<float> operator * ( float, const Trigintaduonion<float> & );
template Trigintaduonion<float> operator * ( long, const Trigintaduonion<float> & );
template Trigintaduonion<float> operator / ( float, const Trigintaduonion<float> & );
template Trigintaduonion<float> operator / ( long, const Trigintaduonion<float> & );

template bool operator == ( float, const Trigintaduonion<float> & );
template bool operator == ( long, const Trigintaduonion<float> & );
template bool operator != ( float, const Trigintaduonion<float> & );
template bool operator != ( long, const Trigintaduonion<float> & );

template <> const Trigintaduonion<float> Trigintaduonion<float>::ZERO = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>  Trigintaduonion<float>::ONE = Trigintaduonion<float>( 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>    Trigintaduonion<float>::I = Trigintaduonion<float>( 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>    Trigintaduonion<float>::J = Trigintaduonion<float>( 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>    Trigintaduonion<float>::K = Trigintaduonion<float>( 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::U1 = Trigintaduonion<float>( 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::I1 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::J1 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::K1 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::U2 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::I2 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::J2 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::K2 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::U3 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::I3 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::J3 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::K3 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::U4 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::I4 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::J4 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::K4 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::U5 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::I5 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::J5 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::K5 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::U6 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::I6 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::J6 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::K6 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::U7 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::I7 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::J7 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 );
template <> const Trigintaduonion<float>   Trigintaduonion<float>::K7 = Trigintaduonion<float>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 );

template <> const Trigintaduonion<float> Trigintaduonion<float>::UNITS[32] = {
	Trigintaduonion<float>::ONE,
	Trigintaduonion<float>::I,
	Trigintaduonion<float>::J,
	Trigintaduonion<float>::K,
	Trigintaduonion<float>::U1,
	Trigintaduonion<float>::I1,
	Trigintaduonion<float>::J1,
	Trigintaduonion<float>::K1,
	Trigintaduonion<float>::U2,
	Trigintaduonion<float>::I2,
	Trigintaduonion<float>::J2,
	Trigintaduonion<float>::K2,
	Trigintaduonion<float>::U3,
	Trigintaduonion<float>::I3,
	Trigintaduonion<float>::J3,
	Trigintaduonion<float>::K3,
	Trigintaduonion<float>::U4,
	Trigintaduonion<float>::I4,
	Trigintaduonion<float>::J4,
	Trigintaduonion<float>::K4,
	Trigintaduonion<float>::U5,
	Trigintaduonion<float>::I5,
	Trigintaduonion<float>::J5,
	Trigintaduonion<float>::K5,
	Trigintaduonion<float>::U6,
	Trigintaduonion<float>::I6,
	Trigintaduonion<float>::J6,
	Trigintaduonion<float>::K6,
	Trigintaduonion<float>::U7,
	Trigintaduonion<float>::I7,
	Trigintaduonion<float>::J7,
	Trigintaduonion<float>::K7
};

template float real( const Trigintaduonion<float> & );
template float imag_i( const Trigintaduonion<float> & );
template float imag_j( const Trigintaduonion<float> & );
template float imag_k( const Trigintaduonion<float> & );
template float imag_u1( const Trigintaduonion<float> & );
template float imag_i1( const Trigintaduonion<float> & );
template float imag_j1( const Trigintaduonion<float> & );
template float imag_k1( const Trigintaduonion<float> & );
template float imag_u2( const Trigintaduonion<float> & );
template float imag_i2( const Trigintaduonion<float> & );
template float imag_j2( const Trigintaduonion<float> & );
template float imag_k2( const Trigintaduonion<float> & );
template float imag_u3( const Trigintaduonion<float> & );
template float imag_i3( const Trigintaduonion<float> & );
template float imag_j3( const Trigintaduonion<float> & );
template float imag_k3( const Trigintaduonion<float> & );
template float imag_u4( const Trigintaduonion<float> & );
template float imag_i4( const Trigintaduonion<float> & );
template float imag_j4( const Trigintaduonion<float> & );
template float imag_k4( const Trigintaduonion<float> & );
template float imag_u5( const Trigintaduonion<float> & );
template float imag_i5( const Trigintaduonion<float> & );
template float imag_j5( const Trigintaduonion<float> & );
template float imag_k5( const Trigintaduonion<float> & );
template float imag_u6( const Trigintaduonion<float> & );
template float imag_i6( const Trigintaduonion<float> & );
template float imag_j6( const Trigintaduonion<float> & );
template float imag_k6( const Trigintaduonion<float> & );
template float imag_u7( const Trigintaduonion<float> & );
template float imag_i7( const Trigintaduonion<float> & );
template float imag_j7( const Trigintaduonion<float> & );
template float imag_k7( const Trigintaduonion<float> & );
template float csgn( const Trigintaduonion<float> & );
template float abs( const Trigintaduonion<float> & );
template float norm( const Trigintaduonion<float> & );
template float abs_imag( const Trigintaduonion<float> & );
template float norm_imag( const Trigintaduonion<float> & );
template Trigintaduonion<float> imag( const Trigintaduonion<float> & );
template Trigintaduonion<float> conj( const Trigintaduonion<float> & );
template Trigintaduonion<float> signum( const Trigintaduonion<float> & );
template Trigintaduonion<float> sqr( const Trigintaduonion<float> & );
template Trigintaduonion<float> sqrt( const Trigintaduonion<float> & );
template Trigintaduonion<float> rotate( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> exp( const Trigintaduonion<float> & );
template Trigintaduonion<float> log( const Trigintaduonion<float> & );
template Trigintaduonion<float> log10( const Trigintaduonion<float> & );
template Trigintaduonion<float> pow( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> pow( const Trigintaduonion<float> &, float );
template Trigintaduonion<float> inverse( const Trigintaduonion<float> & );
template Trigintaduonion<float> sin( const Trigintaduonion<float> & );
template Trigintaduonion<float> cos( const Trigintaduonion<float> & );
template Trigintaduonion<float> tan( const Trigintaduonion<float> & );
template Trigintaduonion<float> sec( const Trigintaduonion<float> & );
template Trigintaduonion<float> csc( const Trigintaduonion<float> & );
template Trigintaduonion<float> cot( const Trigintaduonion<float> & );
template Trigintaduonion<float> sinh( const Trigintaduonion<float> & );
template Trigintaduonion<float> cosh( const Trigintaduonion<float> & );
template Trigintaduonion<float> tanh( const Trigintaduonion<float> & );
template Trigintaduonion<float> sech( const Trigintaduonion<float> & );
template Trigintaduonion<float> csch( const Trigintaduonion<float> & );
template Trigintaduonion<float> coth( const Trigintaduonion<float> & );
template Trigintaduonion<float> asin( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> acos( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> atan( const Trigintaduonion<float> & );
template Trigintaduonion<float> asec( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> acsc( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> acot( const Trigintaduonion<float> & );
template Trigintaduonion<float> asinh( const Trigintaduonion<float> & );
template Trigintaduonion<float> acosh( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> atanh( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> asech( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> acsch( const Trigintaduonion<float> & );
template Trigintaduonion<float> acoth( const Trigintaduonion<float> &, const Trigintaduonion<float> & );
template Trigintaduonion<float> bessel_J( int, const Trigintaduonion<float> & );
template Trigintaduonion<float> floor( const Trigintaduonion<float> & );
template Trigintaduonion<float> ceil( const Trigintaduonion<float> & );
template Trigintaduonion<float> horner( const Trigintaduonion<float> &, float *, unsigned int );
template Trigintaduonion<float> horner( const Trigintaduonion<float> &, float *, float *, unsigned int );

/************************************************************************
 * ******************************************************************** *
 * *                                                                  * *
 * *    Long Double-precision Floating-point Instance of Template     * *
 * *                                                                  * *
 * ******************************************************************** *
 ************************************************************************/

template class Trigintaduonion<long double>;
template std::ostream & operator << ( std::ostream &, const Trigintaduonion<long double> & );

template Trigintaduonion<long double> operator + ( long double, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> operator + ( long, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> operator - ( long double, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> operator - ( long, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> operator * ( long double, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> operator * ( long, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> operator / ( long double, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> operator / ( long, const Trigintaduonion<long double> & );

template bool operator == ( long double, const Trigintaduonion<long double> & );
template bool operator == ( long, const Trigintaduonion<long double> & );
template bool operator != ( long double, const Trigintaduonion<long double> & );
template bool operator != ( long, const Trigintaduonion<long double> & );

template <> const Trigintaduonion<long double> Trigintaduonion<long double>::ZERO = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>  Trigintaduonion<long double>::ONE = Trigintaduonion<long double>( 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>    Trigintaduonion<long double>::I = Trigintaduonion<long double>( 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>    Trigintaduonion<long double>::J = Trigintaduonion<long double>( 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>    Trigintaduonion<long double>::K = Trigintaduonion<long double>( 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::U1 = Trigintaduonion<long double>( 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::I1 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::J1 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::K1 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::U2 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::I2 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::J2 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::K2 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::U3 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::I3 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::J3 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::K3 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::U4 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::I4 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::J4 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::K4 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::U5 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::I5 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::J5 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::K5 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::U6 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::I6 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::J6 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::K6 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::U7 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::I7 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::J7 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 );
template <> const Trigintaduonion<long double>   Trigintaduonion<long double>::K7 = Trigintaduonion<long double>( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 );

template <> const Trigintaduonion<long double> Trigintaduonion<long double>::UNITS[32] = {
	Trigintaduonion<long double>::ONE,
	Trigintaduonion<long double>::I,
	Trigintaduonion<long double>::J,
	Trigintaduonion<long double>::K,
	Trigintaduonion<long double>::U1,
	Trigintaduonion<long double>::I1,
	Trigintaduonion<long double>::J1,
	Trigintaduonion<long double>::K1,
	Trigintaduonion<long double>::U2,
	Trigintaduonion<long double>::I2,
	Trigintaduonion<long double>::J2,
	Trigintaduonion<long double>::K2,
	Trigintaduonion<long double>::U3,
	Trigintaduonion<long double>::I3,
	Trigintaduonion<long double>::J3,
	Trigintaduonion<long double>::K3,
	Trigintaduonion<long double>::U4,
	Trigintaduonion<long double>::I4,
	Trigintaduonion<long double>::J4,
	Trigintaduonion<long double>::K4,
	Trigintaduonion<long double>::U5,
	Trigintaduonion<long double>::I5,
	Trigintaduonion<long double>::J5,
	Trigintaduonion<long double>::K5,
	Trigintaduonion<long double>::U6,
	Trigintaduonion<long double>::I6,
	Trigintaduonion<long double>::J6,
	Trigintaduonion<long double>::K6,
	Trigintaduonion<long double>::U7,
	Trigintaduonion<long double>::I7,
	Trigintaduonion<long double>::J7,
	Trigintaduonion<long double>::K7
};

template long double real( const Trigintaduonion<long double> & );
template long double imag_i( const Trigintaduonion<long double> & );
template long double imag_j( const Trigintaduonion<long double> & );
template long double imag_k( const Trigintaduonion<long double> & );
template long double imag_u1( const Trigintaduonion<long double> & );
template long double imag_i1( const Trigintaduonion<long double> & );
template long double imag_j1( const Trigintaduonion<long double> & );
template long double imag_k1( const Trigintaduonion<long double> & );
template long double imag_u2( const Trigintaduonion<long double> & );
template long double imag_i2( const Trigintaduonion<long double> & );
template long double imag_j2( const Trigintaduonion<long double> & );
template long double imag_k2( const Trigintaduonion<long double> & );
template long double imag_u3( const Trigintaduonion<long double> & );
template long double imag_i3( const Trigintaduonion<long double> & );
template long double imag_j3( const Trigintaduonion<long double> & );
template long double imag_k3( const Trigintaduonion<long double> & );
template long double imag_u4( const Trigintaduonion<long double> & );
template long double imag_i4( const Trigintaduonion<long double> & );
template long double imag_j4( const Trigintaduonion<long double> & );
template long double imag_k4( const Trigintaduonion<long double> & );
template long double imag_u5( const Trigintaduonion<long double> & );
template long double imag_i5( const Trigintaduonion<long double> & );
template long double imag_j5( const Trigintaduonion<long double> & );
template long double imag_k5( const Trigintaduonion<long double> & );
template long double imag_u6( const Trigintaduonion<long double> & );
template long double imag_i6( const Trigintaduonion<long double> & );
template long double imag_j6( const Trigintaduonion<long double> & );
template long double imag_k6( const Trigintaduonion<long double> & );
template long double imag_u7( const Trigintaduonion<long double> & );
template long double imag_i7( const Trigintaduonion<long double> & );
template long double imag_j7( const Trigintaduonion<long double> & );
template long double imag_k7( const Trigintaduonion<long double> & );
template long double csgn( const Trigintaduonion<long double> & );
template long double abs( const Trigintaduonion<long double> & );
template long double norm( const Trigintaduonion<long double> & );
template long double abs_imag( const Trigintaduonion<long double> & );
template long double norm_imag( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> imag( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> conj( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> signum( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> sqr( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> sqrt( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> rotate( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> exp( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> log( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> log10( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> pow( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> pow( const Trigintaduonion<long double> &, long double );
template Trigintaduonion<long double> inverse( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> sin( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> cos( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> tan( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> sec( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> csc( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> cot( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> sinh( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> cosh( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> tanh( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> sech( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> csch( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> coth( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> asin( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> acos( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> atan( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> asec( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> acsc( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> acot( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> asinh( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> acosh( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> atanh( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> asech( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> acsch( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> acoth( const Trigintaduonion<long double> &, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> bessel_J( int, const Trigintaduonion<long double> & );
template Trigintaduonion<long double> floor( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> ceil( const Trigintaduonion<long double> & );
template Trigintaduonion<long double> horner( const Trigintaduonion<long double> &, long double *, unsigned int );
template Trigintaduonion<long double> horner( const Trigintaduonion<long double> &, long double *, long double *, unsigned int );
