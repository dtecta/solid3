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

#include "Kinetic.h"

void Kinetic::proceed(MT_Scalar step) 
{
	m_xform.setOrigin(m_xform.getOrigin() + m_lin_vel * step);
	m_orn += m_ang_vel * m_orn * (step * 0.5f);
	m_orn.normalize();
	m_xform.setRotation(m_orn);
}

void Kinetic::setTransform(const MT_Transform& xform)
{
	m_xform = xform;
	m_xform.getBasis().getRotation(m_orn);
}

void Kinetic::reset(const MT_Transform& xform)
{
	setTransform(xform);
	m_lin_vel.setValue(0.0f, 0.0f, 0.0f);
	m_ang_vel.setValue(0.0f, 0.0f, 0.0f);
}

