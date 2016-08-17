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

#include "Dynamic.h"

void Dynamic::setMassProps(MT_Scalar mass, const MT_Vector3& inertia)
{
	m_inv_mass = mass != 0.0f ? 1.0f / mass : 0.0f;
	m_inv_inertia.setValue(inertia[0] != 0.0f ? 1.0f / inertia[0]: 0.0f,
						   inertia[1] != 0.0f ? 1.0f / inertia[1]: 0.0f,
						   inertia[2] != 0.0f ? 1.0f / inertia[2]: 0.0f);

}
	
void Dynamic::updateInertiaTensor() 
{
	m_inv_inertia_tensor = m_xform.getBasis().scaled(m_inv_inertia) * m_xform.getBasis().transpose();
}

void Dynamic::proceed(MT_Scalar step) 
{
	Kinetic::proceed(step);
	updateInertiaTensor();
	
	m_lin_vel += m_total_force * (m_inv_mass * step);
	m_ang_vel += m_inv_inertia_tensor * m_total_torque * step;
	clearForces();
}

void Dynamic::setTransform(const MT_Transform& xform)
{
	Kinetic::setTransform(xform);
	updateInertiaTensor();
}

void Dynamic::reset(const MT_Transform& xform)
{
	Kinetic::reset(xform);
	updateInertiaTensor();
	clearForces();
}
