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

#ifndef KINETIC_H
#define KINETIC_H

#include "MT_Transform.h"
#include "MT_Point3.h"
#include "MT_Quaternion.h"

class Kinetic {
public:
	Kinetic() 
      : m_orn(0.0f, 0.0f, 0.0f, 1.0f),
		m_lin_vel(0.0f, 0.0f, 0.0f),
		m_ang_vel(0.0f, 0.0f, 0.0f)
	{
		m_xform.setIdentity();
	}
	
	virtual void proceed(MT_Scalar step); 
	
	const MT_Point3&     getPosition() const { return m_xform.getOrigin(); }
	const MT_Quaternion& getOrientation() const { return m_orn; }
	
	const MT_Transform&  getTransform() const { return m_xform; }
	const MT_Vector3&    getLinearVelocity() const { return m_lin_vel; }
	const MT_Vector3&    getAngularVelocity() const { return m_ang_vel; }
	
	virtual void setTransform(const MT_Transform& xform);		
	virtual void reset(const MT_Transform& xform);

	void setLinearVelocity(const MT_Vector3& lin_vel) { m_lin_vel = lin_vel; }
	void setAngularVelocity(const MT_Vector3& ang_vel) { m_ang_vel = ang_vel; }

	MT_Vector3 getVelocity(const MT_Vector3& rel_pos) const
	{
		return m_lin_vel + m_ang_vel.cross(rel_pos);
	}

	void translate(const MT_Vector3& v) 
	{
		m_xform.getOrigin() += v; 
	}

protected:	
	MT_Transform  m_xform;
	MT_Quaternion m_orn;
	MT_Vector3    m_lin_vel;
	MT_Vector3    m_ang_vel;
};

#endif
