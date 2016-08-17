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

#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "Kinetic.h"

class Dynamic : public Kinetic {
public:
	Dynamic() 
      : m_total_force(0.0f, 0.0f, 0.0f),
		m_total_torque(0.0f, 0.0f, 0.0f)
	{
		updateInertiaTensor();
	}

	void setMassProps(MT_Scalar mass, const MT_Vector3& inertia);
	
	MT_Scalar getInvMass() const { return m_inv_mass; }
	const MT_Matrix3x3& getInvInertiaTensor() const { return m_inv_inertia_tensor; }

	virtual void proceed(MT_Scalar step); 
	virtual void setTransform(const MT_Transform& xform);
	virtual void reset(const MT_Transform& xform);

	void applyCentralForce(const MT_Vector3& force)
	{
		m_total_force += force;
	}
    
	void applyTorque(const MT_Vector3& torque)
	{
		m_total_torque += torque;
	}
	
	void applyForce(const MT_Vector3& force, const MT_Vector3& rel_pos) 
	{
		applyCentralForce(force);
		applyTorque(rel_pos.cross(force));
	}
	
	void applyCentralImpulse(const MT_Vector3& impulse)
	{
		m_lin_vel += impulse * m_inv_mass;
	}
	
  	void applyTorqueImpulse(const MT_Vector3& torque)
	{
		m_ang_vel += m_inv_inertia_tensor * torque;
	}
	
	void applyImpulse(const MT_Vector3& impulse, const MT_Vector3& rel_pos) 
	{
		applyCentralImpulse(impulse);
		applyTorqueImpulse(rel_pos.cross(impulse));
	}
	
	void clearForces() 
	{
		m_total_force.setValue(0.0f, 0.0f, 0.0f);
		m_total_torque.setValue(0.0f, 0.0f, 0.0f);
	}
	
protected:	
	void updateInertiaTensor();    
	
	MT_Matrix3x3  m_inv_inertia_tensor;
	MT_Scalar     m_inv_mass;
	MT_Vector3    m_inv_inertia;
	MT_Vector3    m_total_force;
	MT_Vector3    m_total_torque;
};

#endif
  
   
    
  
 
    
   
