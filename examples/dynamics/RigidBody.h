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

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <vector>

#include "Dynamic.h"

class RigidBody : public Dynamic {
public:
	RigidBody() 
	  : m_gravity(0.0f, 0.0f, 0.0f)
	{}
	
	virtual void proceed(MT_Scalar step);
	virtual void reset(const MT_Transform& xform);
	
 
	
	void backup();  
	
	void applyForces(MT_Scalar step);
	
	void setGravity(const MT_Vector3& acceleration);  
	void setDamping(MT_Scalar lin_damping, MT_Scalar ang_damping);
	
	void resolveCollision(const MT_Vector3& pos, MT_Scalar depth, const MT_Vector3& normal, MT_Scalar restitution, MT_Scalar friction); 
	
private:
	MT_Transform     m_prev_xform;
	MT_Quaternion    m_prev_orn;
	MT_Vector3       m_gravity;	
	MT_Scalar        m_lin_damping;
	MT_Scalar        m_ang_damping;
};

void resolveCollision(RigidBody& body1, const MT_Vector3& pos1,
                      RigidBody& body2, const MT_Vector3& pos2,
                      MT_Scalar depth, const MT_Vector3& normal, MT_Scalar restitution);
#endif
