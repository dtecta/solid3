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

#include "RigidBody.h"

static MT_Scalar ContactThreshold = -10.0f;  

void RigidBody::setGravity(const MT_Vector3& acceleration) 
{
	if (m_inv_mass != 0.0f)
	{
		m_gravity = acceleration * (1.0f / m_inv_mass);
	}
}

void RigidBody::setDamping(MT_Scalar lin_damping, MT_Scalar ang_damping)
{
	m_lin_damping = GEN_clamped(1.0f - lin_damping, 0.0f, 1.0f);
	m_ang_damping = GEN_clamped(1.0f - ang_damping, 0.0f, 1.0f);
}

void RigidBody::reset(const MT_Transform& xform)
{
	Dynamic::reset(xform);
}


void RigidBody::applyForces(MT_Scalar step)
{
	applyCentralForce(m_gravity);	
	m_lin_vel *= powf(m_lin_damping, step);
	m_ang_vel *= powf(m_ang_damping, step);
}


void RigidBody::proceed(MT_Scalar step)
{
	m_prev_xform = m_xform;
	m_prev_orn = m_orn;
	Dynamic::proceed(step);
}

void RigidBody::backup()
{
	m_xform = m_prev_xform;
	m_orn = m_prev_orn;   
	updateInertiaTensor();
}

inline MT_Scalar restitutionCurve(MT_Scalar rel_vel, MT_Scalar restitution)
{
	return restitution * GEN_min(1.0f, rel_vel / ContactThreshold);
}

void RigidBody::resolveCollision(const MT_Vector3& pos, 
								 MT_Scalar depth, const MT_Vector3& normal, 
								 MT_Scalar restitution, MT_Scalar friction)
{
	MT_Vector3 rel_pos = pos - getPosition(); 
	MT_Vector3 vel = getVelocity(rel_pos);
	
	MT_Scalar rel_vel = normal.dot(vel);
	if (rel_vel < -MT_EPSILON) 
	{
		MT_Vector3 temp = getInvInertiaTensor() * rel_pos.cross(normal);  
		MT_Scalar rest = restitutionCurve(rel_vel, restitution);
		MT_Scalar impulse = -(1.0f + rest) * rel_vel / 
			(getInvMass() + normal.dot(temp.cross(rel_pos)));
		
		applyImpulse(normal * impulse, rel_pos);
		
		MT_Vector3 lat_vel = vel - normal * rel_vel;
		MT_Scalar lat_rel_vel = lat_vel.length();
		if (lat_rel_vel > 0.0f)
		{
			lat_vel /= lat_rel_vel;
			temp = getInvInertiaTensor() * rel_pos.cross(lat_vel); 
			MT_Scalar friction_impulse = -lat_rel_vel / 
				(getInvMass() + lat_vel.dot(temp.cross(rel_pos)));
			
			GEN_set_min(friction_impulse, impulse * friction);
			applyImpulse(lat_vel * friction_impulse, rel_pos);
		}
	}
}


void resolveCollision(RigidBody& body1, const MT_Vector3& pos1,
                      RigidBody& body2, const MT_Vector3& pos2,
                      MT_Scalar depth, const MT_Vector3& normal, MT_Scalar restitution)
{
	MT_Vector3 rel_pos1 = pos1 - body1.getPosition(); 
	MT_Vector3 rel_pos2 = pos2 - body2.getPosition();
	
	MT_Scalar rel_vel = normal.dot(body1.getVelocity(rel_pos1) - body2.getVelocity(rel_pos2));
	if (rel_vel < -MT_EPSILON) 
	{
		MT_Vector3 temp1 = body1.getInvInertiaTensor() * rel_pos1.cross(normal); 
		MT_Vector3 temp2 = body2.getInvInertiaTensor() * rel_pos2.cross(normal); 
      MT_Scalar rest = restitutionCurve(rel_vel, restitution);
		MT_Scalar impulse = -(1.0f + rest) * rel_vel / 
			(body1.getInvMass() + body2.getInvMass() + normal.dot(temp1.cross(rel_pos1) + temp2.cross(rel_pos2)));
		
		MT_Vector3 impulse_vector = normal * impulse;
		body1.applyImpulse(impulse_vector, rel_pos1);
		body2.applyImpulse(-impulse_vector, rel_pos2);
	}
}
