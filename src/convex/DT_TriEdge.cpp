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

#include "DT_TriEdge.h"

TriangleStore g_triangleStore;

bool link(const Edge& edge0, const Edge& edge1) 
{
	bool ok = edge0.getSource() == edge1.getTarget() && edge0.getTarget() == edge1.getSource();
    
    //assert(ok);
    
    if (ok)
    {
	edge0.triangle()->m_adjEdges[edge0.index()] = edge1;
    edge1.triangle()->m_adjEdges[edge1.index()] = edge0;
}

    return ok;
}

void half_link(const Edge& edge0, const Edge& edge1) 
{
	assert(edge0.getSource() == edge1.getTarget() && edge0.getTarget() == edge1.getSource());
    
	edge0.triangle()->m_adjEdges[edge0.index()] = edge1;
}


bool Triangle::computeClosest(const MT_Vector3 *verts)
{
    const MT_Vector3& p0 = verts[m_indices[0]]; 

    MT_Vector3 v1 = verts[m_indices[1]] - p0;
    MT_Vector3 v2 = verts[m_indices[2]] - p0;
    MT_Scalar v1dv1 = v1.length2();
    MT_Scalar v1dv2 = dot(v1, v2);
    MT_Scalar v2dv2 = v2.length2();
    MT_Scalar p0dv1 = dot(p0, v1); 
    MT_Scalar p0dv2 = dot(p0, v2);
    
    m_det = v1dv1 * v2dv2 - v1dv2 * v1dv2; // non-negative
    m_lambda1 = p0dv2 * v1dv2 - p0dv1 * v2dv2;
    m_lambda2 = p0dv1 * v1dv2 - p0dv2 * v1dv1; 
    
    if (m_det > MT_Scalar(0.0)) 
	{
		m_closest = p0 + (m_lambda1 * v1 + m_lambda2 * v2) / m_det;
		m_dist2 = m_closest.length2();

		return true;
    }

    return false;
} 

bool Edge::silhouette(const MT_Vector3 *verts, Index_t index, TriangleStore& triangleStore) const 
{
    if (!m_triangle->isObsolete()) 
    {
		if (!m_triangle->isVisibleFrom(verts, index)) 
        {
			Triangle *triangle = triangleStore.newTriangle(verts, index, getTarget(), getSource());

			if (triangle)
			{
				half_link(Edge(triangle, 1), *this);
				return true;
			}

			return false;
		}	
        else 
        {
            m_triangle->setObsolete(true); // Triangle is visible 
         
			int backup = triangleStore.getFree();

            if (!m_triangle->getAdjEdge(circ_next(m_index)).silhouette(verts, index, triangleStore))
			{
				m_triangle->setObsolete(false);
				
				Triangle *triangle = triangleStore.newTriangle(verts, index, getTarget(), getSource());

				if (triangle)
				{
					half_link(Edge(triangle, 1), *this);
					return true;
				}

				return false;
			}
			else if (!m_triangle->getAdjEdge(circ_prev(m_index)).silhouette(verts, index, triangleStore))
			{
				m_triangle->setObsolete(false);

				triangleStore.setFree(backup);
								
				Triangle *triangle = triangleStore.newTriangle(verts, index, getTarget(), getSource());

				if (triangle)
				{
					half_link(Edge(triangle, 1), *this);
					return true;
				}

				return false;
			}
        }
    }

	return true;
}

bool Triangle::silhouette(const MT_Vector3 *verts, Index_t index, TriangleStore& triangleStore) 
{
	//assert(isVisibleFrom(verts, index));

	int first = triangleStore.getFree();

	setObsolete(true);
	
	bool result = m_adjEdges[0].silhouette(verts, index, triangleStore) &&
	              m_adjEdges[1].silhouette(verts, index, triangleStore) &&
	              m_adjEdges[2].silhouette(verts, index, triangleStore);

	if (result)
	{
		int i, j;
		for (i = first, j = triangleStore.getFree()-1; i != triangleStore.getFree(); j = i++)
		{
			Triangle *triangle = &triangleStore[i];
			half_link(triangle->getAdjEdge(1), Edge(triangle, 1));
            if (!link(Edge(triangle, 0), Edge(&triangleStore[j], 2)))
            {
                return false;
            }
		}
	}

	return result;
}

