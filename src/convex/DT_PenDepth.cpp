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

#include "DT_PenDepth.h"
#include "DT_TriEdge.h"
#include "DT_GJK.h"
#include "DT_Convex.h"
#include "MT_Quaternion.h"

//#define DEBUG


const int       MaxSupportPoints = 100;
const int       MaxFacets         = 200;

static MT_Point3  pBuf[MaxSupportPoints];
static MT_Point3  qBuf[MaxSupportPoints];
static MT_Vector3 yBuf[MaxSupportPoints];


static Triangle *triangleHeap[MaxFacets];
static int  num_triangles;

class TriangleComp
{
public:
    
    bool operator()(const Triangle *face1, const Triangle *face2) 
    { 
        return face1->getDist2() > face2->getDist2();
    }
} triangleComp;

inline void addCandidate(Triangle *triangle, MT_Scalar upper2) 
{
    if (triangle->isClosestInternal() && triangle->getDist2() <= upper2)
    {
        triangleHeap[num_triangles++] = triangle;
        std::push_heap(&triangleHeap[0], &triangleHeap[num_triangles], triangleComp);
#ifdef DEBUG
        std::cout << " accepted" << std::endl;
#endif
    }
    else 
    {
#ifdef DEBUG
        std::cout << " rejected, ";
        if (!triangle->isClosestInternal()) 
            {
                std::cout << "closest point not internal";
            }
        else 
        {
            std::cout << "triangle is further than upper bound";
        }
        std::cout << std::endl;
#endif
    }
}		

inline int originInTetrahedron(const MT_Vector3& p1, const MT_Vector3& p2, 
                               const MT_Vector3& p3, const MT_Vector3& p4)
{
    MT_Vector3 normal1 = cross(p2 - p1, p3 - p1);
	if ((dot(normal1, p1) > MT_Scalar(0.0)) == (dot(normal1, p4) > MT_Scalar(0.0)))
    {
        return 4;
    }
    
    MT_Vector3 normal2 = cross(p4 - p2, p3 - p2);
    if ((dot(normal2, p2) > MT_Scalar(0.0)) == (dot(normal2, p1) > MT_Scalar(0.0)))
    {
        return 1;
    }
    
    MT_Vector3 normal3 = cross(p4 - p3, p1 - p3);
    if ((dot(normal3, p3) > MT_Scalar(0.0)) == (dot(normal3, p2) > MT_Scalar(0.0)))
    {
        return 2;
    }
    
    MT_Vector3 normal4 = cross(p2 - p4, p1 - p4);
    if ((dot(normal4, p4) > MT_Scalar(0.0)) == (dot(normal4, p3) > MT_Scalar(0.0)))
    {
        return 3; 
    }
    
    return 0;
}

bool penDepth(const DT_GJK& gjk, const DT_Convex& a, const DT_Convex& b,
              MT_Vector3& v, MT_Point3& pa, MT_Point3& pb)
{
	
	int num_verts = gjk.getSimplex(pBuf, qBuf, yBuf);
    MT_Scalar tolerance = DT_Accuracy::tol_error * gjk.maxVertex();
    
    num_triangles = 0;
    
    g_triangleStore.clear();
	
    switch (num_verts) 
    {
    case 1:
        // Touching contact. Yes, we have a collision,
        // but no penetration.
        return false;
    case 2:	
    {
        // We have a line segment inside the Minkowski sum containing the
        // origin. Blow it up by adding three additional support points.
	    
        MT_Vector3 dir  = yBuf[1] - yBuf[0];
        
        dir /= length(dir);
        int        axis = dir.furthestAxis();
	    
        static const MT_Scalar sin_60 = sqrt(MT_Scalar(3.0)) * MT_Scalar(0.5);
	    
        MT_Quaternion rot(dir[0] * sin_60, dir[1] * sin_60, dir[2] * sin_60, MT_Scalar(0.5));
        MT_Matrix3x3 rot_mat(rot);
	    
        MT_Vector3 aux1 = cross(dir, MT_Vector3(axis == 0, axis == 1, axis == 2));
        MT_Vector3 aux2 = rot_mat * aux1;
        MT_Vector3 aux3 = rot_mat * aux2;
        
        pBuf[2] = a.support(aux1);
        qBuf[2] = b.support(-aux1);
        yBuf[2] = pBuf[2] - qBuf[2];
	    
        pBuf[3] = a.support(aux2);
        qBuf[3] = b.support(-aux2);
        yBuf[3] = pBuf[3] - qBuf[3];
	    
        pBuf[4] = a.support(aux3);
        qBuf[4] = b.support(-aux3);
        yBuf[4] = pBuf[4] - qBuf[4];
	    
        if (originInTetrahedron(yBuf[0], yBuf[2], yBuf[3], yBuf[4]) == 0) 
        {
            pBuf[1] = pBuf[4];
            qBuf[1] = qBuf[4];
            yBuf[1] = yBuf[4];
        }
        else if (originInTetrahedron(yBuf[1], yBuf[2], yBuf[3], yBuf[4]) == 0) 
        {
            pBuf[0] = pBuf[4];
            qBuf[0] = qBuf[4];
            yBuf[0] = yBuf[4];
        } 
        else 
        {
            // Origin not in initial polytope
            return false;
        }
	    
        num_verts = 4;
    }
    // Fall through allowed!!
    case 4: 
    {
        int bad_vertex = originInTetrahedron(yBuf[0], yBuf[1], yBuf[2], yBuf[3]);
        
        if (bad_vertex == 0)
        {
            Triangle *f0 = g_triangleStore.newTriangle(yBuf, 0, 1, 2);
            Triangle *f1 = g_triangleStore.newTriangle(yBuf, 0, 3, 1);
            Triangle *f2 = g_triangleStore.newTriangle(yBuf, 0, 2, 3);
            Triangle *f3 = g_triangleStore.newTriangle(yBuf, 1, 3, 2);
            
            if (!(f0 && f0->getDist2() > MT_Scalar(0.0) &&
                  f1 && f1->getDist2() > MT_Scalar(0.0) &&
                  f2 && f2->getDist2() > MT_Scalar(0.0) &&
                  f3 && f3->getDist2() > MT_Scalar(0.0)))
			{
				return false;
			}
            
            link(Edge(f0, 0), Edge(f1, 2));
            link(Edge(f0, 1), Edge(f3, 2));
            link(Edge(f0, 2), Edge(f2, 0));
            link(Edge(f1, 0), Edge(f2, 2));
            link(Edge(f1, 1), Edge(f3, 0));
            link(Edge(f2, 1), Edge(f3, 1));
            
            addCandidate(f0, MT_INFINITY);
            addCandidate(f1, MT_INFINITY);
            addCandidate(f2, MT_INFINITY);
            addCandidate(f3, MT_INFINITY);
            break;
        }
        
        if (bad_vertex < 4)
        {
            pBuf[bad_vertex - 1] = pBuf[4];
            qBuf[bad_vertex - 1] = qBuf[4];
            yBuf[bad_vertex - 1] = yBuf[4];
            
        }
        
        num_verts = 3;
        
    }
    // Fall through allowed!! 
    case 3: 
    {
        // We have a triangle inside the Minkowski sum containing
        // the origin. First blow it up.
	    
        MT_Vector3 v1     = yBuf[1] - yBuf[0];
        MT_Vector3 v2     = yBuf[2] - yBuf[0];
        MT_Vector3 vv     = cross(v1, v2);
	    
        pBuf[3] = a.support(vv);
        qBuf[3] = b.support(-vv);
        yBuf[3] = pBuf[3] - qBuf[3];
        pBuf[4] = a.support(-vv);
        qBuf[4] = b.support(vv);
        yBuf[4] = pBuf[4] - qBuf[4];
	    
        Triangle* f0 = g_triangleStore.newTriangle(yBuf, 0, 1, 3);
        Triangle* f1 = g_triangleStore.newTriangle(yBuf, 1, 2, 3);
        Triangle* f2 = g_triangleStore.newTriangle(yBuf, 2, 0, 3); 
        Triangle* f3 = g_triangleStore.newTriangle(yBuf, 0, 2, 4);
        Triangle* f4 = g_triangleStore.newTriangle(yBuf, 2, 1, 4);
        Triangle* f5 = g_triangleStore.newTriangle(yBuf, 1, 0, 4);
        
        if (!(f0 && f0->getDist2() > MT_Scalar(0.0) &&
              f1 && f1->getDist2() > MT_Scalar(0.0) &&
              f2 && f2->getDist2() > MT_Scalar(0.0) &&
              f3 && f3->getDist2() > MT_Scalar(0.0) &&
              f4 && f4->getDist2() > MT_Scalar(0.0) &&
              f5 && f5->getDist2() > MT_Scalar(0.0)))
        {
            return false;
        }
        
        link(Edge(f0, 1), Edge(f1, 2));
        link(Edge(f1, 1), Edge(f2, 2));
        link(Edge(f2, 1), Edge(f0, 2));
        
        link(Edge(f0, 0), Edge(f5, 0));
        link(Edge(f1, 0), Edge(f4, 0));
        link(Edge(f2, 0), Edge(f3, 0));
        
        link(Edge(f3, 1), Edge(f4, 2));
        link(Edge(f4, 1), Edge(f5, 2));
        link(Edge(f5, 1), Edge(f3, 2));
	    
        addCandidate(f0, MT_INFINITY);
        addCandidate(f1, MT_INFINITY);
        addCandidate(f2, MT_INFINITY);
        addCandidate(f3, MT_INFINITY);  
        addCandidate(f4, MT_INFINITY);
        addCandidate(f5, MT_INFINITY);
	    
        num_verts = 5;
    }
    break;
    }
    
    // We have a polytope inside the Minkowski sum containing
    // the origin.
    
    if (num_triangles == 0)
    {
        return false;
    }
    
    // at least one triangle on the heap.	
    
    Triangle *triangle = 0;
    
    MT_Scalar upper_bound2 = MT_INFINITY; 	
    
    do 
    {
        triangle = triangleHeap[0];
        std::pop_heap(&triangleHeap[0], &triangleHeap[num_triangles], triangleComp);
        --num_triangles;
		
        if (!triangle->isObsolete()) 
        {
            if (num_verts == MaxSupportPoints)
            {
#ifdef DEBUG
                std::cout << "Ouch, no convergence!!!" << std::endl;
#endif 
                assert(false);	
                break;
            }
			
            pBuf[num_verts] = a.support( triangle->getClosest());
            qBuf[num_verts] = b.support(-triangle->getClosest());
            yBuf[num_verts] = pBuf[num_verts] - qBuf[num_verts];
			
            int index = num_verts++;
            MT_Scalar far_dist = dot(yBuf[index], triangle->getClosest());
			
            // Make sure the support mapping is OK.
            assert(far_dist > MT_Scalar(0.0));
            MT_Scalar far_dist2 = far_dist * far_dist / triangle->getDist2();
            GEN_set_min(upper_bound2, far_dist2);
			
            MT_Scalar error = far_dist - triangle->getDist2();
            if (error <= GEN_max(DT_Accuracy::rel_error2 * far_dist, tolerance)
#if 1
                || yBuf[index] == yBuf[(*triangle)[0]] 
                || yBuf[index] == yBuf[(*triangle)[1]]
                || yBuf[index] == yBuf[(*triangle)[2]]
#endif
                ) 
            {
                break;
            }
            
            
            // Compute the silhouette cast by the new vertex
            // Note that the new vertex is on the positive side
            // of the current triangle, so the current triangle will
            // not be in the convex hull. Start local search
            // from this triangle.
			
            int i = g_triangleStore.getFree();
            
            if (!triangle->silhouette(yBuf, index, g_triangleStore))
            {
                break;
            }
			
            while (i != g_triangleStore.getFree())
            {
                Triangle *newTriangle = &g_triangleStore[i];
                //assert(triangle->getDist2() <= newTriangle->getDist2());
                
                addCandidate(newTriangle, upper_bound2);
                
                ++i;
            }
        }
    }
    while (num_triangles > 0 && triangleHeap[0]->getDist2() <= upper_bound2);
	
#ifdef DEBUG    
    std::cout << "#triangles left = " << num_triangles << std::endl;
#endif
    
    v = triangle->getClosest();
    pa = triangle->getClosestPoint(pBuf);    
    pb = triangle->getClosestPoint(qBuf);    
    return true;
}

