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

#include <stdio.h>

//#define USE_QUADS     //Use quad mesh instead of a triangle mesh
//#define USE_HULL      //Use the convex hulls of the rings 

#include <SOLID.h>

#include "MT_Point3.h"
#include "MT_Vector3.h"
#include "MT_Quaternion.h"

const MT_Scalar SPACE_SIZE = 20.0;

const int NUM_ITER   = 10000;
const int NUM_OBJECTS = 2;


typedef struct MyObject {
    int id;
	MT_Point3 pos;
    DT_ObjectHandle objectHandle;
} MyObject; 


/* ARGSUSED */
DT_Bool collide1(void * client_data, void *obj1, void *obj2,
			  const DT_CollData *coll_data)
{
	return DT_CONTINUE;
}

/* ARGSUSED */
DT_Bool collide2(void * client_data, void *obj1, void *obj2,
			  const DT_CollData *coll_data)
{
    FILE *stream = (FILE *)client_data;

    fprintf(stream, "Aha!, Object %d interferes with object %d\n", 
            (*(MyObject *)obj1).id, (*(MyObject *)obj2).id);

	return DT_CONTINUE;
}

#ifdef STATISTICS
extern int num_box_tests;
#endif

int main() 
{
	MyObject objects[NUM_OBJECTS];

	int i;
	for (i = 0; i != NUM_OBJECTS; ++i) 
	{
        objects[i].id = i;
		objects[i].pos.setValue(MT_random() * SPACE_SIZE, MT_random() * SPACE_SIZE, MT_random() * SPACE_SIZE);
	}

#ifdef USE_HULL
    DT_ShapeHandle shape = DT_NewPolytope(0);
#else
    DT_ShapeHandle shape = DT_NewComplexShape(0);
#endif

    MT_Scalar a = 10; 
    MT_Scalar b = 2; 

    fprintf(stderr, "Loading a torus with a major radius of %d and a minor radius of %d,\n", (int)a, (int)b); 

    const int n1 = 50;
    const int n2 = 50;

#ifdef USE_QUADS
    fprintf(stderr, "composed of %d quads...",n1 * n2); fflush(stderr); 
#else
    fprintf(stderr, "composed of %d triangles...", 2 * n1 * n2); fflush(stderr); 
#endif
    int uc;
    for (uc = 0; uc < n1; uc++) 
	{
        int vc;
        for (vc = 0; vc < n2; vc++)
		{
            MT_Scalar u1 = (MT_2_PI * uc) / n1; 
            MT_Scalar u2 = (MT_2_PI * (uc+1)) / n1; 
            MT_Scalar v1 = (MT_2_PI * vc) / n2; 
            MT_Scalar v2 = (MT_2_PI * (vc+1)) / n2; 
            
            MT_Scalar p1[3], p2[3], p3[3], p4[3];
            
            p1[0] = (a - b * MT_cos(v1)) * MT_cos(u1);
            p2[0] = (a - b * MT_cos(v1)) * MT_cos(u2);
            p3[0] = (a - b * MT_cos(v2)) * MT_cos(u1);
            p4[0] = (a - b * MT_cos(v2)) * MT_cos(u2);
            p1[1] = (a - b * MT_cos(v1)) * MT_sin(u1);
            p2[1] = (a - b * MT_cos(v1)) * MT_sin(u2);
            p3[1] = (a - b * MT_cos(v2)) * MT_sin(u1);
            p4[1] = (a - b * MT_cos(v2)) * MT_sin(u2);
            p1[2] = b * MT_sin(v1);
            p2[2] = b * MT_sin(v1);
            p3[2] = b * MT_sin(v2);
            p4[2] = b * MT_sin(v2);
            
#ifdef USE_QUADS

			DT_Begin();   
            DT_Vertex(p1);
            DT_Vertex(p2);
            DT_Vertex(p4);
            DT_Vertex(p3);
            DT_End();

#else      
            DT_Begin();
            DT_Vertex(p1);
            DT_Vertex(p2);
            DT_Vertex(p3);
            DT_End();

            DT_Begin();
            DT_Vertex(p4);
            DT_Vertex(p1);
            DT_Vertex(p2);
            DT_End();
#endif  
        }
    }
    fprintf(stderr, "done.\n");

#ifdef USE_HULL
    fprintf(stderr, "Building convex hull..."); fflush(stderr);
    DT_EndPolytope();
#else
	fprintf(stderr, "Building hierarchy..."); fflush(stderr);
    DT_EndComplexShape();
#endif

	fprintf(stderr, "done.\n");

    DT_SceneHandle scene = DT_CreateScene();
	
	DT_RespTableHandle respTable = DT_CreateRespTable();
	DT_ResponseClass responseClass = DT_GenResponseClass(respTable);
	
	for (i = 0; i != NUM_OBJECTS; ++i) 
	{
		objects[i].objectHandle = DT_CreateObject(&objects[i], shape);
		DT_AddObject(scene, objects[i].objectHandle); 
		DT_SetResponseClass(respTable, objects[i].objectHandle, responseClass);
	}
   
    DT_AddDefaultResponse(respTable, &collide1, DT_SIMPLE_RESPONSE, stdout);

	int col_count = 0;
    
    printf("Running %d tests on %d objects at random placements\n", NUM_ITER, NUM_OBJECTS);
    printf("in a space of size %.0f...\n", SPACE_SIZE);  
    int k;
	for (k = 0; k != NUM_ITER; ++k) 
	{
		for (i = 0; i != NUM_OBJECTS; ++i) 
		{ 
			objects[i].pos.setValue(MT_random() * SPACE_SIZE, 
									MT_random() * SPACE_SIZE, 
									MT_random() * SPACE_SIZE);

			DT_SetPosition(objects[i].objectHandle, objects[i].pos);
			DT_SetOrientation(objects[i].objectHandle, MT_Quaternion::random());
		}
       

        col_count += DT_Test(scene, respTable);
		//fprintf(stderr, "."); fflush(stderr);
    }
    printf("done\n");
    
    std::cout << "Number of collisions: " << col_count << std::endl;
#ifdef STATISTICS
    std::cout << "Number of sep_axis calls: " << num_box_tests << std::endl;
#endif
	for (i = 0; i != NUM_OBJECTS; ++i)
	{
		DT_RemoveObject(scene, objects[i].objectHandle);
		DT_DestroyObject(objects[i].objectHandle);
	}
	    
    DT_DestroyRespTable(respTable);
	DT_DestroyScene(scene);
    DT_DeleteShape(shape);
    
    return 0;
}







