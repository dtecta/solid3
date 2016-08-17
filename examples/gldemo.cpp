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

#include <GL/glut.h>
#include <SOLID.h>

#define DISPLAY           // Interactively draw new placements 
#define LOG               // Show logging in console window 

#define DRAW_BBOX         // Draw the world-axis aligned bounding boxes 
//#define DRAW_COORD      // Draw local coordinate systems

//#define USE_BOXES       // Use boxes as objects  
//#define USE_HULL        // Box shape is convex hull of its vertices
//#define USE_COMPLEX     // Box is a triangle mesh and has no interior
//#define USE_QUADS       // Idem, but now a quad mesh
                          // NB: No ray cast for quads!!

//#define USE_TRIANGLES   // Use triangles as objects

//#define USE_SPHERES     // Use spheres as objects
                          // Penetration depth computations are 
                          // terible, since GJK seldom returns
                          // more than two vertices for a pair of
                          // spheres. EPA `hates' symmetry. 

//#define USE_MARGIN      // The easy way out is to define the
                          // spheres as points and set their
                          // margin to the radius
                          // NB: No ray cast for point + margin!!

//#define SCALING_ON      // Objects are nonuniformly scaled
//#define COMMON_POINT    // Common point computation instead of penetration depth
//#define STATISTICS      // Show statistics. Requires a libsolid built for this purpose

#include "MT_Point3.h"
#include "MT_Vector3.h"
#include "MT_Quaternion.h"
#include "MT_Matrix3x3.h"
#include "MT_Transform.h"

inline MT_Scalar irnd() { return 2.0f * MT_random() - 1.0f; }  

const MT_Scalar SPACE_SIZE = 0.1f;  // Size of the space 
const MT_Scalar SIZE_RATIO = 1.0e-0;  // Difference in size 
const MT_Scalar STEPSIZE   = 5.0f;  // Displacement of camera in degrees

#ifdef SCALING_ON

const MT_Scalar SCALE_BOTTOM = 0.5f; // Lowest scale factor
const MT_Scalar SCALE_RANGE  = 2.0f; // Range for scaling 
									 //	[SCALE_BOTTOM, SCALE_BOTTOM + SCALE_RANGE ]
#endif

class Shape {
public:
	Shape(DT_ShapeHandle shape = 0) : m_shape(shape) {}
    virtual ~Shape() {}	

	void setShape(DT_ShapeHandle shape) { m_shape = shape; }

	DT_ShapeHandle getShape() const { return m_shape; }

	virtual void paint() const = 0;

protected:
	DT_ShapeHandle m_shape;
};


class GLSphere : public Shape {
public:
    GLSphere(MT_Scalar radius) 
#if defined(USE_MARGIN)
      :	Shape(DT_NewPoint(MT_Vector3(0.0f, 0.0f, 0.0f))), 
#else
      :	Shape(DT_NewSphere(1.0f)), 
#endif
		m_radius(radius) 
	{}

	virtual void paint() const 
	{ 
		glutSolidSphere(m_radius, 20, 20); 
	}

private:
	MT_Scalar      m_radius;
};

class GLTriangle : public Shape {
public:
    GLTriangle(const DT_Vector3 points[])
	{
		m_points[0].setValue(points[0]);
		m_points[1].setValue(points[1]);
		m_points[2].setValue(points[2]);			

		m_base = DT_NewVertexBase(m_points, 0);
#if defined(USE_HULL)

		setShape(DT_NewPolytope(m_base));
		
		DT_VertexRange(0, 3);

		DT_EndPolytope();

#else
		setShape(DT_NewComplexShape(m_base));
		
		DT_VertexRange(0, 3);
		
		DT_EndComplexShape();
#endif
	}

	virtual void paint() const 
	{ 
		glBegin(GL_TRIANGLES);
    
		glVertex3fv(m_points[0]);
		glVertex3fv(m_points[1]);
		glVertex3fv(m_points[2]);
		
	    glEnd();	
	}

private:
	MT_Point3            m_points[3];
	DT_VertexBaseHandle  m_base;
};

class GLBox : public Shape {
public:

	GLBox(MT_Scalar x, MT_Scalar y, MT_Scalar z) 
      :	m_extent(x, y, z)
	{
		MT_Vector3 e(x * 0.5f, y * 0.5f, z * 0.5f);
		
		unsigned int bits;
		for (bits = 0x0; bits != 0x08; ++bits)
		{
			m_vertices[bits] = MT_Point3((bits & 0x1) ? e[0] : -e[0], 
										 (bits & 0x2) ? e[1] : -e[1],
										 (bits & 0x4) ? e[2] : -e[2]);
		}

		m_base = DT_NewVertexBase(m_vertices, 0);

#if defined(USE_HULL)
		setShape(DT_NewPolytope(m_base));
		
		DT_VertexRange(0, 8);

		DT_EndPolytope();

#elif defined(USE_COMPLEX)
		setShape(DT_NewComplexShape(m_base));

#ifdef USE_QUADS

		DT_Begin();   
		DT_VertexIndex(0);
		DT_VertexIndex(2);
		DT_VertexIndex(6);
		DT_VertexIndex(4);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(1);
		DT_VertexIndex(3);
		DT_VertexIndex(7);
		DT_VertexIndex(5);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(0);
		DT_VertexIndex(1);
		DT_VertexIndex(5);
		DT_VertexIndex(4);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(2);
		DT_VertexIndex(3);
		DT_VertexIndex(7);
		DT_VertexIndex(6);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(0);
		DT_VertexIndex(1);
		DT_VertexIndex(3);
		DT_VertexIndex(2);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(4);
		DT_VertexIndex(5);
		DT_VertexIndex(7);
		DT_VertexIndex(6);
		DT_End();
#else      
		DT_Begin();   
		DT_VertexIndex(0);
		DT_VertexIndex(2);
		DT_VertexIndex(6);
		DT_End();

		DT_Begin();   
		DT_VertexIndex(2);
		DT_VertexIndex(6);
		DT_VertexIndex(4);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(1);
		DT_VertexIndex(3);
		DT_VertexIndex(7);
		DT_End();

		DT_Begin();   
		DT_VertexIndex(3);
		DT_VertexIndex(7);
		DT_VertexIndex(5);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(0);
		DT_VertexIndex(1);
		DT_VertexIndex(5);
		DT_End();

		DT_Begin();   
		DT_VertexIndex(1);
		DT_VertexIndex(5);
		DT_VertexIndex(4);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(2);
		DT_VertexIndex(3);
		DT_VertexIndex(7);
		DT_End();

		DT_Begin();   
		DT_VertexIndex(3);
		DT_VertexIndex(7);
		DT_VertexIndex(6);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(0);
		DT_VertexIndex(1);
		DT_VertexIndex(3);
		DT_End();

		DT_Begin();   
		DT_VertexIndex(1);
		DT_VertexIndex(3);
		DT_VertexIndex(2);
		DT_End();
		
		DT_Begin();   
		DT_VertexIndex(4);
		DT_VertexIndex(5);
		DT_VertexIndex(7);
		DT_End();

		DT_Begin();   
		DT_VertexIndex(5);
		DT_VertexIndex(7);
		DT_VertexIndex(6);
		DT_End();

#endif  

		DT_EndComplexShape();

#else
	
		setShape(DT_NewBox(x, y, z));

#endif

	}    	
    
	virtual void paint() const 
	{
        glPushMatrix();
        glScalef(m_extent[0], m_extent[1], m_extent[2]);
        glutSolidCube(1.0f);
        glPopMatrix();
    }

private:
	MT_Point3           m_vertices[8];
	MT_Vector3          m_extent;
	DT_VertexBaseHandle m_base; 
};


class GLCone : public Shape {
public:
    GLCone(MT_Scalar bottomRadius, MT_Scalar height) 
      :	Shape(DT_NewCone(bottomRadius, height)),
		m_bottomRadius(bottomRadius),
		m_height(height),
		m_displayList(0)
	{}

	virtual void paint() const 
	{ 
		if (m_displayList) 
		{
			glCallList(m_displayList);
		}
		else 
		{
			GLUquadricObj *quadObj = gluNewQuadric();
			m_displayList = glGenLists(1);
			glNewList(m_displayList, GL_COMPILE_AND_EXECUTE);
			glPushMatrix();
			glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
			glTranslatef(0.0f, 0.0f, -0.5f * m_height);
			gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
			gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
			gluCylinder(quadObj, m_bottomRadius, 0.0f, m_height, 15, 10);
			glPopMatrix ();
			glEndList();
		}
	}
  
private:
	MT_Scalar m_bottomRadius;
	MT_Scalar m_height;
    mutable GLuint         m_displayList;
};

class GLCylinder : public Shape {

private:
    mutable GLuint displayList;
public:
    GLCylinder(MT_Scalar radius, MT_Scalar height) 
      :	Shape(DT_NewCylinder(radius, height)),
		m_radius(radius),
		m_height(height),
		m_displayList(0) 
	{}

	virtual void paint() const 
	{ 
		if (m_displayList) 
		{
			glCallList(m_displayList); 
		}
		else 
		{
			GLUquadricObj *quadObj = gluNewQuadric();
			m_displayList = glGenLists(1);
			glNewList(m_displayList, GL_COMPILE_AND_EXECUTE);
			glPushMatrix();
			glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
			glTranslatef(0.0f, 0.0f, -0.5f * m_height);
			gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
			gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
			gluCylinder(quadObj, m_radius, m_radius, m_height, 15, 10);
			glPopMatrix ();
			glEndList();
		}
	}
  
private:
	MT_Scalar m_radius;
	MT_Scalar m_height;
	mutable GLuint m_displayList;
};


void coordSystem()
{
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3d(0.0f, 0.0f, 0.0f);
    glVertex3d(10.0f, 0.0f, 0.0f);
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3d(0.0f, 0.0f, 0.0f);
    glVertex3d(0.0f, 10.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3d(0.0f, 0.0f, 0.0f);
    glVertex3d(0.0f, 0.0f, 10.0f);
    glEnd();
    glEnable(GL_LIGHTING);
}



class Object {
public:
	Object() {}
	Object(Shape *shape, MT_Scalar margin = 0.0f)
	  : m_shape(shape),
		m_object(DT_CreateObject(this, shape->getShape())) 
	{
		DT_SetMargin(m_object, margin);
	}

	~Object() 
	{
		DT_DestroyObject(m_object);
		delete m_shape; 
	}

	void setShape(DT_ShapeHandle shape)
	{
		m_shape->setShape(shape);
	}

	void paint() const 
	{
		float m[16];
		DT_GetMatrixf(m_object, m);
		glPushMatrix(); 
		glMultMatrixf(m);
#ifdef DRAW_COORD
		coordSystem();
#endif
		m_shape->paint();
		glPopMatrix();
	}

	DT_ObjectHandle getHandle() const { return m_object; }

private:
	Shape            *m_shape;
	DT_ObjectHandle   m_object;
};

#if defined(USE_BOXES)

Object obj1(new GLBox(2.0f, 2.0f, 2.0f));
Object obj2(new GLBox(SIZE_RATIO * 2.0f, SIZE_RATIO * 2.0f, SIZE_RATIO * 2.0f));

#elif defined(USE_SPHERES)

#if defined(USE_MARGIN)

Object obj1(new GLSphere(1.0f), 1.0f);
Object obj2(new GLSphere(1.0f), 1.0f);

#else

Object obj1(new GLSphere(1.0f));
Object obj2(new GLSphere(1.0f));

#endif

#elif defined(USE_TRIANGLES)

static DT_Vector3 points[] = { 
	{ -1.0, -1.0, 1.0f } , 
	{  1.0f, -1.0f, 1.0f },
    { -1.0f, 1.0f, 1.0f }
};

Object obj1(new GLTriangle(points));
Object obj2(new GLSphere(1.0f));//Object obj2(new GLTriangle(points));


#else

Object obj1(new GLCone(1.0f, 2.0f));
Object obj2(new GLCylinder(1.0f, 2.0f));

#endif



MT_Point3 cp1, cp2, source, target, spot, normal;
MT_Scalar param;
DT_Bool   intersection;

const void *hit_client;

void display_bbox(const MT_Point3& min, const MT_Point3& max) 
{
    glColor3f(0.0f, 1.0f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_QUAD_STRIP);
    glVertex3d(min[0], min[1], min[2]);
    glVertex3d(min[0], min[1], max[2]);
    glVertex3d(max[0], min[1], min[2]);
    glVertex3d(max[0], min[1], max[2]);
    glVertex3d(max[0], max[1], min[2]);
    glVertex3d(max[0], max[1], max[2]);
    glVertex3d(min[0], max[1], min[2]);
    glVertex3d(min[0], max[1], max[2]);
    glVertex3d(min[0], min[1], min[2]);
    glVertex3d(min[0], min[1], max[2]);
    glEnd();  
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}



#ifdef DISPLAY

void display(void) 
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    obj1.paint();
	obj2.paint();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
 
#ifdef DRAW_BBOX
	MT_Point3 min;
	MT_Point3 max;
	DT_GetBBox(obj1.getHandle(), min, max); 
	display_bbox(min, max);
	DT_GetBBox(obj2.getHandle(), min, max); 
	display_bbox(min, max);
#endif

    glColor3f(1.0f, 1.0f, 0.0f);
#ifdef COMMON_POINT
    if (intersection) {
        glPointSize(5);
        glBegin(GL_POINTS);
        glVertex3d(cp1[0], cp1[1], cp1[2]);
        glEnd();
        glPointSize(1);
    }
    else
#endif 
    {
        glBegin(GL_LINES);
        glVertex3fv(cp1);
        glVertex3fv(cp2);
		glEnd();
	}
	
	
	glColor3f(1.0f, 0.0f, 0.0f);

	if (hit_client && param < 1.0f)
	{
		glBegin(GL_LINES);
		glVertex3fv(source);
		glVertex3fv(spot);
		glEnd();
	
		glColor3f(0.0f, 0.0f, 1.0f);
		glBegin(GL_LINES);
		glVertex3fv(spot);
		glVertex3fv(target);
		glEnd();
	}
	else
	{
		glBegin(GL_LINES);
		glVertex3fv(source);
		glVertex3fv(target);
		glEnd();
	}

	if (hit_client)
	{
		glPointSize(5);
		glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_POINTS);
        glVertex3fv(spot);
        glEnd();
        
		glPointSize(1);

		glColor3f(0.0f, 1.0f, 0.0f);
		
		if (normal.length2() > 0.0f)
		{
			glBegin(GL_LINES);
			glVertex3fv(spot);
			glVertex3fv(spot + normal.normalized());
			glEnd();
		}
    }
  
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    glFlush();
    glutSwapBuffers();
}

#else

void display(void) {}

#endif




void toggleIdle();


MT_Vector3 pos1, pos2;
MT_Quaternion orn1, orn2;
#ifdef SCALING_ON
MT_Vector3 scl1, scl2;
#endif

MT_Vector3 separation;


DT_SceneHandle scene;

void doTest() 
{
	DT_SetPosition(obj1.getHandle(), pos1);
	DT_SetOrientation(obj1.getHandle(), orn1);	
	DT_SetPosition(obj2.getHandle(), pos2);
	DT_SetOrientation(obj2.getHandle(), orn2);

#ifdef SCALING_ON
	DT_SetScaling(obj1.getHandle(), scl1);
	DT_SetScaling(obj2.getHandle(), scl2);
#endif

#ifdef COMMON_POINT
    if ((intersection = DT_GetCommonPoint(obj1.getHandle(), obj2.getHandle(), cp1)))
#else
    if ((intersection = DT_GetPenDepth(obj1.getHandle(), obj2.getHandle(), cp1, cp2)))
#endif

    {
#ifdef LOG
        std::cout << "Intersection    ";
#endif
		separation = cp1 - cp2;

    }
    else {
#ifdef LOG 
        std::cout << "No intersection ";
#endif
        DT_GetClosestPair(obj1.getHandle(), obj2.getHandle(), cp1, cp2);
		separation = cp2 - cp1;
    }
#ifdef LOG    
    std::cout << "distance = " << cp1.distance(cp2);   
    std::cout << std::endl;
#endif

	hit_client = DT_RayCast(scene, 0, source, target, 20.0f, &param, normal);

	if (hit_client == 0)
	{
		hit_client = DT_RayCast(scene, 0, source, target, 20.0f, &param, normal);
	}

	spot = source.lerp(target, param);
 
    display();
    
#ifdef STATISTICS
	extern int num_iterations;
	extern int num_irregularities;

	static int max_iterations = 0;
	static int sum_iterations = 0;
	static int num_placements = 0;

	if (num_iterations > max_iterations) {  
		max_iterations = num_iterations;
	}
    sum_iterations += num_iterations;
	++num_placements;
    std::cout << "#iters = " << num_iterations << ",  max = " << 
        max_iterations << ", avg = " << sum_iterations / num_placements << ",  #irregs = " << 
        num_irregularities << std::endl; 
#endif
}

MT_Scalar distance; 
MT_Scalar ele, azi;

void setCamera(); 

void newPlacements()
{        
#ifdef LOG
	static int num_placements = 0;

	std::cout << num_placements++ << ' ';
#endif

    pos1.setValue(irnd() * SPACE_SIZE, irnd() * SPACE_SIZE, irnd() * SPACE_SIZE);
    pos2.setValue(irnd() * SPACE_SIZE, irnd() * SPACE_SIZE, irnd() * SPACE_SIZE);
	orn1 = MT_Quaternion::random();
	orn2 = MT_Quaternion::random(); 

#ifdef SCALING_ON
	scl1.setValue(SCALE_BOTTOM + SCALE_RANGE * MT_random(),
				  SCALE_BOTTOM + SCALE_RANGE * MT_random(),
				  SCALE_BOTTOM + SCALE_RANGE * MT_random());

	scl2.setValue(SCALE_BOTTOM + SCALE_RANGE * MT_random(),
				  SCALE_BOTTOM + SCALE_RANGE * MT_random(),
				  SCALE_BOTTOM + SCALE_RANGE * MT_random());
#endif
 

	source.setValue(irnd() * SPACE_SIZE, irnd() * SPACE_SIZE, irnd() * SPACE_SIZE);
	target.setValue(irnd() * SPACE_SIZE, irnd() * SPACE_SIZE, irnd() * SPACE_SIZE);

	azi = ele = 0.0f;
    distance = 5.0f;
	doTest();
	setCamera();
}

void toggleIdle() 
{
    static bool idle = true;
    if (idle) 
	{
        glutIdleFunc(newPlacements);
        idle = false;
    }
    else 
	{
        glutIdleFunc(0);
        idle = true;
    }
}

void setCamera() 
{
    glLoadIdentity();
    MT_Scalar rele = MT_radians(ele);
    MT_Scalar razi = MT_radians(azi);
    MT_Point3 eye(distance * sin(razi) * cos(rele), 
                  distance * sin(rele),
                  distance * cos(razi) * cos(rele));

	gluLookAt(eye[0], eye[1], eye[2], 
              cp1[0], cp1[1], cp1[2], 
              0.0f, 1.0f, 0.0f);
	display();
}


void stepLeft() { azi -= STEPSIZE; if (azi < -180.0f) azi += 360.0f; setCamera(); }
void stepRight() { azi += STEPSIZE; if (azi >= 180.0f) azi -= 360.0f; setCamera(); }
void stepFront() { ele += STEPSIZE; if (azi >= 180.0f) azi -= 360.0f; setCamera(); }
void stepBack() { ele -= STEPSIZE; if (azi < -180.0f) azi += 360.0f; setCamera(); }
void zoomIn() { distance -= MT_Scalar(0.1); setCamera(); }
void zoomOut() { distance += MT_Scalar(0.1); setCamera(); }


void separate() 
{ 
	if (separation.length2() != MT_Scalar(0.0))
	{
		pos2 += 0.01f * separation.normalized(); 
	    doTest(); 
	}
}

void connect()
{ 
	if (separation.length2() != MT_Scalar(0.0))
	{
		pos2 -= 0.01f * separation.normalized(); 
	    doTest(); 
	} 
}


void move_up() { pos2[1] += 0.01f; doTest(); }
void move_down() { pos2[1] -= 0.01f;doTest();  }
void move_right() { pos2[0] += 0.01f; doTest(); }
void move_left() { pos2[0] -= 0.01f; doTest(); }

void myReshape(int w, int h) 
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0f, (float)w/(float)h, 1.0f, 200.0f);
    glMatrixMode(GL_MODELVIEW);
}

void myKeyboard(unsigned char key, int x, int y)
{
    switch (key) 
	{
    case 'z': 
		zoomIn(); 
		break;
    case 'x': 
		zoomOut(); 
		break;
	case 'w': 
		move_up(); 
		break;
    case 's': 
		move_down(); 
		break;
	case 'a': 
		move_left(); 
		break;
    case 'd': 
		move_right(); 
		break;  
	case 'i':
		toggleIdle(); 
		break;
    case ' ':
		newPlacements();
		break;
    default:
        break;
    }
}

void mySpecial(int key, int x, int y)
{
    switch (key) 
	{
    case GLUT_KEY_LEFT: 
		stepLeft(); 
		break;
    case GLUT_KEY_RIGHT: 
		stepRight(); 
		break;
    case GLUT_KEY_UP:
		stepFront(); 
		break;
    case GLUT_KEY_DOWN:
		stepBack(); 
		break;
    case GLUT_KEY_PAGE_UP:
		zoomIn(); 
		break;
    case GLUT_KEY_PAGE_DOWN:
		zoomOut(); 
		break;
    case GLUT_KEY_HOME:
		toggleIdle();
		break;
    default:
        break;
    }
}

void goodbye( void)
{
    std::cout << "goodbye ..." << std::endl;
    exit(0);
}

void menu(int choice)
{

    static int fullScreen = 0;
    static int px, py, sx, sy;
 
    switch(choice) {
    case 1:
        if (fullScreen==1) {
            glutPositionWindow(px, py);
            glutReshapeWindow(sx, sy);
            glutChangeToMenuEntry(1, "Full Screen", 1);
            fullScreen=0;
        } 
		else {
            px = glutGet((GLenum)GLUT_WINDOW_X);
            py = glutGet((GLenum)GLUT_WINDOW_Y);
            sx = glutGet((GLenum)GLUT_WINDOW_WIDTH);
            sy = glutGet((GLenum)GLUT_WINDOW_HEIGHT);
            glutFullScreen();
            glutChangeToMenuEntry(1, "Close Full Screen", 1);
            fullScreen = 1;
        }
        break;
    case 2:
        toggleIdle();
        break;
    case 3:
        goodbye();
        break;
    default:
        break;
    }
}

void init(void) 
{
    GLfloat light_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };

    /*	light_position is NOT default value	*/
    GLfloat light_position0[] = { 1.0f, 1.0f, 1.0f, 0.0f };
    GLfloat light_position1[] = { -1.0f, -1.0f, -1.0f, 0.0f };
  
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
  
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
  

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
  
    glShadeModel(GL_SMOOTH);
  
#ifdef SCALING_ON
	glEnable(GL_NORMALIZE);
#endif

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
  
    //  glEnable(GL_CULL_FACE);
    //  glCullFace(GL_BACK);

	//DT_SetAccuracy(1.0e-2);

	scene = DT_CreateScene();
	DT_AddObject(scene, obj1.getHandle());
	DT_AddObject(scene, obj2.getHandle());
}

void createMenu()
{
    glutCreateMenu(menu);
    glutAddMenuEntry("Full Screen", 1);
    glutAddMenuEntry("Toggle Idle (Start/Stop)", 2);
    glutAddMenuEntry("Quit", 3);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

int main(int argc, char **argv) 
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(500, 500);
    glutCreateWindow("GLdemo");

    init();
    glutKeyboardFunc(myKeyboard);
    glutSpecialFunc(mySpecial);
    glutReshapeFunc(myReshape);
    createMenu();
	
    glutIdleFunc(0);
	newPlacements();
    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}
