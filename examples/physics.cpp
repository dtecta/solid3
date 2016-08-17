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

//#define STACKING            // Try stacking some boxes.
//#define FAKE_IT             // Set the inertia of rotation to infinite. 
                              // (Cheat to get things stable.) 
//#define USE_COMPLEX_GROUND  // Use a 20x20 mesh of triangles as ground surface                                
//#define USE_QUADS           // Idem, but now with quads  

#include <new>
#include <GL/glut.h>

#include "GEN_MinMax.h"
#include "MT_Point3.h"
#include "MT_Vector3.h"
#include "MT_Quaternion.h"
#include "MT_Matrix3x3.h"
#include "MT_Transform.h"


#include <SOLID.h>
#include "RigidBody.h"

const MT_Scalar TimeStep = 0.01; //Physics runs @ 100Hz

const MT_Scalar GroundMargin =  5.0f;
const MT_Scalar BodyMargin = 0.1f;

const MT_Scalar LinDamping = 0.1f;
const MT_Scalar AngDamping = 0.1f;

const MT_Scalar Restitution = 0.7f;
const MT_Scalar Friction    = 0.7f;

const int MaxImpulseIterations = 3;
const int MaxRelaxIterations = 3;
const MT_Scalar Relaxation  = 0.5f;

void coordSystem() 
{
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3d(0, 0, 0);
    glVertex3d(10, 0, 0);
    glColor3f(0, 1, 0);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 10, 0);
    glColor3f(0, 0, 1);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 0, 10);
    glEnd();
    glEnable(GL_LIGHTING);
}

void display_bbox(const MT_Point3& min, const MT_Point3& max) 
{
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glColor3f(0, 1, 1);
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
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
}



class GLShape {
public:
    virtual void paint() const = 0;
};


class GLSphere : public GLShape {
    MT_Scalar radius;
public:
    GLSphere(MT_Scalar r) : radius(r) {}

    void paint() const 
	{
        glutSolidSphere(radius, 20, 20);
    }
};


class GLBox : public GLShape {
    MT_Vector3 extent;
public:
    GLBox(MT_Scalar x, MT_Scalar y, MT_Scalar z) : 
        extent(x, y, z) {}

    void paint() const 
	{
        glPushMatrix();
        glScaled(extent[0], extent[1], extent[2]);
        glutSolidCube(1.0);
        glPopMatrix();
    }
};


class GLCone : public GLShape {
    MT_Scalar bottomRadius;
    MT_Scalar height;
    mutable GLuint displayList;

public:
    GLCone(MT_Scalar r, MT_Scalar h) :  
        bottomRadius(r), 
        height(h), 
        displayList(0) {}
  
    void paint() const 
	{ 
        if (displayList) glCallList(displayList); 
        else {
            GLUquadricObj *quadObj = gluNewQuadric();
            displayList = glGenLists(1);
            glNewList(displayList, GL_COMPILE_AND_EXECUTE);
            glPushMatrix();
            glRotatef(-90.0, 1.0, 0.0, 0.0);
            glTranslatef(0.0, 0.0, -1.0);
            gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
            gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
            gluCylinder(quadObj, bottomRadius, 0, height, 15, 10);
            glPopMatrix();
            glEndList();
        }
    }
};

class GLCylinder : public GLShape {
    MT_Scalar radius;
    MT_Scalar height;
    mutable GLuint displayList;

public:
    GLCylinder(MT_Scalar r, MT_Scalar h) : 
        radius(r), 
        height(h), 
        displayList(0) {}

    void paint() const 
	{ 
        if (displayList) glCallList(displayList); 
        else {
            GLUquadricObj *quadObj = gluNewQuadric();
            displayList = glGenLists(1);
            glNewList(displayList, GL_COMPILE_AND_EXECUTE);
            glPushMatrix();
            glRotatef(-90.0, 1.0, 0.0, 0.0);
            glTranslatef(0.0, 0.0, -1.0);
            gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
            gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
            gluCylinder(quadObj, radius, radius, height, 15, 10);
            glPopMatrix ();
            glEndList();
        }
    }
};




struct MassProps {
	DT_Scalar   m_mass;
	DT_Vector3  m_inertia;
};

class Object : public RigidBody {
public:
    Object() {}
    Object(GLShape *gl_shape, DT_ShapeHandle shape, const MassProps& massProps)
	  : m_gl_shape(gl_shape),
        m_shape(shape),
        m_object(DT_CreateObject(this, shape)),
        m_penalty(0.0f, 0.0f, 0.0f),
		m_scaling(1.0f, 1.0f, 1.0f),
		m_color(MT_Vector3::random().absolute())
	{
#ifdef FAKE_IT
      MT_Vector3 inertia(0.0f, 0.0f, 0.0f);
#else
      MT_Vector3 inertia(massProps.m_inertia);
#endif
		setMassProps(massProps.m_mass, inertia);
      setDamping(LinDamping, AngDamping);
	}
	
    ~Object() { DT_DestroyObject(m_object); }
	
	
    DT_ObjectHandle getHandle() const { return m_object; }
    
	void correct(const MT_Vector3& error) 
	{
		m_penalty += error;
	}
	
 	void setTransform(const MT_Transform& xform)
	{
		RigidBody::setTransform(xform);	
		float m[16];
		xform.getValue(m);
		DT_SetMatrixf(m_object, m);
		DT_SetScaling(m_object, m_scaling);
	}
	
	void relax()
	{
		translate(m_penalty * Relaxation);	
		m_penalty.setValue(0.0f, 0.0f, 0.0f);
		DT_SetPosition(m_object, getPosition()); 
	}
	
    void proceed(MT_Scalar step)
	{
		RigidBody::proceed(step);
		float m[16];
		m_xform.getValue(m);
		DT_SetMatrixf(m_object, m);
		DT_SetScaling(m_object, m_scaling);
	}
	
	void setScaling(MT_Scalar x, MT_Scalar y, MT_Scalar z)
	{
		m_scaling.setValue(x, y, z);
	}
	
	void paint() 
	{
		float m[16];
		DT_GetMatrixf(m_object, m);
		glPushMatrix(); 
		glMultMatrixf(m);
#ifdef DRAW_COORD
		coordSystem();
#endif
		glColor3fv(m_color);
		m_gl_shape->paint();
		glPopMatrix();
	}
	
private:
    GLShape        *m_gl_shape;
    DT_ShapeHandle  m_shape;
    DT_ObjectHandle m_object;
    MT_Vector3      m_penalty;
	MT_Vector3      m_scaling;
	MT_Vector3      m_color; 
};



MassProps immobile = {
    0.0f,
    { 0.0f, 0.0f, 0.0f }
};


MassProps massBox = {
    1.0f,
    { 0.67f, 0.67f, 0.67f }
};

MassProps massBlock = {
    1.0f,
    { 3.33f , 0.67f, 3.33f }
};

MassProps massSphere = {
    1.0f,
    { 0.4f, 0.4f, 0.4f }
};

MassProps massCone = {
    1.0f,
    { 0.4f, 0.4f, 0.4f }
};

MassProps massCylinder = {
    1.0f,
    { 0.58f, 0.5f, 0.58f }
};

const MT_Scalar SPACE_SIZE = 2;

const int NumObjects     = 5;
Object object[NumObjects];

static GLSphere       gl_sphere(1.0f);
static DT_ShapeHandle sm_sphere = DT_NewSphere(1.0f);

static GLCylinder     gl_cylinder(1.0f, 2.0f);
static DT_ShapeHandle sm_cylinder = DT_NewCylinder(1.0f, 2.0f);

static GLCone         gl_cone(1.0f, 2.0f);
static DT_ShapeHandle sm_cone = DT_NewCone(1.0f, 2.0f);

static GLBox          gl_box(2.0f, 2.0f, 2.0f);
static DT_ShapeHandle sm_box = DT_NewBox(2.0f, 2.0f, 2.0f);

static GLBox          gl_block(2.0f, 6.0f, 2.0f);
static DT_ShapeHandle sm_block = DT_NewBox(2.0f, 6.0f, 2.0f);

static GLBox          gl_ground(50.0f, 2.0f * GroundMargin, 50.0f);

#ifdef USE_COMPLEX_GROUND

const int GRID_SCALE = 1;
const MT_Scalar GRID_UNIT  = 25.0f / GRID_SCALE;

DT_ShapeHandle createComplex() 
{
    DT_ShapeHandle shape = DT_NewComplexShape(0);
    for (int i0 = -GRID_SCALE; i0 != GRID_SCALE; ++i0) 
	{
        for (int j0 = -GRID_SCALE; j0 != GRID_SCALE; ++j0) 
		{
            int i1 = i0 + 1;
            int j1 = j0 + 1;

			MT_Vector3 verts[4];
			verts[0].setValue(GRID_UNIT * i0, 0.0f, GRID_UNIT * j0);
            verts[1].setValue(GRID_UNIT * i0, 0.0f, GRID_UNIT * j1);
            verts[2].setValue(GRID_UNIT * i1, 0.0f, GRID_UNIT * j0);
            verts[3].setValue(GRID_UNIT * i1, 0.0f, GRID_UNIT * j1);
#ifdef USE_QUADS
            DT_Begin();
            DT_Vertex(verts[0]);
            DT_Vertex(verts[1]);
            DT_Vertex(verts[2]);
            DT_Vertex(verts[3]);
            DT_End();
#else
            DT_Begin();
            DT_Vertex(verts[0]);
            DT_Vertex(verts[1]);
            DT_Vertex(verts[2]);
            DT_End();

            DT_Begin();
            DT_Vertex(verts[1]);
            DT_Vertex(verts[2]);
            DT_Vertex(verts[3]);
            DT_End();
#endif

        }
    }
    DT_EndComplexShape();
    return shape;
}

static DT_ShapeHandle sm_ground = createComplex();

#else 

static DT_ShapeHandle sm_ground = DT_NewBox(50.0f, 0.0f, 50.0f);

#endif

static Object ground(&gl_ground, sm_ground, immobile);

static DT_SceneHandle     g_scene;

static DT_RespTableHandle g_respTable;
static DT_ResponseClass   g_bodyClass;
static DT_ResponseClass   g_groundClass;

static DT_RespTableHandle g_fixRespTable;
static DT_ResponseClass   g_fixBodyClass;
static DT_ResponseClass   g_fixGroundClass;

DT_Bool body2body(void *client_data,  
			   void *object1,
			   void *object2,
			   const DT_CollData *coll_data) 
{
	Object& obj1 = *(Object *)object1;
	Object& obj2 = *(Object *)object2;
	MT_Point3 pos1(coll_data->point1);
	MT_Point3 pos2(coll_data->point1);
	MT_Vector3 normal(coll_data->normal);
	MT_Scalar depth = normal.length();
	if (depth > MT_EPSILON)
	{
		normal /= depth;
		resolveCollision(obj1, pos1, obj2, pos2, depth, normal, Restitution); 
	}

	return DT_CONTINUE;
}

DT_Bool body2body_fix(void *client_data,  
				   void *object1,
				   void *object2,
				   const DT_CollData *coll_data) 
{
	Object& obj1 = *(Object *)object1;
	Object& obj2 = *(Object *)object2;
	MT_Vector3 normal(coll_data->normal);	
   
	MT_Vector3 error = normal * 0.5f;
	obj1.correct(error);
	obj2.correct(-error);
	
	return DT_CONTINUE;
}

DT_Bool body2ground(void *client_data,  
				 void *object1,
				 void *object2,
				 const DT_CollData *coll_data) 
{
	Object& obj1 = *(Object *)object1;
	MT_Point3 pos1(coll_data->point1);
	MT_Vector3 normal(coll_data->normal);
	MT_Scalar depth = normal.length();
	if (depth > MT_EPSILON)
	{
		normal /= depth;
		obj1.resolveCollision(pos1, depth, normal, Restitution, Friction); 
	}
	
	return DT_CONTINUE;
}

DT_Bool body2ground_fix(void *client_data,  
					 void *object1,
					 void *object2,
					 const DT_CollData *coll_data) 
{
	Object& obj1 = *(Object *)object1;
	MT_Vector3 normal(coll_data->normal);
	obj1.correct(normal); 
	
	return DT_CONTINUE;
}





void display(void) 
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    ground.paint();
    
	int i;
    for (i = 0; i < NumObjects; ++i) 
	{
        object[i].paint();
    }

    glFlush();
    glutSwapBuffers();
}





static MT_Scalar DISTANCE = 20; 

static MT_Scalar ele = 0, azi = 0;
static MT_Point3 eye(0.0f, 0.0f, DISTANCE);
static MT_Point3 center(0, 0, 0);

inline double irnd() { return 2 * MT_random() - 1; }  

static const double SCALE_BOTTOM = 0.5;
static const double SCALE_FACTOR = 2;

void setCamera();

void newPlacements() 
{
    /* The platform */
    ground.setTransform(MT_Transform(MT_Quaternion(0, 0, 0, 1), MT_Point3(0, -10, 0)));
    
    center.setValue(0.0, 0.0, 0.0);
	
#ifdef STACKING
	int i;
    for (i = 0; i < NumObjects; ++i) 
    {
        object[i].reset(MT_Transform(MT_Quaternion(0.0f, 0.0f, 0.0f, 1.0f), 
                                     MT_Point3(0.0f, -3.0f + 2.5f * i, 0.0f)));
    }
#else
	int i;
    for (i = 0; i < NumObjects; ++i) 
	{
        object[i].reset(MT_Transform(MT_Quaternion::random(), MT_Point3(0.0f, 5.0f * i, 0.0f)));
    }

#endif

    setCamera();
}

void moveAndDisplay() 
{
   int i;
   for (i = 0; i < NumObjects; ++i) 
   {
	   object[i].applyForces(TimeStep);
	   object[i].proceed(TimeStep);
   }
   
   int k = MaxImpulseIterations;
   while (k-- && DT_Test(g_scene, g_respTable))
   {
	   for (i = 0; i < NumObjects; ++i) 
	   {
		   object[i].backup();
		   object[i].proceed(TimeStep);
	   }
   }
   
   DT_Test(g_scene, g_respTable);
   
   k = MaxRelaxIterations;
   while (k-- && DT_Test(g_scene, g_fixRespTable))
   {
	   for (i = 0; i < NumObjects; ++i) 
	   {
		   object[i].relax();
	   }
   }
   
   display();
}

void toggleIdle() 
{
    static bool idle = true;
    if (idle) 
	{
        glutIdleFunc(moveAndDisplay);
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
    eye.setValue(DISTANCE * sin(razi) * cos(rele), 
                 DISTANCE * sin(rele),
                 DISTANCE * cos(razi) * cos(rele));
    gluLookAt(eye[0], eye[1], eye[2], 
              center[0], center[1], center[2], 
              0, 1, 0);
    display();
}

const MT_Scalar STEPSIZE = 5;

void stepLeft() { azi -= STEPSIZE; if (azi < 0) azi += 360; setCamera(); }
void stepRight() { azi += STEPSIZE; if (azi >= 360) azi -= 360; setCamera(); }
void stepFront() { ele += STEPSIZE; if (azi >= 360) azi -= 360; setCamera(); }
void stepBack() { ele -= STEPSIZE; if (azi < 0) azi += 360; setCamera(); }
void zoomIn() { DISTANCE -= 1; setCamera(); }
void zoomOut() { DISTANCE += 1; setCamera(); }


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

void init(void) 
{
    GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };

    /*	light_position is NOT default value	*/
    GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
    GLfloat light_position1[] = { -1.0, -1.0, -1.0, 0.0 };
  
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
  
	glEnable(GL_COLOR_MATERIAL);
	glShadeModel(GL_SMOOTH);
  
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
  
    //  glEnable(GL_CULL_FACE);
    //  glCullFace(GL_BACK);

    g_scene       = DT_CreateScene();
    g_respTable   = DT_CreateRespTable();
	g_bodyClass   = DT_GenResponseClass(g_respTable);
	g_groundClass = DT_GenResponseClass(g_respTable);
	 
    
    DT_AddPairResponse(g_respTable, g_bodyClass, g_bodyClass, body2body, DT_DEPTH_RESPONSE, 0);  
    DT_AddPairResponse(g_respTable, g_bodyClass, g_groundClass, body2ground, DT_DEPTH_RESPONSE, 0); 
    
    g_fixRespTable   = DT_CreateRespTable();
	g_fixBodyClass   = DT_GenResponseClass(g_fixRespTable);
	g_fixGroundClass = DT_GenResponseClass(g_fixRespTable);
	 
    DT_AddPairResponse(g_fixRespTable, g_fixBodyClass, g_fixBodyClass, body2body_fix, DT_DEPTH_RESPONSE, 0);  
    DT_AddPairResponse(g_fixRespTable, g_fixBodyClass, g_fixGroundClass, body2ground_fix, DT_DEPTH_RESPONSE, 0);  

    DT_SetMargin(ground.getHandle(), GroundMargin);

    DT_AddObject(g_scene, ground.getHandle());
    DT_SetResponseClass(g_respTable, ground.getHandle(), g_groundClass);
    DT_SetResponseClass(g_fixRespTable, ground.getHandle(), g_fixGroundClass);

    int i;

#ifdef STACKING
    
    for (i = 0; i < NumObjects; ++i) 
	{
        new(&object[i]) Object(&gl_box, sm_box, massBox);
    }

#else

    new(&object[0]) Object(&gl_cylinder, sm_cylinder, massSphere);
    new(&object[1]) Object(&gl_cone, sm_cone, massCone);
    new(&object[2]) Object(&gl_box, sm_box, massBox);
    new(&object[3]) Object(&gl_block, sm_block, massBlock);
    new(&object[4]) Object(&gl_sphere, sm_sphere, massSphere);
  
#endif

    for (i = 0; i < NumObjects; ++i) 
	{
		DT_SetMargin(object[i].getHandle(), BodyMargin);
		object[i].setGravity(MT_Vector3(0.0, -9.8, 0.0));
		
		DT_AddObject(g_scene, object[i].getHandle());
		DT_SetResponseClass(g_respTable, object[i].getHandle(), g_bodyClass); 
		DT_SetResponseClass(g_fixRespTable, object[i].getHandle(), g_fixBodyClass);
    }
    
	
    newPlacements();
}

void goodbye( void)
{
    for (int i = 0; i < NumObjects; ++i) 
    {
        DT_RemoveObject(g_scene, object[i].getHandle());
    }

    DT_RemoveObject(g_scene, ground.getHandle());
    DT_DestroyScene(g_scene);
    DT_DestroyRespTable(g_respTable);

    std::cout << "goodbye ..." << std::endl;
    exit(0);
}

void menu(int choice)
{

    static int fullScreen = 0;
    static int px, py, sx, sy;
 
    switch(choice) {
    case 1:
        if (fullScreen == 1) 
		{
            glutPositionWindow(px,py);
            glutReshapeWindow(sx,sy);
            glutChangeToMenuEntry(1,"Full Screen",1);
            fullScreen = 0;
        } 
		else 
		{
            px=glutGet((GLenum)GLUT_WINDOW_X);
            py=glutGet((GLenum)GLUT_WINDOW_Y);
            sx=glutGet((GLenum)GLUT_WINDOW_WIDTH);
            sy=glutGet((GLenum)GLUT_WINDOW_HEIGHT);
            glutFullScreen();
            glutChangeToMenuEntry(1,"Close Full Screen",1);
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
    glutCreateWindow("Physics demo");

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
