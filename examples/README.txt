This directory contains a number of sample applications for SOLID 3.

sample: 
		This is a console application that performs 10000 static intersection
		tests on a pair of rings at random placements. By default the rings
		are composed of 5000 triangles. By setting the USE_QUADS flag the
		rings are built from 2500 quads. By setting USE_HULL, the convex hulls
		of the rings (closed discs with rounded edges) are used as collision
		shapes.

gldemo: 
		This is the main demo of SOLID 3 features. The application is
		controlled using following keys: 
		
		Arrow keys				Control camera view position
		z, x					Control distance to target point
		Page Up, Page Down		Idem
		Space					Generate new random placements
		i						Toggle interactive/batch mode
		Home					idem
		w, a, s, d				Move one of the objects

		The yellow lines represents the distance or penetration depth vector.
		The white dot is the hit spot of the ray. The part of the ray from the
		source to the hit spot is rendered in red, and the part from the hit
		spot to the target is rendered in blue. If the source of the ray is
		located outside the objects, then a green line represents the normal
		at the hit spot. Besides boxes this application can be used to test
		a pair of spheres, triangles, cones, and cylinders. See the gldemo.cpp
		source file for the different compile options.

physics:	   	 
		This is an attempt to physically simulate collisions and contacts
		relying solely on local information. No global optimization scheme a
		la Baraff is used here (linear complementarity). Instead, the
		collisions and contacts are resolved iteratively. Collision resolution
		is performed in two phases. Firstly, the relative velocities of all
		contacts are corrected using impulses. Secondly, the interpenetrations 
		are resolved using relaxation. 

		In the impulse phase, for each collision, a counter impulse is
		applied, and the objects are backed up to their previous position, but
		without setting their velocities back to the previous frame. The
		integration step is repeated for the same time step, and secondary
		collisions are resolved in the same way. This procedure is repeated
		MaxImpulseIterations times.

		In the relaxation phase, the penetration depth vector is added to a
		penalty vector, and the objects are translated over a fraction of the
		penalty vector. The fraction is given by the Relaxation constant. This 
		procedure is repeated MaxRelaxIterations times.

mnm:
		This is a stress test for the collision detection and penetration
		depth computation. It basically is the same application as "physics",
		only here, 100 ellipsoids are tossed in a polygonal 'parabowl'.

		The two applications are controlled using the following keys

 		Arrow keys				Control camera view position
		z, x					Control distance to target point
		Page Up, Page Down		Idem
		Space					Generate new random placements
		i						Start/stop animation
		Home					idem


 