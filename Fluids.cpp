// ParticleToy.cpp : Defines the entry point for the console application.
//

#include "View.h"

#if defined(__CYGWIN__) || defined(WIN32)
	#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );
    float dt;
    int N;
    float d;
	if ( argc == 1 ) {
		N = 64;
		dt = 0.002f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );
	printf ( "\t Decrease/increase time step by pressing the '-' and '=' keys\n" );
    printf ( "\t Spawn fluid particles with the 'p' key\n" );
    printf ( "\t Spawn smoke particles in the smoke scene with the 'o' key\n" );
    printf ( "\t Spawn a rigid body with the 'r' key\n" );
	printf ( "\t Toggle adaptive with the 'a' key\n" );
    printf ( "\t Toggle visibility of forces with the 'f' key\n" );
    printf ( "\t Toggle visibility of velocities with the 'v' key\n" );
    printf ( "\t Toggle marching cubes with the 'm' key\n" );
	printf ( "\t Use ',' '.' and '/' to rotate\n" );
    printf ( "\t Use '1' to '4' to change between solvers:\n" );
    printf ( "\t 1: Explicit Euler\n" );
    printf ( "\t 2: Semi explicit euler\n" );
    printf ( "\t 3: 4th order Runge Kutta\n" );
    printf ( "\t 4: Mid point\n" );
	printf ( "\t Use '5' to '9' to change between Scenes\n" );
    printf ( "\t 5: Basic cloth scene\n" );
	printf ( "\t 6: Basic water scene\n" );
	printf ( "\t 7: Water scene in a funnel\n" );
	printf ( "\t 8: Smoke scene\n" );
	printf ( "\t 9: Water in glass scene\n" );
    printf ( "\t Change the wind direction to left using 'j'\n" );
    printf ( "\t Change the wind direction to forward using 'i'\n" );
    printf ( "\t Change the wind direction to backward using 'k'\n" );
    printf ( "\t Change the wind direction to right using 'l'\n" );
    printf ( "\t Disable wind using 'u'\n" );

	View v(768, 512, dt, N);
    v.initialize(SystemBuilder::CLOTH);
	exit (0);
}

