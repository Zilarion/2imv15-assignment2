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
    printf ( "\t Spawn a rigid body with the 'r' key\n" );
    printf ( "\t Spawn a heavier rigid body with the 'h' key\n" );
	printf ( "\t Toggle adaptive with the 'a' key\n" );
	printf ( "\t Toggle visibility of forces with the 'f' key\n" );
	printf ( "\t Toggle visibility of constraints with the 'o' key\n" );
	printf ( "\t Use ',' '.' and '/' to rotate\n" );
	printf ( "\t Use '6' to 9' and '0' to change between Scenes\n" );
	printf ( "\t 6: Basic water scene\n" );
	printf ( "\t 7: Water scene in a funnel\n" );
	printf ( "\t 8: Smoke scene\n" );
	printf ( "\t 9: Cloth scene\n" );
	printf ( "\t 0: Hair scene\n" );

	View v(768, 512, dt, N);
    v.initialize(SystemBuilder::BASIC);
	exit (0);
}

