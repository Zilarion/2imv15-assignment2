//
// Created by Ruud Andriessen on 10/05/2017.
//

#include "View.h"
#include "System.h"
#include "solvers/RungeKutta.h"
#include "solvers/Euler.h"
#include "solvers/Midpoint.h"
#include "forces/DragForce.h"
#include "forces/PressureForce.h"

#if defined(__CYGWIN__) || defined(WIN32)
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif

// Callback handles due to glut being ugly
static View* currentInstance = NULL;

void displayCallback() {
    currentInstance->onDisplay();
}
void idleCallback() {
    currentInstance->onIdle();
}
void motionCallback(int x, int y) {
    currentInstance->onMotionEvent(x, y);
}
void reshapeCallback(int w, int h) {
    currentInstance->onReshape(w, h);
}
void mouseCallback(int b, int s, int x, int y) {
    currentInstance->onMouseEvent(b, s, x, y);
}
void keypressCallback(unsigned char k, int x, int y) {
    currentInstance->onKeyPress(k, x, y);
}

View::View(int width, int height, float dt, int N)
        : width(width), height(height), isSimulating(false), dumpFrames(false), drawVelocity(false), drawForces(false),
          drawConstraints(true), drawMarchingCubes(false), adaptive(false), frameNumber(0), dt(dt), N(N) {
}

void View::initialize(SystemBuilder::AvailableSystems system) {
    glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );

    glutInitWindowPosition ( 0, 0 );
    glutInitWindowSize ( width, height );
    id = glutCreateWindow ( "Fluids!" );

    glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);


    // enable lights
    GLfloat ambient[] = {0.2, 0.2, 0.2, 1.0};
    GLfloat diffuse[] = {0.8, 0.8, 0.8, 1.0};
    GLfloat specular[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat lightPosition[] = {1.0, 1.0, 1.0, 0.0};

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDepthFunc(GL_LEQUAL);
    glShadeModel(GL_SMOOTH);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    currentInstance = this;

    glutKeyboardFunc ( keypressCallback );
    glutMouseFunc ( mouseCallback );
    glutMotionFunc ( motionCallback );
    glutReshapeFunc ( reshapeCallback );
    glutIdleFunc ( idleCallback );
    glutDisplayFunc ( displayCallback );

    sys = SystemBuilder::get(system);
    wind = new DirectionalForce(sys->particles, Vector3f(0.f, 0.f, 0.f));
    sys->addForce(wind);
    mouseDragForce = new DirectionalForce({sys->particles[0]}, Vector3f(0,0,0));

    glutMainLoop ();
}

void View::onKeyPress ( unsigned char key, int x, int y )
{
    float rx, rz;
    switch ( key )
    {
        case 'd':
        case 'D':
            dumpFrames = !dumpFrames;
            break;
        case 'q':
        case 'Q':
            sys->free ();
            exit ( 0 );
            break;
        case '1':
            printf("Using Explicit Euler\n");
            sys->solver = new Euler(Euler::EXPLICIT);
            break;
        case '2':
            printf("Using Semi Explicit Euler\n");
            sys->solver = new Euler(Euler::SEMI);
            break;
        case '3':
            printf("Using 4th order Runge-Kutta\n");
            sys->solver = new RungeKutta();
            break;
        case '4':
            printf("Using Midpoint\n");
            sys->solver = new Midpoint();
            break;
        case '5':
            printf("Cloth + water scene\n");
            sys = SystemBuilder::get(SystemBuilder::CLOTH);
            sys->type = SystemBuilder::CLOTH;
            break;
        case '6':
            printf("Basic water scene\n");
            sys = SystemBuilder::get(SystemBuilder::BASIC);
            sys->type = SystemBuilder::BASIC;
            break;
        case '7':
            printf("Water scene with a funnel\n");
            sys = SystemBuilder::get(SystemBuilder::WATER);
            sys->type = SystemBuilder::WATER;
            break;
        case '8':
            printf("Smoke scene\n");
            sys = SystemBuilder::get(SystemBuilder::SMOKE);
            sys->type = SystemBuilder::SMOKE;
            break;
        case '9':
            printf("Water in a glass scene\n");
            sys = SystemBuilder::get(SystemBuilder::GLASS);
            sys->type = SystemBuilder::GLASS;
            break;
//        case '0':
//            printf("Hair scene\n");
//            sys = SystemBuilder::get(SystemBuilder::CLOTH);
//            sys->type = SystemBuilder::CLOTH;
//            break;
        case '=':
            sys->dt += 0.001f;
            printf("Increase dt: %f\n", sys->dt);
            break;
        case '-':
            sys->dt -= 0.001f;
            printf("Decrease dt: %f\n", sys->dt);
            break;
        case 'v':
            drawVelocity = !drawVelocity;
            break;
        case 'f':
            drawForces = !drawForces;
            break;
        case 'i':
            wind->direction = Vector3f(0.0f, 0.0f, -10.0f);
            break;
        case 'j':
            wind->direction = Vector3f(-10.0f, 0.0f, 0.0f);
            break;
        case 'k':
            wind->direction = Vector3f(0.0f, 0.0f, 10.0f);
            break;
        case 'l':
            wind->direction = Vector3f(10.0f, 0.0f, 0.0f);
            break;
        case 'u':
            wind->direction = Vector3f(0.0f, 0.0f, 0.0f);
            break;
        case ',':
            rotate = 1;
            break;
        case '.':
            rotate = -1;
            break;
        case '/':
            rotate = 0;
            break;
        case 'a':
            adaptive = !adaptive;
            if (adaptive)
                printf("Adaptive on\n");
            else
                printf("Adaptive off\n");
            break;
        case 'p':
            for (int i = 0; i < 1; i++)
                sys->addParticle(new Particle(Vector3f((rand() % 10 + 1) * 0.01f, .3f, (rand() % 10 + 1) * 0.01f),
                                              1.f, sys->particles.size() + 1, true));
            break;
        case 'o':
            if(sys->type==SystemBuilder::SMOKE) {
                for (int i = 0; i < 100; i++)
                    sys->addParticle(new Particle(Vector3f((rand() % 10 + 1) * 0.02f, -.8f - (rand() % 10 + 1) * 0.05f,
                                                           (rand() % 10 + 1) * 0.02f),
                                                  1.f, sys->particles.size() + 1, true));
            }
            break;
        case 'r':
        {
            RigidBody* r = new RigidBody(Vector3f(0,0,0), Vector3f(.15f,.15f,.15f), Vector3f(5,5,5), 3.f);
            sys->addRigidBody(r);
            break;
        }
        case ' ':
            isSimulating = !isSimulating;
            if(isSimulating)
                sys->reset ();
            break;
        case 'M':
        case 'm':
            drawMarchingCubes = !drawMarchingCubes;
            printf("marching cubes: %s\n", drawMarchingCubes ? "true" : "false");
            break;
    }
}

void View::onMouseEvent( int button, int state, int x, int y )
{
    initialMx = omx = mx = x;
    initialMy = omx = my = y;

    if(!mouse_down[0]){hmx=x; hmy=y;}
    if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
    if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
    mouse_down[button] = state == GLUT_DOWN;

    //Reset force on mouse up
    if(state == GLUT_UP){
        mouseDragForce->setActive(false);
    } else {
        //get world coordinates of click point
        GLdouble modelMatrix[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
        GLdouble projectionMatrix[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
        GLint viewMatrix[4];
        glGetIntegerv(GL_VIEWPORT, viewMatrix);
        Particle *closestParticle = sys->particles[0];
        double closestDistance = 10000000;
        vector<Particle*> particleCandidates = sys->particles;
//        for(RigidBody *r:sys->rigidBodies){
//            for(Particle *p:r->particles){
//                particleCandidates.push_back(p);
//            }
//        }
        for (int i = 0; i < particleCandidates.size(); i++) {
            if (particleCandidates[i]->movable || particleCandidates[i]->rigid) {
                Vector3f position = particleCandidates[i]->position;
                double screenCoordinates[3];
                gluProject(position[0], position[1], position[2], modelMatrix, projectionMatrix, viewMatrix,
                           &screenCoordinates[0], &screenCoordinates[1], &screenCoordinates[2]);
                double distance = abs(x - screenCoordinates[0]) + abs(y - (height - screenCoordinates[1]));
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestParticle = particleCandidates[i];
                }
            }
        }
        //update mouseDragParticle
            mouseDragParticle = closestParticle;
            //update the current mousedragforce
            mouseDragForce = new DirectionalForce({mouseDragParticle}, Vector3f(0, 0, 0));
            sys->addForce(mouseDragForce);
    }
}

void View::onMotionEvent( int x, int y )
{
        this->mx = x;
        this->my = y;
        GLdouble modelMatrix[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
        GLdouble projectionMatrix[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
        GLint viewMatrix[4];
        glGetIntegerv(GL_VIEWPORT, viewMatrix);
        double screenCoordinates[3];
        Particle *midParticle = sys->particles[sys->particles.size() / 2];
        gluProject(midParticle->position[0], midParticle->position[1], midParticle->position[2], modelMatrix,
                   projectionMatrix, viewMatrix,
                   &screenCoordinates[0], &screenCoordinates[1], &screenCoordinates[2]);
        float z = screenCoordinates[2];
        double objCoordinates[3];
        gluUnProject(x, height - y, z, modelMatrix, projectionMatrix, viewMatrix, &objCoordinates[0],
                     &objCoordinates[1], &objCoordinates[2]);
        Vector3f position = mouseDragParticle->position;
        mouseDragForce->direction =  80.f * Vector3f((objCoordinates[0] - position[0]), (objCoordinates[1] - position[1]),
                                   (objCoordinates[2] - position[2]));
}

void View::onReshape(int width, int height )
{
    if (height == 0) height = 1;
    GLfloat aspect = (GLfloat)width / (GLfloat)height;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glTranslatef(0.0f, .5f, .0f);

    gluPerspective(90.0f, aspect, 0.1f, 100.0f);

    this->width = width;
    this->height = height;
}

void View::onIdle()
{
    if ( isSimulating ) sys->step(adaptive);
    else {getFromGUI();remapGUI();}

    glutSetWindow ( id );
    glutPostRedisplay ();
}

void View::onDisplay()
{
    frame++;

    //get the current time
    currenttime = glutGet(GLUT_ELAPSED_TIME);
    char title[30];

    //check if a second has passed
    if (currenttime - timebase > 1000)
    {
        sprintf(title, "Fluids! (FPS: %4.2f)", frame*1000.0/(currenttime-timebase));
        glutSetWindowTitle(title);
        timebase = currenttime;
        frame = 0;
    }

    preDisplay3D ();

    if (sys != NULL)
        sys->draw(drawVelocity, drawForces, drawConstraints, drawMarchingCubes);

    postDisplay ();
}


void View::preDisplay3D()
{
    glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glMatrixMode ( GL_MODELVIEW );
    glLoadIdentity ();
    glTranslatef(0.0f, .2f, -2.5f);
//    glRotatef(90, 1.0f, 0.0f, 0.0f);
    glRotatef(camAngle, 0.0f, 1.0f, 0.0f);

    if (rotate != 0) {
        camAngle += rotate * 0.5f;
    }
}

void View::postDisplay()
{
    glutSwapBuffers ();
}

void View::getFromGUI() {
    int i, j;
    // int size, flag;
    int hi, hj;
    // float x, y;
    if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0]
         && !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

    i = (int)((       mx /(float)width)*N);
    j = (int)(((height-my)/(float)height)*N);

    if ( i<1 || i>N || j<1 || j>N ) return;

    if ( mouse_down[0] ) {

    }

    if ( mouse_down[2] ) {
    }

    hi = (int)((       hmx /(float)width)*N);
    hj = (int)(((height-hmy)/(float)height)*N);

    if( mouse_release[0] ) {
    }

    omx = mx;
    omy = my;
}



void View::remapGUI()
{
	for(int i=0; i < sys->particles.size(); i++)
	{
        sys->particles[i]->position[0] = sys->particles[i]->startPos[0];
        sys->particles[i]->position[1] = sys->particles[i]->startPos[1];
        sys->particles[i]->position[2] = sys->particles[i]->startPos[2];
	}
}