//
// Created by Ruud Andriessen on 10/05/2017.
//

#ifndef FLUIDS_VIEW_H
#define FLUIDS_VIEW_H

#include "System.h"
#include "SystemBuilder.h"
#include "forces/DirectionalForce.h"

class View {
public:
    View(int width, int height, float dt, int N);

    ~View() { delete sys; };

    // Event handlers
    void onDisplay();
    void onIdle(void);
    void onMotionEvent(int x, int y);
    void onReshape(int width, int height);
    void onMouseEvent(int button, int state, int x, int y);
    void onKeyPress(unsigned char key, int x, int y);

    void initialize(SystemBuilder::AvailableSystems type);

private:
    System* sys = NULL;
    DirectionalForce* wind;
    DirectionalForce* mouseDragForce;
    Particle* mouseDragParticle;
    int id;
    int mx, my, omx, omy, hmx, hmy;
    int initialMx, initialMy;
    int width, height;
    bool dumpFrames;
    bool drawVelocity, drawConstraints, drawForces, drawMarchingCubes;
    bool isSimulating, adaptive;
    int rotate = 0;
    int N;
    int frameNumber;
    float dt;
    float camAngle = 0;
    int frame;
    float currenttime, timebase;
    int mouse_down[3];
    int mouse_release[3];
    int mouse_shiftclick[3];

    void getFromGUI();
    void remapGUI();

    // Display utility
    void preDisplay3D();
    void postDisplay();
};

#endif //FLUIDS_VIEW_H
