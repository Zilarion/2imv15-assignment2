//
// Created by Ruud Andriessen on 10/05/2017.
//

#ifndef PARTICLETOY_SYSTEMBUILDER_H
#define PARTICLETOY_SYSTEMBUILDER_H

class System;
class SystemBuilder {
public:
    enum AvailableSystems {
        WATER,
        SMOKE,
        CLOTH,
        BASIC,
        GLASS,
        HAIR
    };

    static System* get(AvailableSystems s);

private:
    static System* initBasic();
    static System* initSmoke();
    static System* initTrechter();
    static System* initBasicCloth();
    static System* initCloth();
    static System* initHair();
    static System* initGlass();
};


#endif //PARTICLETOY_SYSTEMBUILDER_H
