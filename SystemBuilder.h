//
// Created by Ruud Andriessen on 10/05/2017.
//

#ifndef PARTICLETOY_SYSTEMBUILDER_H
#define PARTICLETOY_SYSTEMBUILDER_H

class System;
class SystemBuilder {
public:
    enum AvailableSystems {
        BASIC,
    };

    static System* get(AvailableSystems s);

private:
    static System* initBasic();
};


#endif //PARTICLETOY_SYSTEMBUILDER_H
