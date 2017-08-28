//
// Created by yyq on 17-4-10.
//

#ifndef STARDRAW_FORKLIFT_OBSTACLE_RADAR_H
#define STARDRAW_FORKLIFT_OBSTACLE_RADAR_H

#include "jetsonGPIO.h"

class Obstacle_radar{
public:
    Obstacle_radar()
    {
        WARN0 = gpio36;
        WARN1 = gpio37;
        DANGER= gpio38;

        Area_set0 = gpio184;
        Area_set1 = gpio187;

        gpioExport(WARN0) ;
        gpioExport(WARN1 ) ;
        gpioExport(DANGER) ;
        gpioExport(Area_set0) ;
        gpioExport(Area_set1) ;

        gpioSetDirection(WARN0 ,inputPin) ;
        gpioSetDirection(WARN1 ,inputPin) ;
        gpioSetDirection(DANGER,inputPin) ;
        gpioSetDirection(Area_set0,outputPin) ;
        gpioSetDirection(Area_set0,outputPin) ;
    }
    void close()
    {
        cout << "Obstacle_radar Closed." << endl;
        gpioUnexport(WARN0);          // unexport the Obstacle_radar WARN0
        gpioUnexport(WARN1);      	  // unexport the Obstacle_radar WARN1
        gpioUnexport(DANGER);      	  // unexport the Obstacle_radar DANGER
        gpioUnexport(Area_set0);      // unexport the Obstacle_radar Area_set0
        gpioUnexport(Area_set1);      // unexport the Obstacle_radar Area_set1
    }
public:
    jetsonTX1GPIONumber WARN0;
    jetsonTX1GPIONumber WARN1;
    jetsonTX1GPIONumber DANGER;

    jetsonTX1GPIONumber Area_set0;
    jetsonTX1GPIONumber Area_set1;
};
#endif //STARDRAW_FORKLIFT_OBSTACLE_RADAR_H
