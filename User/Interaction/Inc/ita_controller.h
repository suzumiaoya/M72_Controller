#ifndef ITA_CONTROLLER_H
#define ITA_CONTROLLER_H

#include "ctl_manipulator.h"
#include "dvc_referee.h"
#include "dvc_lcd.h"

class Class_Controller
{
public:
    Class_Manipulator Left_Arm;
    Class_Manipulator Right_Arm;
    Class_Referee Referee;
    Class_LCD LCD;

    void Init();
};

#endif
