#include "robot.h"

Robot::Robot(int start_x, int start_y) : x_(start_x), y_(start_y) 
{

}

void Robot::moveTo(int new_x, int new_y) 
{
    x_ = new_x;
    y_ = new_y;
}

int Robot::getX() const 
{
    return x_;
}

int Robot::getY() const 
{
    return y_;
}
