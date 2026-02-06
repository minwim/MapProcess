#ifndef ROBOT_H
#define ROBOT_H

class Robot 
{
public:
    Robot(int start_x, int start_y);

    void moveTo(int new_x, int new_y);

    int getX() const;
    int getY() const;

private:
    int x_, y_;
};

#endif // ROBOT_H
