#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H
#include <QObject>

class RobotControl : public QObject
{
    Q_OBJECT

public:

    RobotControl(QObject *parent = 0);
    ~RobotControl(){}

    void init();
    void sense();
    void act();

signals:
    void messageOut(QString);

};

#endif
