#ifndef BUFFEREDLOGGER_H_
#define BUFFEREDLOGGER_H_

#include <QFile>
#include <QTextStream>
#include <QBuffer>
#include "util/Vec2.h"
#include "util/Vec3.h"

// WARNING, this class has not been tested yet.

class BufferedLogger
{
	QFile file;
	QTextStream stream;
    QBuffer buffer;

public:
    BufferedLogger(QString rl, bool append = false);
    ~BufferedLogger();

    void writeToFile();

    BufferedLogger& operator<<(const Vec2 v);
    BufferedLogger& operator<<(const Vec3 v);
    BufferedLogger& operator<<(const QString s);
    BufferedLogger& operator<<(const double d);
    BufferedLogger& operator<<(const float f);
    BufferedLogger& operator<<(const int i);
    BufferedLogger& operator<<(const char c);
    BufferedLogger& operator++();
    BufferedLogger& operator++(int c);
};

#endif /* BUFFEREDLOGGER_H_ */
