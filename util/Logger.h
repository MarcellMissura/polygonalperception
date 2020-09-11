#ifndef LOGGER_H_
#define LOGGER_H_

#include <QFile>
#include <QTextStream>
#include "util/Vec2.h"
#include "util/Vec3.h"
#include "util/VecN.h"

class Logger
{
    QFile file;
	QTextStream stream;
    bool append;
    QString fileName;

public:
    Logger();
    Logger(QString rl, bool append = false);
	~Logger();

    void open(QString rl);
    void setAppend(bool append);
	void clear();
    void flush();

    QString getFileName() const;

    Logger& operator<<(const VecN<4> v);
    Logger& operator<<(const VecN<3> v);
    Logger& operator<<(const VecN<2> v);
    Logger& operator<<(const Vec2 v);
    Logger& operator<<(const Vec3 v);
	Logger& operator<<(const QString s);
	Logger& operator<<(const double d);
	Logger& operator<<(const float f);
	Logger& operator<<(const int i);
    Logger& operator<<(const uint i);
	Logger& operator<<(const char c);
    Logger& operator<<(const bool b);
	Logger& operator++();
	Logger& operator++(int c);
};

#endif /* LOGGER_H_ */
