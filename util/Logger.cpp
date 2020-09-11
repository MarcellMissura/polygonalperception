#include "Logger.h"

Logger::Logger()
{
    append = false;
}

Logger::Logger(QString rl, bool append)
{
    this->fileName = rl;
    this->append = append;
    open(fileName);
}

Logger::~Logger()
{
    flush();
    file.close();
}

void Logger::open(QString rl)
{
    this->fileName = rl;
    if (file.isOpen())
    {
        flush();
        file.close();
    }
    file.setFileName(fileName);
    file.open(append ? QFile::Append : QFile::WriteOnly | QFile::Text);
    stream.setDevice(&file);
}

void Logger::setAppend(bool append)
{
    this->append = append;
    if (file.isOpen())
        open(fileName);
}

// Clears the contents of the logfile.
void Logger::clear()
{
    file.resize(0);
}

// Flushes unwritten parts of the text stream, if any, into the log file.
void Logger::flush()
{
    stream.flush();
}

QString Logger::getFileName() const
{
    return file.fileName();
}

Logger& Logger::operator<<(const Vec2 v)
{
	stream << " " << v.x << " " << v.y;
	return *this;
}

Logger& Logger::operator<<(const VecN<2> v)
{
    stream << " " << v.x << " " << v.y;
    return *this;
}

Logger& Logger::operator<<(const Vec3 v)
{
	stream << " " << v.x << " " << v.y << " " << v.z;
	return *this;
}

Logger& Logger::operator<<(const VecN<3> v)
{
    stream << " " << v.x << " " << v.y << " " << v.z;
    return *this;
}

Logger& Logger::operator<<(const VecN<4> v)
{
    stream << " " << v.x << " " << v.y << " " << v.z << " " << v.w;
    return *this;
}

Logger& Logger::operator<<(const QString s)
{
	stream << " " << s;
	return *this;
}

Logger& Logger::operator<<(const double d)
{
	stream << " " << d;
	return *this;
}

Logger& Logger::operator<<(const float f)
{
	stream << " " << f;
	return *this;
}

Logger& Logger::operator<<(const int i)
{
	stream << " " << i;
	return *this;
}

Logger& Logger::operator<<(const uint i)
{
    stream << " " << i;
    return *this;
}

Logger& Logger::operator<<(const char c)
{
	stream << " " << c;
	return *this;
}

Logger& Logger::operator<<(const bool b)
{
    stream << " " << b;
    return *this;
}

Logger& Logger::operator++()
{
	stream << "\n";
    stream.flush();
	return *this;
}

Logger& Logger::operator++(int c)
{
	stream << "\n";
    stream.flush();
	return *this;
}


