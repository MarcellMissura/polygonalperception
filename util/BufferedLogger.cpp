#include "BufferedLogger.h"

// WARNING, this class has not been tested yet.

BufferedLogger::BufferedLogger(QString rl, bool append)
{
	file.setFileName(rl);
	file.open(append ? QFile::Append : QFile::WriteOnly | QFile::Text);

    buffer.open(QFile::WriteOnly | QFile::Text);
    stream.setDevice(&buffer);
}

BufferedLogger::~BufferedLogger()
{
    writeToFile();
    file.close();
}

// Flushes the buffer to the file.
// After this operation the buffer will be empty and its contents will be appended to the file.
void BufferedLogger::writeToFile()
{
    stream << "\n";
    stream.flush();
    file.write(buffer.buffer());
}

BufferedLogger& BufferedLogger::operator<<(const Vec2 v)
{
	stream << " " << v.x << " " << v.y;
	return *this;
}

BufferedLogger& BufferedLogger::operator<<(const Vec3 v)
{
	stream << " " << v.x << " " << v.y << " " << v.z;
	return *this;
}

BufferedLogger& BufferedLogger::operator<<(const QString s)
{
	stream << " " << s;
	return *this;
}

BufferedLogger& BufferedLogger::operator<<(const double d)
{
	stream << " " << d;
	return *this;
}

BufferedLogger& BufferedLogger::operator<<(const float f)
{
	stream << " " << f;
	return *this;
}

BufferedLogger& BufferedLogger::operator<<(const int i)
{
	stream << " " << i;
	return *this;
}

BufferedLogger& BufferedLogger::operator<<(const char c)
{
	stream << " " << c;
	return *this;
}

BufferedLogger& BufferedLogger::operator++()
{
	stream << "\n";
	return *this;
}

BufferedLogger& BufferedLogger::operator++(int c)
{
	stream << "\n";
	return *this;
}


