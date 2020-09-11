#ifndef FILELOADER_H_
#define FILELOADER_H_
#include <QList>

class FileLoader
{
	QList< QList<double> > data;

public:
    FileLoader();
    FileLoader(QString fileName);
	~FileLoader();

	int lines();
	void load(QString fileName);

	QList<double>& operator[](int i);
    FileLoader& operator<<(const FileLoader &f);
};

#endif /* FILELOADER_H_ */
