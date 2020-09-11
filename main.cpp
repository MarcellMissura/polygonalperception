#include "PolygonalPerception.h"
#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{
    // Instantiate the QApplication and the main window.
    QApplication a(argc, argv);
    PolygonalPerception w;

    // Install the main window as an event filter.
    a.installEventFilter(&w);

    // Apply a stylesheet to the application.
    QFile file("styles.css");
    int ctr = 0;
    while (!file.open(QFile::ReadOnly) && ctr < 3)
    {
        sleep(1);
        ctr++;
    }

    QString styleSheet = QLatin1String(file.readAll());
    w.setStyleSheet(styleSheet);

    // Show the window and start the main event loop.
    w.show();
    return a.exec();
}
