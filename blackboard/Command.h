#ifndef COMMAND_H_
#define COMMAND_H_

#include "util/Vec3.h"
#include <QList>

// The global command object contains user input from the GUI.
struct Command
{
    bool bufferToFile;

	Command();
};

extern Command command;

#endif /* COMMAND_H_ */
