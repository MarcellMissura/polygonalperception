#include "Command.h"
#include "globals.h"

// The global command object contains user input from the GUI.
Command command;

Command::Command()
{
    bufferToFile = false;
}

