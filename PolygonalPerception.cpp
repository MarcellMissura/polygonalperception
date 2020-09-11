#include "PolygonalPerception.h"
#include <QMainWindow>
#include <QMenuBar>

PolygonalPerception::PolygonalPerception(QWidget *parent)
    : QMainWindow(parent)
{
    QCoreApplication::setOrganizationName("MarcellMissura");
    QCoreApplication::setApplicationName("PolygonalPerception");

	QWidget* cw = new QWidget();
	ui.setupUi(cw);
	setCentralWidget(cw); // Because it's a QMainWindow.

	// Initialize the state and the config descriptors.
	state.init();
	config.init();
    config.load();

    // Initialize the widgets and the world.
    robotControlLoop.init();
	graphWidget.init();
	checkboxWidget.init();
	configWidget.init();
    cameraViewWidget.init();

	// Build the splitter separated gui components.
	verticalSplitterTop = new QSplitter();
	verticalSplitterTop->addWidget(&configWidget);
    verticalSplitterTop->addWidget(&openGLWidget);
	verticalSplitterBottom = new QSplitter();
	verticalSplitterBottom->addWidget(&checkboxWidget);
    verticalSplitterBottom->addWidget(&graphWidget);
    verticalSplitterBottom->addWidget(&cameraViewWidget);

	horizontalSplitter = new QSplitter(Qt::Vertical);
	horizontalSplitter->addWidget(verticalSplitterTop);
	horizontalSplitter->addWidget(verticalSplitterBottom);

	QSettings settings;
	verticalSplitterTop->restoreState(settings.value("verticalSplitterTop").toByteArray());
	verticalSplitterBottom->restoreState(settings.value("verticalSplitterBottom").toByteArray());
	horizontalSplitter->restoreState(settings.value("horizontalSplitter").toByteArray());

	QHBoxLayout* centralLayout = new QHBoxLayout(ui.centralWidget);
	centralLayout->setMargin(0);
	ui.centralWidget->setLayout(centralLayout);
	centralLayout->addWidget(horizontalSplitter);

	connect(verticalSplitterTop, SIGNAL(splitterMoved(int, int)), this, SLOT(topSplitterMoved()));
	connect(verticalSplitterBottom, SIGNAL(splitterMoved(int, int)), this, SLOT(bottomSplitterMoved()));

	// Build the menu bar.
	QMenuBar* menuBar = new QMenuBar();
	setMenuBar(menuBar);

    QMenu* fileMenu = menuBar->addMenu(tr("&File"));

	QAction* saveStateAction = fileMenu->addAction(tr("&Save State"));
	saveStateAction->setToolTip(tr("Saves the state history."));
    saveStateAction->setShortcut(QKeySequence(tr("Ctrl+S")));
	connect(saveStateAction, SIGNAL(triggered()), this, SLOT(saveStateHistory()));

	QAction* loadStateAction = fileMenu->addAction(tr("&Load State"));
	loadStateAction->setToolTip(tr("Loads the state history."));
    loadStateAction->setShortcut(QKeySequence(tr("Ctrl+L")));
	connect(loadStateAction, SIGNAL(triggered()), this, SLOT(loadStateHistory()));

    QAction* clearStateAction = fileMenu->addAction(tr("&Clear State"));
    clearStateAction->setToolTip(tr("Clears the state history."));
    //clearStateAction->setShortcut(QKeySequence(tr("Ctrl+L")));
    connect(clearStateAction, SIGNAL(triggered()), this, SLOT(clearStateHistory()));

	fileMenu->addSeparator();

	QAction* saveConfigAction = fileMenu->addAction(tr("&Save Config"));
	saveConfigAction->setToolTip(tr("Saves the config."));
	saveConfigAction->setShortcut(QKeySequence(tr("Ctrl+C")));
	connect(saveConfigAction, SIGNAL(triggered()), this, SLOT(saveConfig()));

	QAction* loadConfigAction = fileMenu->addAction(tr("&Load Config"));
	loadConfigAction->setToolTip(tr("Loads the config."));
	loadConfigAction->setShortcut(QKeySequence(tr("Ctrl+R")));
	connect(loadConfigAction, SIGNAL(triggered()), this, SLOT(loadConfig()));

    fileMenu->addSeparator();

    QAction* exportAction = fileMenu->addAction(tr("&Export shown data"));
    exportAction->setToolTip(tr("Exports the data currently visible in the graph widget."));
    exportAction->setShortcut(QKeySequence(tr("Ctrl+Shift+E")));
    connect(exportAction, SIGNAL(triggered()), &graphWidget, SLOT(exportData()));

    QMenu* viewMenu = menuBar->addMenu(tr("&View"));

    QAction* configViewAction = viewMenu->addAction(tr("&Config"));
    configViewAction->setToolTip(tr("Toggles the config widget."));
    configViewAction->setShortcut(QKeySequence(tr("C")));
    configViewAction->setCheckable(true);
    configViewAction->setChecked(false);
    connect(configViewAction, SIGNAL(triggered()), this, SLOT(toggleConfig()));

    QAction* graphViewAction = viewMenu->addAction(tr("&Graph"));
    graphViewAction->setToolTip(tr("Toggles the graph widget."));
    graphViewAction->setShortcut(QKeySequence(tr("G")));
    graphViewAction->setCheckable(true);
    graphViewAction->setChecked(false);
    connect(graphViewAction, SIGNAL(triggered()), this, SLOT(toggleGraph()));

    viewMenu->addSeparator();

    QAction* showAxisAction = viewMenu->addAction(tr("&Axis"));
    showAxisAction->setToolTip(tr("Toggles the axis."));
    showAxisAction->setShortcut(QKeySequence(tr("A")));
    showAxisAction->setCheckable(true);
    showAxisAction->setChecked(openGLWidget.axisIsDrawn());
    connect(showAxisAction, SIGNAL(triggered()), &openGLWidget, SLOT(toggleAxis()));

    QAction* showFloorAction = viewMenu->addAction(tr("&Floor"));
    showFloorAction->setToolTip(tr("Shows an imaginary floor."));
    showFloorAction->setShortcut(QKeySequence(tr("F")));
    showFloorAction->setCheckable(true);
    showFloorAction->setChecked(openGLWidget.showFloor);
    connect(showFloorAction, SIGNAL(triggered()), &openGLWidget, SLOT(toggleFloor()));

    viewMenu->addSeparator();

    QAction* showPointCloudAction = viewMenu->addAction(tr("&Point Cloud"));
    showPointCloudAction->setToolTip(tr("Toggles the point cloud view."));
    showPointCloudAction->setShortcut(QKeySequence(tr("P")));
    showPointCloudAction->setCheckable(true);
    showPointCloudAction->setChecked(openGLWidget.showPointCloud);
    connect(showPointCloudAction, SIGNAL(triggered()), &openGLWidget, SLOT(togglePointCloud()));

    QAction* showDiscardedAction = viewMenu->addAction(tr("&Discarded Points"));
    showDiscardedAction->setToolTip(tr("Toggles removal of floor and ceiling."));
    showDiscardedAction->setShortcut(QKeySequence(tr("D")));
    showDiscardedAction->setCheckable(true);
    showDiscardedAction->setChecked(openGLWidget.showDiscardedPoints);
    connect(showDiscardedAction, SIGNAL(triggered()), &openGLWidget, SLOT(toggleDiscardedPoints()));

    QAction* showHeightMapAction = viewMenu->addAction(tr("&Height Map"));
    showHeightMapAction->setToolTip(tr("Toggles the grid view."));
    showHeightMapAction->setShortcut(QKeySequence(tr("H")));
    showHeightMapAction->setCheckable(true);
    showHeightMapAction->setChecked(openGLWidget.showOccupancyMap);
    connect(showHeightMapAction, SIGNAL(triggered()), &openGLWidget, SLOT(toggleOccupancyMap()));

    QAction* showPolygonsAction = viewMenu->addAction(tr("&Polygons"));
    showPolygonsAction->setToolTip(tr("Toggles the polygons view."));
    showPolygonsAction->setShortcut(QKeySequence(tr("CTRL+P")));
    showPolygonsAction->setCheckable(true);
    showPolygonsAction->setChecked(openGLWidget.showPolygons);
    connect(showPolygonsAction, SIGNAL(triggered()), &openGLWidget, SLOT(togglePolygons()));
    connect(showPolygonsAction, SIGNAL(triggered()), &cameraViewWidget, SLOT(togglePolygons()));

    QAction* showCameraTransformAction = viewMenu->addAction(tr("&Camera Transform"));
    showCameraTransformAction->setToolTip(tr("Toggles the camera transform."));
    showCameraTransformAction->setShortcut(QKeySequence(tr("M")));
    showCameraTransformAction->setCheckable(true);
    showCameraTransformAction->setChecked(openGLWidget.showCameraTransform);
    connect(showCameraTransformAction, SIGNAL(triggered()), &openGLWidget, SLOT(toggleCameraTransform()));

    QAction* showFloorTransformAction = viewMenu->addAction(tr("&Floor Detection"));
    showFloorTransformAction->setToolTip(tr("Toggles the floor finder visuals."));
    showFloorTransformAction->setShortcut(QKeySequence(tr("CTRL+F")));
    showFloorTransformAction->setCheckable(true);
    showFloorTransformAction->setChecked(openGLWidget.showFloorDetection);
    connect(showFloorTransformAction, SIGNAL(triggered()), &openGLWidget, SLOT(toggleFloorDetection()));
    connect(showFloorTransformAction, SIGNAL(triggered()), &cameraViewWidget, SLOT(toggleFloorDetection()));

    viewMenu->addSeparator();

    QAction* resampleColorsAction = viewMenu->addAction(tr("&Resample Colors"));
    resampleColorsAction->setToolTip(tr("Resamples the colors of the curves."));
    connect(resampleColorsAction, SIGNAL(triggered()), &graphWidget, SLOT(resampleColors()));

	menuBar->addSeparator();
	QAction* fakeSeparator1 = menuBar->addAction("     ");
	fakeSeparator1->setEnabled(false);

    recordAction = menuBar->addAction(tr("Record"));
    recordAction->setCheckable(true);
    recordAction->setChecked(true);
    recordAction->setEnabled(true);
    recordAction->setToolTip(tr("Toggles the robot control loop."));
    recordAction->setShortcut(QKeySequence(tr("Return")));
    connect(recordAction, SIGNAL(triggered()), this, SLOT(record()));

	
	QAction* resetAction = menuBar->addAction(tr("&Reset"));
	resetAction->setToolTip(tr("Resets the simulation state."));
	resetAction->setShortcut(QKeySequence(tr("R")));
	connect(resetAction, SIGNAL(triggered()), this, SLOT(reset()));

	menuBar->addSeparator();
    QAction* fakeSeparator3 = menuBar->addAction("     ");
    fakeSeparator3->setEnabled(false);

	QAction* jumpToStartAction = menuBar->addAction(tr("|<"));
	jumpToStartAction->setToolTip(tr("Sets the player to the first frame."));
	jumpToStartAction->setShortcut(QKeySequence(tr("Backspace")));
	connect(jumpToStartAction, SIGNAL(triggered()), this, SLOT(jumpToStart()));

	QAction* frameBackAction = menuBar->addAction(tr("<"));
	frameBackAction->setToolTip(tr("Rewinds the player by one frame."));
	//frameBackAction->setShortcut(QKeySequence(tr("Backspace")));
	connect(frameBackAction, SIGNAL(triggered()), this, SLOT(frameBack()));

	QAction* playAction = menuBar->addAction(tr("Play"));
	playAction->setToolTip(tr("Starts the playback."));
	playAction->setShortcut(QKeySequence(tr("Space")));
	connect(playAction, SIGNAL(triggered()), this, SLOT(play()));

	//QAction* stopAction = menuBar->addAction(tr("Stop"));
	//stopAction->setToolTip(tr("Stops the playback."));
	//stopAction->setShortcut(QKeySequence(tr("Space")));
	//connect(stopAction, SIGNAL(triggered()), this, SLOT(stop()));

	QAction* frameForwardAction = menuBar->addAction(tr(">"));
	frameForwardAction->setToolTip(tr("Advances the player by one frame."));
    //frameForwardAction->setShortcut(QKeySequence(tr("Backspace")));
	connect(frameForwardAction, SIGNAL(triggered()), this, SLOT(frameForward()));

	QAction* jumpToEndAction = menuBar->addAction(tr(">|"));
	jumpToEndAction->setToolTip(tr("Sets the player to the last frame."));
    jumpToEndAction->setShortcut(QKeySequence(tr("CTRL+Backspace")));
    connect(jumpToEndAction, SIGNAL(triggered()), this, SLOT(jumpToEnd()));


    connect(this, SIGNAL(progressOut(int)), ui.frameSlider, SLOT(setValue(int)));
	connect(ui.frameSlider, SIGNAL(sliderMoved(int)), this, SLOT(jumpToFrame(int)));
//	connect(ui.frameSlider, SIGNAL(sliderReleased()), this, SLOT(play()));

    connect(&configWidget, SIGNAL(configChangedOut()), &openGLWidget, SLOT(update()));
    connect(&checkboxWidget, SIGNAL(stateMemberStatusChanged(int, bool)), &graphWidget, SLOT(setShowCurve(int, bool)));
    connect(&graphWidget, SIGNAL(messageOut(QString)), this, SLOT(messageIn(QString)));

    toggleGraph();
    toggleConfig();
    //showFullScreen();

	// Animation components.
    tscale = 1;
    recording = false;
    cfi = 0;
    animationTimer.setInterval(120);
	connect(&animationTimer, SIGNAL(timeout()), this, SLOT(animate()));
    connect(this, SIGNAL(frameIndexChangedOut(int)), &openGLWidget, SLOT(update()));
    connect(this, SIGNAL(frameIndexChangedOut(int)), &graphWidget, SLOT(frameIndexChangedIn(int)));
    connect(this, SIGNAL(frameIndexChangedOut(int)), &cameraViewWidget, SLOT(frameIndexChangedIn(int)));
    //animationTimer.start();

    //record();
    loadStateHistory();
}

PolygonalPerception::~PolygonalPerception()
{
    QSettings settings;
    settings.setValue("verticalSplitterTop", verticalSplitterTop->saveState());
    settings.setValue("verticalSplitterBottom", verticalSplitterBottom->saveState());
    settings.setValue("horizontalSplitter", horizontalSplitter->saveState());
}

// This is needed when the entire config has changed, e.g. when the robot model was
// changed, the config was reset or a new robot was detected.
void PolygonalPerception::configChanged()
{
    configWidget.configChangedIn();
    openGLWidget.update();
}

// Toggles the config widget.
void PolygonalPerception::toggleConfig()
{
    if (configWidget.isHidden())
        configWidget.show();
    else
        configWidget.hide();
}

// This function is called by the animation timer to update the gui.
void PolygonalPerception::animate()
{
    cfi = bound(0, cfi-tscale, state.size()-1);
    if (recording)
        cfi = 0;
    loadFrame(cfi);
}

// Toggles the record mode. In record mode the robot control thread is
// running and data is being buffer into the state history.
void PolygonalPerception::record()
{
    if (recording)
    {
        recording = false;
        openGLWidget.recording = false;
        openGLWidget.update();
        animationTimer.stop();
        robotControlLoop.stop();
    }
    else
    {
        recording = true;
        openGLWidget.recording = true;
        animationTimer.start();
        robotControlLoop.start();
    }
}

// Play button handler.
void PolygonalPerception::play()
{
    if (animationTimer.isActive())
    {
        stop();
    }
    else
    {
        if (!recording)
        {
            tscale = 1;
            animationTimer.start();
        }
    }
}

// Stop button handler.
void PolygonalPerception::stop()
{
    if (!recording)
    {
        animationTimer.stop();
        update();
    }
}

void PolygonalPerception::frameBack()
{
    if (!recording)
    {
        animationTimer.stop();
        cfi = qMin(cfi+1, state.size()-1);
        loadFrame(cfi);
    }
}

void PolygonalPerception::frameForward()
{
    if (!recording)
    {
        animationTimer.stop();
        if (cfi > 0)
            cfi--;
        else
            robotControlLoop.step();
        loadFrame(cfi);
    }
}

void PolygonalPerception::jumpToStart()
{
    if (!recording)
    {
        animationTimer.stop();
        cfi = state.size()-1;
        loadFrame(cfi);
    }
}

// Handles the frame slider.
void PolygonalPerception::jumpToFrame(int f)
{
    if (!recording)
    {
        animationTimer.stop();
        cfi = qMax(0, int(state.size()-1 - (double)((state.size()-1) * f)/1000));
        loadFrame(cfi);
    }
}

void PolygonalPerception::saveConfig()
{
    config.save("config");
    messageIn("Config saved.");
}

void PolygonalPerception::loadConfig()
{
    config.load("config");
    configChanged();
    messageIn("Config reset.");
}

void PolygonalPerception::saveStateHistory()
{
    state.saveHistory();
    messageIn("State history saved.");
}

void PolygonalPerception::clearStateHistory()
{
    state.clear();
}

void PolygonalPerception::loadStateHistory()
{
    if (recording)
        record();
    messageIn("Loading...");
    state.loadHistory(config.bufferSize);
    jumpToStart();
    messageIn("State history loaded.");
}

// Global event filter.
bool PolygonalPerception::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        keyPressEvent(keyEvent);
        return true;
    }
    else if (event->type() == QEvent::KeyRelease)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        keyReleaseEvent(keyEvent);
        return true;
    }
    else
    {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}

// Vertical splitter synchronization.
void PolygonalPerception::topSplitterMoved()
{
	verticalSplitterBottom->setSizes(verticalSplitterTop->sizes());
}
void PolygonalPerception::bottomSplitterMoved()
{
	verticalSplitterTop->setSizes(verticalSplitterBottom->sizes());
}

// Slot for internal message strings.
void PolygonalPerception::messageIn(QString m)
{
    openGLWidget.messageIn(m);
}

// Toggles the graph widget.
void PolygonalPerception::toggleGraph()
{
    if (verticalSplitterBottom->isHidden())
		verticalSplitterBottom->show();
	else
		verticalSplitterBottom->hide();
}

void PolygonalPerception::jumpToEnd()
{
    if (!recording)
    {
        cfi = 0;
        emit progressOut(qMax(0, (int)(1000.0 * ((state.size()-1)-cfi)/(state.size()-1))));
        emit frameIndexChangedOut(cfi);
    }
}

void PolygonalPerception::reset()
{
    robotControlLoop.reset();
	messageIn("Reset");
}

// This slot is called when the user browses the state history with the slider or with
// the frame forward, frame backward, and playback functions.
void PolygonalPerception::loadFrame(int frameIndex)
{
    if (frameIndex > 0)
    {
        // The browsing of the state history is implemented in a generic way such that
        // the current state is overwritten with an older version of the state from
        // state history.
        state.restore(frameIndex);

        // We also step the robot control once to recompute things.
        robotControlLoop.smallStep(frameIndex);
    }

    cfi = frameIndex;
    emit progressOut(qMax(0, (int)(1000.0 * ((state.size()-1)-cfi)/(state.size()-1))));
    emit frameIndexChangedOut(cfi);
}

void PolygonalPerception::toggleFileBuffering()
{
    command.bufferToFile = !command.bufferToFile;
    if (command.bufferToFile)
    {
        messageIn("File buffering is enabled.");
    }
    else
    {
        messageIn("File buffering is disabled.");
    }
}

// Keyboard handling.
void PolygonalPerception::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    if (event->key() == Qt::Key_Backspace && event->modifiers() & Qt::ControlModifier)
    {
        jumpToEnd();
    }
    else if (event->key() == Qt::Key_Backspace)
    {
        jumpToStart();
    }
    else if (event->key() == Qt::Key_Escape)
    {
        close();
    }
    else if (event->key() == Qt::Key_Up)
    {
        if (recording)
        {
            //command.gcv.x = config.V;
        }
        else
        {
            if (!animationTimer.isActive())
                animationTimer.start();
            tscale = 10;
        }
    }
    else if (event->key() == Qt::Key_Down)
    {
        if (recording)
        {
            //command.gcv.x = -config.V;
        }
        else
        {
            if (!animationTimer.isActive())
                animationTimer.start();
            tscale = -10;
        }
    }
    else if (event->key() == Qt::Key_Right && event->modifiers() & Qt::ControlModifier)
    {
        if (recording)
        {
            //command.gcv.z = config.V;
        }
        else
        {
            frameForward();
        }
    }
    else if (event->key() == Qt::Key_Left && event->modifiers() & Qt::ControlModifier)
    {
        if (recording)
        {
            //command.gcv.z = -config.V;
        }
        else
        {
            frameBack();
        }
    }
    else if (event->key() == Qt::Key_Right)
    {
        if (recording)
        {
            //command.gcv.y = config.V;
        }
        else
        {
            frameForward();
        }
    }
    else if (event->key() == Qt::Key_Left)
    {
        if (recording)
        {
            //command.gcv.y = -config.V;
        }
        else
        {
            frameBack();
        }
    }
}

void PolygonalPerception::keyReleaseEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat())
        return;

    if (event->key() == Qt::Key_Up)
    {
        if (recording)
        {
            //command.gcv.x = 0;
        }
        else
        {
            tscale = 1;
            if (!recording)
                animationTimer.stop();
        }
    }
    else if (event->key() == Qt::Key_Down)
    {
        if (recording)
        {
            //command.gcv.x = 0;
        }
        else
        {
            tscale = 1;
            if (!recording)
                animationTimer.stop();
        }
    }
    else if (event->key() == Qt::Key_Right)
    {
        if (recording)
        {
            //command.gcv.y = 0;
            //command.gcv.z = 0;
        }
    }
    else if (event->key() == Qt::Key_Left)
    {
        if (recording)
        {
            //command.gcv.y = 0;
            //command.gcv.z = 0;
        }
    }
}
