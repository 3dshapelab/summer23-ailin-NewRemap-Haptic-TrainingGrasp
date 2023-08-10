#pragma once
// The following macros define the minimum required platform.  The minimum required platform
// is the earliest version of Windows, Internet Explorer etc. that has the necessary features to run 
// your application.  The macros work by enabling all features available on platform versions up to and 
// including the version specified.

// Modify the following defines if you have to target a platform prior to the ones specified below.
// Refer to MSDN for the latest info on corresponding values for different platforms.
#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif



/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "CylinderPointsStimulus.h"
#include "StimulusDrawer.h"
#include "GLText.h"
#include "BalanceFactor.h"
#include "ParametersLoader.h"
#include "Util.h"
#include "VRCamera.h"

#include <direct.h>
#include "Optotrak2.h"
#include "Marker.h"
#include "BrownMotorFunctions.h"
#include "BrownPhidgets.h"

/***** CALIBRATION THINGS *****/
#include "Calibration_017B.h"
#include "matrix_functions.h"

#include <windows.h>
#include <MMSystem.h>


/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownPhidgets;
using namespace BrownMotorFunctions;

const float DEG2RAD = M_PI / 180;

/********* VISUALIZATION VARIABLES *****************/
static const bool gameMode = true;
static const bool stereo = true;

/********* VARIABLES OBJECTS  **********************/
Screen screen;
VRCamera cam;
Optotrak2 optotrak;
CoordinatesExtractor headEyeCoords;
/********** VISUALIZATION AND STIMULI ***************/
double display_distance = -400;

/********** TRIAL SPECIFIC PARAMETERS ***************/
ParametersLoader parameters;
BalanceFactor<double> trial; //if true

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 60;


/****** PHIDGETS VARIABLES *******/
int kit_plugIn_ID = 0; // which slot on the interface kit 888 did you plug in?

double init_phidget_distance = 2; // initial 
double current_phidget_distance = init_phidget_distance; //0.0;
double target_phidget_distance;
double next_phidget_distance = 20.0;
int phidget_position_read;
double movement_offset;

// set phidget movement parameters
double distance_speed_cutoff = 5.0;
double distance_buffer_inc = 0.4;
double distance_buffer_dec = 0.2;

// set parameter for movements with test and readjustment
double moveoffset_thresh_good = 0.6;
double moveoffset_thresh_pass = 1.0;
int max_attempt_good = 1;
int max_attempt_pass = 2;

double haptic_offset = 22.7;

/********* VARIABLES *********/
int totalBlkNum = 2;
int blkNum = 1;
int trialNum = 1;

// Time variables 
Timer trial_timer;
Timer wait_timer;
double waitTime;


double depth_base, depth_larger, depth_delta;

int moveNum = 0;

bool visibleInfo = true;
enum trialStages { exp_initing, trial_prep, trial_respond, breakTime, exp_completed, trial_error };
trialStages current_stage = exp_initing;


bool resetScreen_betweenRuns = false;
bool automatic_testing_mode = true;

/*************************** EXPERIMENT SPECS ****************************/
string experiment_directory = "C:/Users/labdomin/Documents/data/ailin/testData/linActuatorTest4/";
string subjectName;

string parametersFileName = experiment_directory + "parameters_phdigemove.txt";
ifstream parametersFile;

string responseFileName;
ofstream responseFile;
string responseFile_headers = "subjName\tblockN\ttrialN\tmoveID\tdistTwoSpd\tdistBufferINC\tdistBufferDEC\tDepthTo\tendDistance\tmoveOffset\ttime";



/********** FUNCTION PROTOTYPES *****/
void beepOk(int tone);
void drawGLScene();
void handleKeypress(unsigned char key, int x, int y);
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d& _transformation = Affine3d::Identity(), bool synchronous = true);
void update(int value);
void idle();
void initMotors();
void initGLVariables();
void initVariables();
void initStreams();
void initRendering();
void initBlock();
void initTrial();
void onlineTrial();
void drawInfo();
void advanceTrial();
void shutdown();
void initOptotrak();
void updateTheMarkers();

double phidget_move_record(double targetDistance, double cutoffDistance, double bufferDistance_inc, double bufferDistance_dec);
void initTrial_basic();
void initTrial_readjust();
int phidget_move_readjust(double setToDistance, int MaxAttempt_good, int MaxAttempt_pass);


/*************************** FUNCTIONS ***********************************/
void initOptotrak()
{

	optotrak.setTranslation(calibration);

	if (optotrak.init(LastAlignedFile, OPTO_NUM_MARKERS, OPTO_FRAMERATE, OPTO_MARKER_FREQ, OPTO_DUTY_CYCLE, OPTO_VOLTAGE) != 0)
	{
		cerr << "Something during Optotrak initialization failed, press ENTER to continue. A error log has been generated, look \"opto.err\" in this folder" << endl;
		cin.ignore(1E6, '\n');
		exit(0);
	}

	for (int i = 0; i < 10; i++) {
		updateTheMarkers();
	}

}
void updateTheMarkers()
{
	optotrak.updateMarkers();
	markers = optotrak.getAllMarkers();

	for (int i = 1; i <= OPTO_NUM_MARKERS; i++)
	{
		markers.at(i).p = rotationM * markers.at(i).p;
	}

}



// This function seems to be used to shut down the system after use
void cleanup() {

	responseFile.close();
	current_phidget_distance = phidgets_linear_move_custom(init_phidget_distance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
	optotrak.stopCollection();
	if (resetScreen_betweenRuns) {
		homeEverything(5000, 4500);
	}
}

void shutdown() {

	exit(0);
}

void initProjectionScreen(double _focalDist, const Affine3d& _transformation, bool synchronous)
{
	focalDistance = _focalDist;
	screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE * SCREEN_HEIGHT / SCREEN_WIDTH);
	screen.setOffset(alignmentX, alignmentY);
	screen.setFocalDistance(_focalDist);
	screen.transform(_transformation);
	cam.init(screen);
	if (synchronous)
		moveScreenAbsolute(_focalDist, homeFocalDistance, 4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist, homeFocalDistance, 4500);
}

// Initialize motors for moving screen around
void initMotors()
{
	if (resetScreen_betweenRuns)
		homeEverything(5000, 4500);

	current_phidget_distance = phidgets_linear_move_custom(init_phidget_distance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);

}

// Method that initializes the openGL parameters needed for creating the stimuli. 
// seems like this is not changed for each experiment (maybe for different experimental setup eg monitor)
void initRendering()
{
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* Set size buffer clear value */
	glClearDepth(1.0);
	/* Enable depth test */
	glEnable(GL_DEPTH_TEST);
	/* Set size function */
	glDepthFunc(GL_LEQUAL);
	// scommenta solo se vuoi attivare lo shading degli stimoli

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	// Tieni questa riga per evitare che con l'antialiasing attivo le linee siano piu' sottili di un pixel e quindi
	// ballerine (le vedi vibrare)
	glLineWidth(1.5);

}

// Initialize the streams, open the file and write to it
void initStreams()
{
	// Initialize the parameter file starting from the file parameters.txt, if the file does not exist, it tells you


	parametersFile.open(parametersFileName.c_str());
	parameters.loadParameterFile(parametersFile);
	// Subject name
	subjectName = parameters.find("SubjectName");
	responseFileName = experiment_directory + "LinActuator_" + subjectName + ".txt";


	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;

}

//Extracts constants from parameters file. Used in the method defined below
void initVariables()
{

	initProjectionScreen(display_distance);

	// eye coordinates
	eyeRight = Vector3d(interoculardistance / 2, 0, 0);
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);
	eyeMiddle = Vector3d(0, 0, 0);

	if (automatic_testing_mode) {
		initBlock();
	}

}


void drawInfo()
{
	// displays relevant information to the screen
	if (visibleInfo)
	{
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);

		GLText text;

		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

		text.enterTextInputMode();

		switch (current_stage) {
		case exp_initing:
			text.draw("ready? press + to start");
			break;

		case trial_prep:
		case trial_respond:
			if (automatic_testing_mode) {
				text.draw("automatic test ... ...");
			}
			else {
				text.draw("# trial num: " + stringify<int>(trialNum));
				text.draw("# target phidget distance: " + stringify<double>(target_phidget_distance));
				text.draw("# current phidget distance: " + stringify<double>(current_phidget_distance));
				text.draw("#                                 #");
				text.draw("# haptic offset: " + stringify<double>(haptic_offset));
				text.draw("# haptic depth: " + stringify<double>(haptic_offset + current_phidget_distance));

			}

			break;

		case breakTime:
			text.draw("break time");
			break;

		case exp_completed:
			text.draw("Thank you!!!");
			break;

		case trial_error:
			text.draw("linear actuator not reaching target position");
			break;

		}


		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);
	}
}


//Central function for projecting image onto the screen
void drawGLScene()
{
	// Note that in this experiment, there is no difference between drawing left and right because
	// we are displaying to the middle (there is no stereo)
	// Draw left eye view
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam.setEye(eyeLeft);//cam.setEye(eyeMiddle); 
	drawInfo();

	// Draw right eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam.setEye(eyeRight); //cam.setEye(eyeMiddle);
	drawInfo();

	glutSwapBuffers();
}


void initBlock()
{

	trial.init(parameters);
	cout << "initialize parameter file bby" << endl;
	initTrial();

}



double phidget_move_record(double targetDistance, double cutoffDistance, double bufferDistance_inc, double bufferDistance_dec) {

	double distance_aftermove = phidgets_linear_move_custom(targetDistance, cutoffDistance, bufferDistance_inc, bufferDistance_dec);

	responseFile << fixed <<
		subjectName << "\t" <<
		blkNum << "\t" <<
		trialNum << "\t" <<
		moveNum << "\t" <<
		cutoffDistance << "\t" <<
		bufferDistance_inc << "\t" <<
		bufferDistance_dec << "\t" <<
		targetDistance << "\t" <<
		distance_aftermove << "\t" <<
		distance_aftermove - targetDistance << "\t" <<
		trial_timer.getElapsedTimeInMilliSec() << endl;

	return distance_aftermove;
}

void initTrial() {

	wait_timer.stop();

	trial.next();
	current_stage = trial_prep;

	target_phidget_distance = trial.getCurrent()["DepthBase"];

	trial_timer.reset();
	trial_timer.start();

	if (abs(target_phidget_distance - current_phidget_distance) < 3) {
		moveNum = 1;

		// double intermediate_phidget_distance = max(init_phidget_distance, target_phidget_distance - 4);
		double intermediate_phidget_distance =  target_phidget_distance - 3;
		if (intermediate_phidget_distance < init_phidget_distance)
			intermediate_phidget_distance = target_phidget_distance + 3;

		current_phidget_distance = phidget_move_record(intermediate_phidget_distance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
	}
	int phidget_indicator = phidget_move_readjust(target_phidget_distance, max_attempt_good, max_attempt_pass);

	if (phidget_indicator < 1) {
		trial_timer.stop();
		if (automatic_testing_mode) {
			cleanup();
			shutdown();
		}
		else {
			current_stage = trial_error;
		}
	}
	else {
		trial_timer.stop();
		wait_timer.start();
		current_stage = trial_respond;
	}
}





int phidget_move_readjust(double setToDistance, int MaxAttempt_good, int MaxAttempt_pass) {

	int move_complete_indicator = 0;

	moveNum = 1;
	current_phidget_distance = phidget_move_record(setToDistance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
	movement_offset = current_phidget_distance - setToDistance;

	if (abs(movement_offset) <= moveoffset_thresh_good) {
		move_complete_indicator = 1;
	}

	while (move_complete_indicator < 1) {

		moveNum++;

		double use_thresh = moveoffset_thresh_good;
		if (moveNum > MaxAttempt_good) {
			use_thresh = moveoffset_thresh_pass;
		}

		//double intermediate_phidget_distance = max(init_phidget_distance, target_phidget_distance - 4);
		double intermediate_phidget_distance = target_phidget_distance - 3;
		if (intermediate_phidget_distance < init_phidget_distance)
			intermediate_phidget_distance = target_phidget_distance + 3;
		current_phidget_distance = phidget_move_record(intermediate_phidget_distance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
		current_phidget_distance = phidget_move_record(setToDistance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
		movement_offset = current_phidget_distance - setToDistance;


		if (abs(movement_offset) <= use_thresh) {
			move_complete_indicator = 1;
		}

		if (moveNum > (MaxAttempt_good + MaxAttempt_pass)) {
			break;
		}
	}

	return move_complete_indicator;
}






void onlineTrial()
{
	switch (current_stage)
	{
	case exp_initing:
		break;
	case trial_prep:
		break;
	case trial_respond:
	{

		waitTime = wait_timer.getElapsedTimeInMilliSec();
		if (waitTime > 1000) {
			advanceTrial();
		}
		
	}
	break;

	case breakTime:
	{


		waitTime = wait_timer.getElapsedTimeInMilliSec();
		if (waitTime > 1000 * 20) {
			blkNum++;
			initBlock();
		}
		

	}
	break;

	case exp_completed:
		break;

	}
}

void drawStimulus()
{


}

void advanceTrial()
{
	wait_timer.stop();
	wait_timer.reset();

	if (!trial.isEmpty()) {
		trialNum++;

		initTrial();

	}
	else {
		if (blkNum < totalBlkNum) {
			wait_timer.start();
			current_stage = breakTime;
		}
		else {
			if (automatic_testing_mode) {
				cleanup();
				shutdown();
			}
			else {
				current_stage = exp_completed;
			}

		}
	}


}



void update(int value)
{
	glutPostRedisplay();
	glutTimerFunc(TIMER_MS, update, 0);
}



void idle()
{
	onlineTrial();

}



// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

}
// Funzione di callback per gestire pressioni dei tasti
void handleKeypress(unsigned char key, int x, int y)
{
	//cout << "listening for keypress" << endl;
	switch (key)
	{

	case '+':
	{

		if (current_stage == exp_initing) {
			initBlock();
		}

		if (current_stage == trial_respond) {
			advanceTrial();
		}

	}
	break;



	case 'Q':
	case 'q':
	case 27:	//corrisponde al tasto ESC
	{
		// Ricorda quando chiami una funzione exit() di chiamare prima cleanup cosi
		// spegni l'Optotrak ed i markers (altrimenti loro restano accesi e li rovini) 
		// close this object
		cleanup();
		shutdown();
	}
	break;

	case '1':
		haptic_offset = haptic_offset - 0.1;
		break;

	case '2':
		haptic_offset = haptic_offset + 0.1;
		break;

	}
}


// this is run at compilation because it's titled 'main'
int main(int argc, char* argv[])
{
	//functions from cncsvision packages
	mathcommon::randomizeStart();

	// initializes optotrak and velmex motors
	initRotationM();
	initOptotrak();
	initMotors();

	// initializing glut (to use OpenGL)
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);
	glutGameModeString("1024x768:32@85"); //resolution  
	glutEnterGameMode();
	glutFullScreen();

	// initializing experiment's parameters

	initRendering(); // initializes the openGL parameters needed for creating the stimuli

	initStreams(); // streams as in files for writing data

	initVariables();
	// glut callback, OpenGL functions that are infinite loops to constantly run 

	glutDisplayFunc(drawGLScene); // keep drawing the stimuli

	glutKeyboardFunc(handleKeypress); // check for keypress

	glutReshapeFunc(handleResize);

	glutIdleFunc(idle);

	glutTimerFunc(TIMER_MS, update, 0);

	glutSetCursor(GLUT_CURSOR_NONE);

	//boost::thread initVariablesThread(&initVariables); 

	glutMainLoop();

	return 0;
}