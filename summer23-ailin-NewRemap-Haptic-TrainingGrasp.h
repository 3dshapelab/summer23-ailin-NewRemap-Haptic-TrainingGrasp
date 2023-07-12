#include "targetver.h"

#include <cstdlib>
#include <cmath>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <queue>
#include <string>


/**** BOOOST MULTITHREADED LIBRARY *********/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"


#ifdef _WIN32
#include <windows.h>
#include "CShaderProgram.h" // shader program
//#include "glext.h" // opengl extensions for multisampling
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include <MMSystem.h>
#endif

/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "StimulusDrawer.h"
#include "GLText.h"

#include "ParametersLoader.h"
#include "Util.h"
#include "VRCamera.h"
#include "BalanceFactor.h"
#include "ParStaircase.h"
#include "Staircase.h"
#include "TrialGenerator.h"
#include "BrownPhidgets.h"
#include <direct.h>
#include "Optotrak2.h"
#include "Marker.h"
#include "BrownMotorFunctions.h"

#include <SOIL.h>//Library for texture mapping

/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownMotorFunctions;
using namespace BrownPhidgets;

/*************** Variable Declarations ********************/
static const bool gameMode = true;
const float DEG2RAD = M_PI / 180;

/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
Optotrak2 optotrak;
Screen screen;
CoordinatesExtractor headEyeCoords, thumbCoords, indexCoords, thumbJointCoords, indexJointCoords;

/***** CALIBRATION FILE *****/
#include "Calibration_017B.h"
static const Vector3d center(0, 0, focalDistance);
double mirrorAlignment = 0.0, screenAlignmentY = 0.0, screenAlignmentZ = 0.0;

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
double interoculardistance = 60.0;
int screen1 = 19, screen2 = 20, screen3 = 21;
int mirror1 = 6, mirror2 = 22;

/********** FINGERS AND MARKERS **********************/
// finger markers
int ind1 = 13, ind2 = 14, ind3 = 16;
int thu1 = 15, thu2 = 17, thu3 = 18;
int calibration_T = 1, calibration_I = 2;

// Position variables for keeping track of finger movementm
Vector3d ind, indJoint, thm, thmJoint;
Vector3d homePos(0, 0, 0), thmTarget(0,0,0), visTarget(0,0,0), centeredObjEdge(0,0,0);
Vector3d vec_ind_thm, thmToHome, thmToTarget;
Vector3d indexCalibrationPoint(0, 0, 0), thumbCalibrationPoint(0, 0, 0), CalibrationPoint(0, 0, 0);
Vector3d indexCalibration_offset(-8, 0, -5.4), thumbCalibration_offset(-8, 0, 5.4);

bool allVisibleIndex = false;
bool allVisibleThumb = false;
bool allVisibleFingers = false;
int fingersOccluded = 0;

// thumb to home
double old_dist_thm_home, dist_thm_home = 0;
double vel_dist_home;
// threshold
double thresholdDist_near_home = 70;
bool handNearHome = false;
int holdCount_home = 0;
int threshHoldCout_home = 50;

// thumb to target
double old_dist_thm_target, dist_thm_target = 0;
double vel_dist_target;
// threshold
double thresholdDist_near_target = 100, thresholdDist_on_target = 16, thresholdVelthm_steady = 1.6;
bool handNearObject = false;
bool handOnObject = false;
bool handSteady = false;
int holdCount_target = 0;
int threshHoldCout_target = 50;

// GA
double old_grip_aperture, grip_aperture = 0;
double vel_grip_change;
// threshold
double thresholdGA_small = 10, thresholdVelGA_steady = 1.2;
bool gripSmall = false;
bool gripSteady = false;

// MSE
double grip_aperture_MSE = 0;
bool attemped_MSE = false;



/********** TRIAL SPECIFIC PARAMETERS ***************/
ParametersLoader parameters;
ParametersLoader parameters_extra;
BalanceFactor<double> trial; //if using costant stimuli
//TrialGenerator<double> trial;//if using staircase: 


/*************************** INPUT AND OUTPUT ****************************/
// experiment directory
string experiment_directory = "C:/Users/labdomin/Documents/data/ailin/summer22-ailin-hapticRemap-Grasping";

// paramters file directory and name
ifstream parametersFile;
string parametersFileName = experiment_directory + "/parameters_summer22-ailin-hapticRemap-Grasping.txt";
ifstream parametersFile_extra;
string parametersFileName_extra = experiment_directory + "/parameters_summer22-ailin-hapticRemap-Grasping-extra.txt";

// response file
ofstream responseFile;
string responseFile_headers = "subjName\tIOD\tblockN\ttrialN\tdisplayDistance\tvisualAngle\tshapeHeight\ttexnum\tnomralizer\tDepth\tDepthDelta\tDepth_text\tDepth_disp\treinforceTexture\tDepth_haptic\tMSE1\tRT_MSE1\tRT_G\tLAcorrection\tcalibNum";

string subjectName;

/**********	TRIALS **************/
int sessionNum = 0;
bool session_full_vs_extra = true;

int totalBlkNum = 1;
int blkNum = 1;
int trialNum = 0;
int trialNum_max = 1000;

int trainNum_cap = 20;

double percentComplete = 0;
int repetition = 4;
int totalTrNum = 6 * 8 * repetition;

int occlusionFrames_MSE = 0;
int occlusionFrames_grasp = 0;
/****** PHIDGETS VARIABLES *******/
int obj_marker = 11;
int velmex_marker = 4;

int Z_AXIS = 0;
int kit_plugIn_ID = 0; // which slot on the interface kit 888 did you plug in?


// linAct No1
//Vector3d centeredObjEdge_velmexM_diff(-89.6, 193.8, 112);
//Vector3d centeredObjEdge_objM_diff(10, -42.0, -18); 
//double grasping_phidget_depthDiff_flat = 18.7, grasping_phidget_depthDiff_medium = 19.6, grasping_phidget_depthDiff_deep = 20.3; 
//double obj_y_offset_flat = -24, obj_y_offset_medium = 0, obj_y_offset_deep = 24;

// linAct No2
Vector3d centeredObjEdge_velmexM_diff(-90.5, 212.3, 103.6);
Vector3d centeredObjEdge_objM_diff(30.3, -28.3, -27.7); 
double grasping_phidget_depthDiff_flat = 18.0, grasping_phidget_depthDiff_medium = 20.4, grasping_phidget_depthDiff_deep = 23.3; 
double obj_y_offset_flat = -30, obj_y_offset_medium = 30, obj_y_offset_deep = 0;
double est_haptic_depth;

double grasping_phidget_depthDiff = grasping_phidget_depthDiff_medium;
double centeredObj_y_now, centeredObj_y_aiming;

double net_phidget_distance = 0, min_phidge_distance = 1.0;
int phidget_position_read;
double init_phidget_distance = 6.0; //0;
double linAct_measured_distance;
double prev_phidget_ditance = init_phidget_distance;

double linAct_offset_sm = 0.8, linAct_offset_lg = 1.2;
/********** STIMULUS SHAPE ***************/
// stimulus shape
double display_distance;
double visual_angle = 8.0; // stim diangonal size

//height and width
double stimulus_height = 70; //tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double stimulus_width = 70; //ratio_bgwidth_height * stimulus_height;

double ratio_bgwidth_height = 1.4;//1.3;
double stimulus_bgwidth = 70;
double ratio_visiblewidth_height = 1.2;//1.1;
double stimulus_visiblewidth = 70; //ratio_visiblewidth_height * stimulus_height;

// depths of visual stimuli
double depth_mean = 40;
double depth_delta = 0;
double depth_text = 40;
double depth_disp = 40;

// depths of haptic stimuli
double depth_haptic = 40.0; 
double depth_thresh_flat = 24, depth_thresh_deep = 36;
enum hapticBumps{flatBump, mediumBump, deepBump};
hapticBumps bump_current = mediumBump;

// training
double depth_training_min = 20; // set in the subject file
int depth_training_range = 24; // set in the subject file
double depth_inc = 2; 
// jitter in distance
double jitter_z = 0;
double display_distance_jittered = display_distance + jitter_z;
double visualTarget_X = 0, visualTarget_Y = 0;

// shapes
enum shapeTypes { Ridge, Gaussian, Cosine, CosineRidge };
shapeTypes current_shape = CosineRidge;
int shapeID = 3;
double gauss_sig_height_ratio = 0.16;

/********** BLOCKING PANELS ***************/
enum panelStates{no_aperture, black_aperture, red_aperture};
panelStates panel_state = black_aperture;

std::vector<Vector3d> vertContainer_Rcontour;
std::vector<Vector3d> vertContainer_Lcontour;

/********** STIMULUS VERTICES ***************/
int nr_points = 201;
// vectors storing vertices data
std::vector<GLfloat> vertices_vec;
std::vector<GLfloat> texcoors_vec;
std::vector<GLfloat> colors_vec;
std::vector<GLfloat> normals_vec;
std::vector<GLuint> indices_draw_triangle_vec;

// texture map
GLuint loaded_textures[51];
int texnum = 10;
double normalizer_to_uv_base = 90;
double normalizer_to_uv = 90;

double u_offset = 0.05;
double v_offset = 0.05;

// light setting
float max_intensity = 0.8;
float amb_intensity = 0.3;
float lightDir_z = 0.6;

/********** STATE VARIABLES ***************/
enum Stages {exp_initializing, stimulus_preview, prep_trial, trial_fixate, trial_visualStimulus, trial_MSE, 
trial_waitToGrasp, trial_grasp, trial_handReturn, break_time, exp_completed};
Stages current_stage = exp_initializing; // if just want to look at the stimuli, use the constant present stage

enum expInitSteps{to_GetCalibrationPoints, to_CalibrateFingerTips, to_CalibrateFingerJoints,  to_MarkHomePos, to_MoveApparatus, to_confirmReady};
expInitSteps currentInitStep = to_GetCalibrationPoints;
int calibrationNum = 0;

bool resetScreen_betweenRuns = true;
bool reinforce_texture_disparity = true;
bool training = true;
bool visibleInfo = true;
bool fingerCalibration_TIPandJOINT = false; // if false, calibration two points - tip cand joint
bool Fingers_Calibrated = false;
bool Exp_Initialized = false;
bool screenBack = false; //has the screen been placed at projection distance

bool task_guide_info = true;

enum errorStates{no_error, training_exceeded, blocked_objMarkers, linAct_error, velmex_error};
errorStates current_error_state = no_error;
int linActRedFlag = 0;
int velmexRedFlag = 0;
int liinAct_correction = 0;
/********** TIME ***************/
// Timer variable, set for each trial 
Timer trial_timer;
double ElapsedTime;
double timestamp_MSEstart, timestamp_MSEend;
double timestamp_graspstart, timestamp_graspend;

double fixateTime = 600;

/*********** for DEBUGGING **********/
bool known_edge_objM_diff = true; // use a marker on the center edge to set the edge, then get the diff between edge and obj marker
double edgeM_x_offset = -3, edgeM_z_offset = 0;

bool testHapticDevice =  false; //true;
int edge_marker = 8;
void draw_objEdge();

float time_var = 200;

double obj_y_diff = 0;


/*************************** FUNCTIONS ***********************************/
void initOptotrak();
void initMotors();
void initRendering();
void initVariables();
void initStreams();
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d& _transformation = Affine3d::Identity(), bool synchronous = true);
void updateTheMarkers();
void online_apparatus_alignment();
void cleanup();
void beepOk(int tone);

void initBlock();
void initTrial();
void onlineTrial();
void advanceTrial();

void initPreviewStimulus(double textDepth, double dispDepth);
void drawStimulus();
void drawInfo();

double NewtonSolver_Cosine(double theHeight, double newDeth_zDenom, double constCos, double l_translatedDist, double y0, double z0);
void buildVertices_congruent(double shapeDepth, double textNormalizer);
void buildVertices_incongruent(double textDepth, double dispDepth, double displayDist, double textNormalizer);
void drawVertices(int texNum, double displayDist, double dispDepth);

void drawProgressBar();
void drawBlockingPanels(double pandelSeparation);
void drawFixation(double displayDist);

void drawPanels(double displayDist, double dispDepth);
int LoadGLTextures();
float adjustAmbient(double textDepth, float maxInt, double rateAmbvsDiff_flat, double rateAmbvsDiff_deep, double Depth_flat, double Depth_deep);

void calibrate_system();
void calibrate_fingers();
void draw_fingers_dots();
void draw_thumb_dots();
void online_fingers();
void drawMarker(int Marker_ID);
void drawThumbTarget();
void drawInfo_alignment(GLText curText);
void drawInfo_fingers(GLText curText);
void drawInfo_calibrationMarkers(GLText curText);
void drawTaskGuide();

void change_haptic_object(double hapticDepth);
void check_haptic();
double find_centerObj_y();
int check_linAct(double offsetAllowed);
int check_velmex();