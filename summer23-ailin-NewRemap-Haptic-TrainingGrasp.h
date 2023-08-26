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
#include "glext.h" // opengl extensions for multisampling
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
Vector3d indexCalibration_offset(-10, 0, -6), thumbCalibration_offset(-10, 0, 6);

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
double old_grip_aperture, grip_aperture = 999;
double vel_grip_change;
// threshold
double thresholdGA_small = 12, thresholdVelGA_steady = 1.2;
bool gripSmall = false;
bool gripSteady = false;

// MSE
double grip_aperture_MSE = 0;
bool attemped_MSE = false;

// occlusion
int occlusionFrames_MSE = 0;
int occlusionFrames_grasp = 0;


/****** PHIDGETS VARIABLES *******/
int kit_plugIn_ID = 0; // which slot on the interface kit 888 did you plug in?
// set phidget movement parameters
double distance_speed_cutoff = 5.0;
double distance_buffer_inc = 0.4;
double distance_buffer_dec = 0.2;

double init_phidget_distance = 2; // initial 
double current_phidget_distance = init_phidget_distance; //0.0;
double target_phidget_distance;
int phidget_position_read;
double movement_offset;
// set parameter for movements with test and readjustment
double moveoffset_thresh_good = 0.6;
double moveoffset_thresh_pass = 1.0;
int max_attempt_good = 1;
int max_attempt_pass = 2;
int moveNum = 0;
int liinAct_reset = 0;
/****** VELMAX VARIABLES *******/
double obj_y_offset = 0;
double obj_y_offset_flat = 30, obj_y_offset_medium = -30, obj_y_offset_deep = 0;
double edgeX_offset = 0;

int obj_marker = 11;
Vector3d centeredObjEdge_objM_diff(30, -28, -23.7);

int velmex_marker = 4;
Vector3d centeredObjEdge_velmexM_diff(-92.5, 218.6, 109.7);



/*************************** INPUT AND OUTPUT ****************************/
string subjectName;
// experiment directory
string experiment_directory = "C:/Users/labdomin/Documents/data/ailin/summer23-ailin-NewRemap-Haptic/";

// paramters file directory and name
ParametersLoader parameters_subj;
ParametersLoader parameters;
string parametersFileName_subj = experiment_directory + "parameters_summer23-ailin-NewRemap-Haptic-MASTER.txt";
string parametersFileName = experiment_directory + "ParametersFiles/parameters_Haptic_Grasping.txt";
string parametersFileName_prep = experiment_directory + "ParametersFiles/parameters_Haptic_Grasping_prep.txt";


// response file
ofstream responseFile;
string responseFile_headers = "subjName\treinforceTexture\tmainTraining\tIOD\tblockN\ttrialN\tdisplayDistance\ttarget_X\tDepthMean\tDepthDelta\tDepth_disp\tDepth_text\tDepth_haptic\tmvID\tMSE1\tRT_MSE1\tRT_G\tnum_texDots\tradius_texDots\tLAcorrection\tcalibNum";


/********** TRIAL SPECIFIC PARAMETERS ***************/
BalanceFactor<double> trial; //if using costant stimuli

int targetCueID; // 0 for texture; 1 for dispairty
bool reinforce_texture_disparity = true;

bool session_full_vs_extra = true;
int sessionNum = 0;
int totalBlkNum = 1;
int blkNum = 1;

int repetition = 3;
int totalTrNum = 4 * 6 * repetition;
int trialNum = 0;
double percentComplete = 0;
int trialNum_break = 20;
int trainNum_cap = 20;




/********** STIMULUS SHAPE ***************/
// stimulus shape
double display_distance;
double visualTarget_X = 19.6, visualTarget_Y = 0;
double visual_angle = 7.4; // stim diangonal size
// jitter in distance
double jitter_z = 0;
double display_distance_jittered = display_distance + jitter_z;
double dist_toEye;


//height and width
double stimulus_height = 70; //tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double stimulus_width = 70; //ratio_bgwidth_height * stimulus_height;
double stimulus_visiblewidth = 70; //ratio_visiblewidth_height * stimulus_height;
double ratio_width_height = 1.36;//1.3;
double ratio_visiblewidth_height = 1.15;//1.1;

// depths of visual stimuli
double depth_mean = 36;
double depth_delta = 0;
double depth_text = 36;
double depth_disp = 36;

// depths of haptic stimuli
double depth_haptic = 36.0; 
double depth_thresh_flat = 25.5, depth_thresh_deep = 34.5;
double haptic_offset = 22.5;
double haptic_offset_flat = 17.5, haptic_offset_medium = 19.5, haptic_offset_deep = 22.5;
enum hapticBumps{flatBump, mediumBump, deepBump};
hapticBumps current_bump = deepBump;

// training
double depth_training_min = 20; // set in the subject file
int depth_training_range = 24; // set in the subject file
double depth_inc = 2; 


/********** STIMULUS VERTICES ***************/
struct Vec2 {
	float x, y;
};

struct CurveYLMap {
	std::vector<double> y_vec;
	std::vector<double> l_vec;
	double curve_depth;
	double curve_height;
	double step_size;
};

struct CurvePtsData {
	std::vector<double> y_vec;
	std::vector<double> z_vec;
	std::vector<double> l_vec;
	double curve_depth;
	double curve_height;
	double step_size;
};


struct TextureDotsData {
	std::vector<Vec2> dot_center_vec;
	Vec2 TexMapSize;
	float Radius;
	float margin_y;
};

struct VerticesData {
	std::vector<GLfloat> vertices_vec;
	std::vector<GLfloat> colors_vec;
	std::vector<GLfloat> light_normals_vec;
	std::vector<GLfloat> textuv_vec;
	std::vector<GLuint> indices_draw_triangle_vec;

};

struct ContourData {
	std::vector<Vector3f> vert_Rcontour;
	std::vector<Vector3f> vert_Lcontour;
};

VerticesData my_verts;
ContourData my_contour_data;
float dot_number;

/********** TEXTURE SURF ***************/
int nr_curve_map = 10001;

double lengthFactor_TM = 1.4; //for a depth with a curve length of l, the TM length is multiplied by this factor.
double del_l = 0.4;

int nr_points_width = 251; // nr of points in x direction
int nr_points_height_default = 201; // default
int nr_points_height = nr_points_height_default;
int total_ind = 0;

/********* TEXTURE *********/
// self-generated
float Tex_dot_radius = 2.7;
float Tex_dot_density = 0.019;
float Tex_dot_separation_ratio = 1.10;

float texture_col_max = 1.0;
float texture_col_min = 0.1;
int TexDot_Lat_nr = 4;
float TexDot_Lat_jitter = 0.4;

//blur edge
double drop_off_rate = 0.75;
double R_intersect_factor = 2 / (1 + drop_off_rate);

/********** LIGHT SHADING ***************/
float max_intensity = 1.0;
float light_amb = 0.3;
float light_dif = 0.5;
float lightDir_z = 0.5;
double light_depthMin = 24;
double light_depthMax = 40;

/********** STATE VARIABLES ***************/
enum Stages {exp_initializing, stimulus_preview, trial_prep, trial_fixate, trial_visualStimulus, trial_MSE,
trial_waitToGrasp, trial_grasp, trial_handReturn, break_time, exp_completed, trial_error};
Stages current_stage = exp_initializing; // if just want to look at the stimuli, use the constant present stage

enum expInitSteps{to_GetCalibrationPoints, to_CalibrateFingerTips, to_CalibrateFingerJoints,  to_MarkHomePos, to_MoveApparatus, to_confirmReady};
expInitSteps currentInitStep = to_GetCalibrationPoints;

enum errorStates{no_error, training_exceeded, blocked_objMarkers, phidget_error, velmex_error, visual_error};
errorStates current_error = no_error;

bool fingerCalibration_TIPandJOINT = false; // if false, calibration two points - tip cand joint
int calibrationNum = 0;
bool Fingers_Calibrated = false;
bool training = true;
bool visibleInfo = true;
bool task_guide_info = true;
bool Exp_Initialized = false;
bool screenBack = false; //has the screen been placed at projection distance
bool stimulus_built = false;

float time_var = 200; // a time variable for the changing thumb target

/********** TIME ***************/
// Timer variable, set for each trial 
Timer trial_timer;
double ElapsedTime;
double timestamp_MSEstart, timestamp_MSEend;
double timestamp_graspstart, timestamp_graspend;

double fixateTime = 600;

/*********** for DEBUGGING **********/
bool testHapticDevice =  false; //true;
double obj_y_diff = 0;
void draw_objEdge();

/********** CONTROL OPTIONS ***************/
bool resetScreen_betweenRuns = true;

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

void initStimulus(double dispDepth, double textDepth);
void drawStimulus();
void drawInfo();

void drawProgressBar();
void drawBlockingPanels(double pandelSeparation);
void drawFixation(double displayDist);

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

bool generateTexture(float TM_X, float TM_Y, float dotDensity, float dotRadius, float dotSeparationRatio_init, int nr_X_Lattice, float dotJitterScale_Lattice, TextureDotsData& outputTexDots);
void buildVertices_Texture(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnText, VerticesData& vertices_data);
void buildContour_Texture(double ContourWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, float distShapeToEye, ContourData& new_contours_vert);
bool buildTextureSurface(double shapeWidth, double shapeHeight, double dispDepth, double textDepth, double distShapeToEye, double contourPanelSeparation, VerticesData& vertices_data, ContourData& contours_vert);
void drawTextureSurface(double distShapeToEye, const VerticesData& vertices_data, const ContourData& contours_vert);
void drawContours(const ContourData& contours_vert);

double getZ(double shapeHeight, double shapeDepth, double vertexY);
double getTg(double shapeHeight, double shapeDepth, double Y);
double SolveForZ_projected(double theHeight, double newDepth, double l, double y0, double z0);
void scanCurve(double shapeHeight, double shapeDepth, CurveYLMap& output_curve_ylmap);
void projectCurve(const CurveYLMap& curve_map_proj, double distShapeToEye, const CurvePtsData& origin_curve, CurvePtsData& output_curve_proj);
Vector3d projectPoint(double shapeHeight, double newDepth, double distShapeToEye, Vector3d fromPoint);
float adjustDiffLight(double textDepth, float maxInt, float ambInt, double Depth_flat, double Depth_deep);

bool change_haptic_object(double hapticDepth);
bool phidget_move_readjust(double setToDistance, int MaxAttempt_good, int MaxAttempt_pass);
bool initHaptic_velmex();
bool initHaptic_phidget();
bool updateObjLocation();

void makeParsFileCopy(string filename_original, string filename_copy);