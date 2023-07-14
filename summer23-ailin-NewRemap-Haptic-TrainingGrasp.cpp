// this script aims to use probe adjustment task to measure the gain/strength of either texture or disparity as a depth cue
#include "summer23-ailin-NewRemap-Haptic-TrainingGrasp.h"

void initPreviewStimulus(double textDepth, double dispDepth) {

	if(reinforce_texture_disparity){
		depth_haptic = depth_text;
	}else{
		depth_haptic = depth_disp;
	}

	change_haptic_object(depth_haptic);
	display_distance_jittered = display_distance;

	if(abs(textDepth - dispDepth) < 0.1){
		buildVertices_congruent(textDepth, normalizer_to_uv);
	}else{	
		buildVertices_incongruent(textDepth, dispDepth, display_distance_jittered, normalizer_to_uv);
	}

}


void buildVertices_congruent(double shapeDepth, double textNormalizer) {

	vertices_vec.clear();
	texcoors_vec.clear();
	colors_vec.clear();
	normals_vec.clear();
	indices_draw_triangle_vec.clear();

	vertContainer_Rcontour.clear();
	vertContainer_Lcontour.clear();
	
	double step_size_width = stimulus_width / (double)(nr_points - 1);
	double step_size_height = stimulus_height / (double)(nr_points - 1);

	GLuint i_ind = 0;

	double x, y, z, y_prev, z_prev, u, v;
	y_prev = -stimulus_height / 2; 
	z_prev = 0; 
	double total_distance_y = 0; //tracks the distance along y/z axis, approximate the "diameter" of the ellipse

	double normal_x, normal_y, normal_z;

	double x_Rcontour = stimulus_visiblewidth / 2;
	double x_Lcontour = -stimulus_visiblewidth / 2;
	

	for (int j = 0; j < nr_points; j++) {  // 

		y = -stimulus_height / 2 + j * step_size_height;
		z = shapeDepth * cos(M_PI * y / stimulus_height);

		total_distance_y = total_distance_y + sqrt(pow(y - y_prev, 2) + pow(z - z_prev, 2));
		v = total_distance_y / textNormalizer + v_offset; //v coordinate

		// shading should be consistent with texture depth
		normal_x = 0;
		normal_y = shapeDepth * sin(M_PI * y / stimulus_height) * M_PI / stimulus_height;
		normal_z = 1;

		vertContainer_Rcontour.push_back(Vector3d(x_Rcontour, y, z)); 
		vertContainer_Lcontour.push_back(Vector3d(x_Lcontour, y, z)); 

		for (int i = 0; i < nr_points; i++) { //

			x = -stimulus_width / 2 + i * step_size_width;
			u = (x + stimulus_width / 2) / textNormalizer + u_offset; //u coordinate. 

			//step 1: build the meshgrid using vertices, 
			vertices_vec.push_back(x);
			vertices_vec.push_back(y);
			vertices_vec.push_back(z);

			colors_vec.push_back(1);
			colors_vec.push_back(0);
			colors_vec.push_back(0);

			texcoors_vec.push_back(u);
			texcoors_vec.push_back(v);

			normals_vec.push_back(normal_x);
			normals_vec.push_back(normal_y);
			normals_vec.push_back(normal_z);

			//step 2: create an array/vector that store how the triangles should be drawn

			// construct the triangle indices to be drawn
			if (i < nr_points - 1 && j < nr_points - 1) {

				indices_draw_triangle_vec.push_back(i_ind);
				indices_draw_triangle_vec.push_back(i_ind + 1);
				indices_draw_triangle_vec.push_back(i_ind + nr_points);

				indices_draw_triangle_vec.push_back(i_ind + nr_points);
				indices_draw_triangle_vec.push_back(i_ind + 1);
				indices_draw_triangle_vec.push_back(i_ind + nr_points + 1);
				//ind = ind + 6;
			}

			i_ind++;
		}

		y_prev = y; z_prev = z;
	}


}


void buildVertices_incongruent(double textDepth, double dispDepth, double displayDist, double textNormalizer) {
	
	double step_size_width = stimulus_width / (double)(nr_points - 1);
	double step_size_height = stimulus_height / (double)(nr_points - 1);

	GLuint i_ind = 0;

	vertices_vec.clear();
	texcoors_vec.clear();
	colors_vec.clear();
	normals_vec.clear();
	indices_draw_triangle_vec.clear();

	vertContainer_Rcontour.clear();
	vertContainer_Lcontour.clear();

	double x_t, y_t, z_t, y_t_prev, z_t_prev, u, v;
	double w, x_d, y_d, z_d;
	y_t_prev = -stimulus_height / 2; 
	z_t_prev = 0; 
	double total_distance_y_t = 0; //tracks the distance along y_t/z_t axis, approximate the "diameter" of the ellipse
	
	double c_cos = M_PI;
	double l = -(displayDist - dispDepth);

	double x_t_Rcontour = stimulus_visiblewidth / 2;
	double x_t_Lcontour = -stimulus_visiblewidth / 2;
	double x_d_Rcontour_Clpeye, x_d_Lcontour_Clpeye;

	double normal_x, normal_y, normal_z;
	
	for (int j = 0; j < nr_points; j++) {  // 

		y_t = -stimulus_height / 2 + j * step_size_height;
		z_t = textDepth * cos(M_PI * y_t / stimulus_height);

		// shading should be consistent with texture depth
		normal_x = 0;
		normal_y = textDepth * sin(M_PI * y_t / stimulus_height) * M_PI / stimulus_height;
		normal_z = 1;
		//normal_norm = sqrt(pow(normal_y,2) + pow(normal_z, 2));

		total_distance_y_t = total_distance_y_t + sqrt(pow(y_t - y_t_prev, 2) + pow(z_t - z_t_prev, 2));
		v = total_distance_y_t / textNormalizer + v_offset; //v coordinate
				
		if(abs(y_t) < 0.01){
			y_d = y_t;
			z_d = dispDepth * cos(M_PI * y_d / stimulus_height) ;

		}else if(abs( abs(y_t)- stimulus_height / 2) < 0.01){
			y_d = y_t;
			z_d = z_t;

		}else{
			c_cos = M_PI * y_t /(stimulus_height * (z_t - l));
			z_d = NewtonSolver_Cosine(stimulus_height, dispDepth, c_cos, l, y_t, z_t);
			w = (z_d - l)/(z_t - l);
			y_d = w * y_t;

		}

		w = (z_d - l)/(z_t - l);

		//x_d_Rcontour_Leye = -(interoculardistance / 2) + w * (x_t_Rcontour + interoculardistance / 2);
		//x_d_Lcontour_Leye = -(interoculardistance / 2) + w * (x_t_Lcontour + interoculardistance / 2);
		//x_d_Rcontour_Reye = (interoculardistance / 2) + w * (x_t_Rcontour - interoculardistance / 2);
		//x_d_Lcontour_Reye = (interoculardistance / 2) + w * (x_t_Lcontour - interoculardistance / 2);
		//vertContainer_std_Rcontour_Leye.push_back(Vector3d(x_d_Rcontour_Leye, y_d, z_d)); 
		//vertContainer_std_Rcontour_Reye.push_back(Vector3d(x_d_Rcontour_Reye, y_d, z_d));
		//vertContainer_std_Lcontour_Leye.push_back(Vector3d(x_d_Lcontour_Leye, y_d, z_d)); 
		//vertContainer_std_Lcontour_Reye.push_back(Vector3d(x_d_Lcontour_Reye, y_d, z_d));
		
		x_d_Rcontour_Clpeye = w * x_t_Rcontour;
		x_d_Lcontour_Clpeye = w * x_t_Lcontour ;

		vertContainer_Rcontour.push_back(Vector3d(x_d_Rcontour_Clpeye, y_d, z_d)); 
		vertContainer_Lcontour.push_back(Vector3d(x_d_Lcontour_Clpeye, y_d, z_d));

		for (int i = 0; i < nr_points; i++) { //

			x_t = -stimulus_width / 2 + i * step_size_width;
			u = (x_t + stimulus_width / 2) / textNormalizer + u_offset; //u coordinate. 

			x_d = x_t; 
	
			//step 1: build the meshgrid using vertices, 

			vertices_vec.push_back(x_d);
			vertices_vec.push_back(y_d);
			vertices_vec.push_back(z_d);

			colors_vec.push_back(1);
			colors_vec.push_back(0);
			colors_vec.push_back(0);

			texcoors_vec.push_back(u);
			texcoors_vec.push_back(v);

			normals_vec.push_back(normal_x);
			normals_vec.push_back(normal_y);
			normals_vec.push_back(normal_z);

			//step 2: create an array/vector that store how the triangles should be drawn

			// construct the triangle indices to be drawn
			if (i < nr_points - 1 && j < nr_points - 1) {

				indices_draw_triangle_vec.push_back(i_ind);
				indices_draw_triangle_vec.push_back(i_ind + 1);
				indices_draw_triangle_vec.push_back(i_ind + nr_points);

				indices_draw_triangle_vec.push_back(i_ind + nr_points);
				indices_draw_triangle_vec.push_back(i_ind + 1);
				indices_draw_triangle_vec.push_back(i_ind + nr_points + 1);
				//ind = ind + 6;
			}

			i_ind++;

		}
				
		y_t_prev = y_t; z_t_prev = z_t;
	}


}


double NewtonSolver_fz(double var_z, double D, double C, double l){
	double val = var_z/D - cos(C*(var_z - l));
	return val;
}

double NewtonSolver_dfz(double var_z, double D, double C, double l){
	double val = 1/D + sin(C*(var_z - l)) * C;
	return val;
}

double NewtonSolver_Cosine(double theHeight, double newDeth_zDenom, double constCos, double l_translatedDist, double y0, double z0){

	double z_new, z, f_z, df_z;
	double C_cos = M_PI * y0/(theHeight * (z0 - l_translatedDist));

	z = z0;

	for (int i = 0; i < 100; i++){

		f_z = NewtonSolver_fz(z, newDeth_zDenom, C_cos, l_translatedDist);
		df_z = NewtonSolver_dfz(z, newDeth_zDenom, C_cos, l_translatedDist);

		if(abs(f_z) < 1e-10){
			//cout << "iteration: " << i << "      success!  fz = " << f_z << endl; 
			break;
		}else if(abs(df_z) < 1e-10){
			//cout << "iteration: " << i << "      uhhhhh  fz = " << f_z << endl; 
			break;
		}else{		
			z_new = z - f_z/df_z;
			z = z_new;
		}
	}

	if(abs(z-z0) > 40)
		z = 0;

	return z;

}

void drawPanels(double displayDist, double dispDepth){

	int n;
	float panel_width = 40;
	float panel_height_extra = 20;

	glPushMatrix();
	glLoadIdentity();
	glTranslated(visualTarget_X, visualTarget_Y, displayDist - dispDepth + 2);

	n = int(vertContainer_Rcontour.size());

	if(n > 0){

		// Right panels
		glBegin(GL_QUAD_STRIP);

		glVertex3f(vertContainer_Rcontour.at(0)[0] + panel_width,		vertContainer_Rcontour.at(0)[1] - panel_height_extra,			vertContainer_Rcontour.at(0)[2]); //0
		glVertex3f(vertContainer_Rcontour.at(0)[0],					vertContainer_Rcontour.at(0)[1] - panel_height_extra,			vertContainer_Rcontour.at(0)[2]); //1

		for (int i = 0; i < n; i++)
		{	
			glVertex3f(vertContainer_Rcontour.at(i)[0] + panel_width,		vertContainer_Rcontour.at(i)[1],			vertContainer_Rcontour.at(i)[2]); //0
			glVertex3f(vertContainer_Rcontour.at(i)[0],					vertContainer_Rcontour.at(i)[1],			vertContainer_Rcontour.at(i)[2]); //1

		}	

		glVertex3f(vertContainer_Rcontour.at(n-1)[0] + panel_width,		vertContainer_Rcontour.at(n-1)[1] + panel_height_extra,			vertContainer_Rcontour.at(n-1)[2]); //0
		glVertex3f(vertContainer_Rcontour.at(n-1)[0],					vertContainer_Rcontour.at(n-1)[1] + panel_height_extra,			vertContainer_Rcontour.at(n-1)[2]); //1

		glEnd();

		// Left panels
		glBegin(GL_QUAD_STRIP);

		glVertex3f(vertContainer_Lcontour.at(0)[0],		vertContainer_Lcontour.at(0)[1] - panel_height_extra,			vertContainer_Lcontour.at(0)[2]); //0
		glVertex3f(vertContainer_Lcontour.at(0)[0] - panel_width,					vertContainer_Lcontour.at(0)[1] - panel_height_extra,			vertContainer_Lcontour.at(0)[2]); //1

		for (int i = 0; i < n; i++)
		{	
			glVertex3f(vertContainer_Lcontour.at(i)[0],		vertContainer_Lcontour.at(i)[1],			vertContainer_Lcontour.at(i)[2]); //0
			glVertex3f(vertContainer_Lcontour.at(i)[0] - panel_width,					vertContainer_Lcontour.at(i)[1],			vertContainer_Lcontour.at(i)[2]); //1

		}	

		glVertex3f(vertContainer_Lcontour.at(n-1)[0],		vertContainer_Lcontour.at(n-1)[1] + panel_height_extra,			vertContainer_Lcontour.at(n-1)[2]); //0
		glVertex3f(vertContainer_Lcontour.at(n-1)[0] - panel_width,					vertContainer_Lcontour.at(n-1)[1] + panel_height_extra,			vertContainer_Lcontour.at(n-1)[2]); //1

		glEnd();
	}


	glPopMatrix();

}


void drawVertices(int texNum, double displayDist, double dispDepth) {

	
    glShadeModel(GL_SMOOTH); // enable Smooth Shading
	glEnable(GL_LIGHTING); // enable lighting
	glEnable(GL_LIGHT1); 
	glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces

	glPushMatrix();
	glLoadIdentity();

	// Light source parameters
	GLfloat LightAmbient[]= { amb_intensity, 0.0f, 0.0f, 1.0f }; // non-directional & overall light (r,g,b,alpha): dark part
	GLfloat LightDiffuse[]= { max_intensity - amb_intensity, 0.0f, 0.0f, 1.0f }; // light created by the light source (directional light; r,g,b,alpha): bright part
	GLfloat LightPosition[]= { 0.0f, 1.f, lightDir_z, 0.0f }; // Light Position (x, y, z, 1.0f); if w==0, directional; if w==1, positional lights. Attenuation can be applied only to the positional light 
	
	//setting the light
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient); //setup the ambient light
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse); //setup the diffuse light
	glLightfv(GL_LIGHT1, GL_POSITION,LightPosition); //position the light

	glPopMatrix();

	// enable matrices for use in drawing below
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);

	
	// bind the texture

	glBindTexture(GL_TEXTURE_2D, loaded_textures[texNum]);

	//glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	
	// activate and specify pointer to vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glPushMatrix();
	glLoadIdentity();
	glTranslated(visualTarget_X, visualTarget_Y, displayDist - dispDepth);


	glVertexPointer(3, GL_FLOAT, 0, &vertices_vec[0]);
	glTexCoordPointer(2, GL_FLOAT, 0, &texcoors_vec[0]);
	glNormalPointer(GL_FLOAT, 0, &normals_vec[0]); //
	glColorPointer(3, GL_FLOAT, 0, &colors_vec[0]);
	glDrawElements(GL_TRIANGLES, indices_draw_triangle_vec.size(), GL_UNSIGNED_INT, &indices_draw_triangle_vec[0]);


	glPopMatrix();

	// deactivate vertex arrays after drawing
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	glDisable(GL_LIGHTING); 


	switch(panel_state){
	case no_aperture:
		break;

	case black_aperture:

		glColor3f(0.0f, 0.0f, 0.0f);

		drawPanels(displayDist, dispDepth);


		break;

	case red_aperture:

		glColor3f(0.4f, 0.0f, 0.0f);
		
		drawPanels(displayDist, dispDepth);

		break;


	}
	

}

void drawBlockingPanels(double pandelSeparation){

	double panel_w = 20, panel_h = 80;

	// left panel
	glDisable(GL_TEXTURE_2D);

	glBegin(GL_QUADS);
	glVertex3f(-pandelSeparation / 2 - panel_w,   panel_h / 2,  0.0f);
	glVertex3f(-pandelSeparation / 2,             panel_h / 2,  0.0f);
	glVertex3f(-pandelSeparation / 2,            -panel_h / 2,  0.0f);
	glVertex3f(-pandelSeparation / 2 - panel_w,  -panel_h / 2,  0.0f);
	glEnd();

	// right panel
	glBegin(GL_QUADS);
	glVertex3f(pandelSeparation / 2,              panel_h / 2,  0.0f);
	glVertex3f(pandelSeparation / 2 + panel_w,    panel_h / 2,  0.0f);
	glVertex3f(pandelSeparation / 2 + panel_w,   -panel_h / 2,  0.0f);
	glVertex3f(pandelSeparation / 2,             -panel_h / 2,  0.0f);
	glEnd();
}

void drawFixation(double displayDist) {
	// draws a small fixation cross at the center of the display
	glDisable(GL_TEXTURE_2D);
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(2.f);

	glPushMatrix();
	glLoadIdentity();
	glTranslated(visualTarget_X, visualTarget_Y, displayDist);
	double cross_length = 5;
	glBegin(GL_LINES);
	glVertex3d(cross_length / 2, 0, 0);
	glVertex3d(-cross_length / 2, 0, 0);
	glVertex3d(0, -cross_length / 2. , 0);
	glVertex3d(0, cross_length / 2. , 0);
	glEnd();
	glPopMatrix();

}

// This function seems to be used to shut down the system after use
void shutdown(){
	cout << "shutting down" << endl;
	responseFile.close(); // close this object
	cleanup();
	exit(0);
}
void cleanup() 
{
// Stop the optotrak
    optotrak.stopCollection();

}
void initProjectionScreen(double _focalDist, const Affine3d &_transformation, bool synchronous)
{
	focalDistance = _focalDist;	
    screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE*SCREEN_HEIGHT/SCREEN_WIDTH);
    screen.setOffset(alignmentX,alignmentY);
    screen.setFocalDistance(_focalDist);
    screen.transform(_transformation);
    cam.init(screen);
	if ( synchronous )
		moveScreenAbsolute(_focalDist,homeFocalDistance,4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist,homeFocalDistance,4500);
}

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

// Initialize motors for moving screen around
void initMotors()
{
	//specify the speed for (objects,screen)
		if(resetScreen_betweenRuns){
		homeEverything(5000,4500);
		}

		phidget_position_read = phidgets_linear_move_slowNearTarget(init_phidget_distance, 6.0, 10);
		prev_phidget_ditance = init_phidget_distance;

}

// Method that initializes the openGL parameters needed for creating the stimuli. 
// seems like this is not changed for each experiment (maybe for different experimental setup eg monitor)
void initRendering()
{   

	glClearColor(0.0,0.0,0.0,1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* Set depth buffer clear value */
	glClearDepth(1.0);
	/* Enable depth test */
	glEnable(GL_DEPTH_TEST);
	/* Set depth function */
	glDepthFunc(GL_LEQUAL);
	// scommenta solo se vuoi attivare lo shading degli stimoli

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glLineWidth(1.5);

	//texture-only
	//glEnable(GL_MULTISAMPLE);
	//glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);


}


void initVariables()
{


	// eye coordinates

	eyeRight = Vector3d(interoculardistance / 2, 0, 0);
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);

	eyeMiddle = Vector3d(0, 0, 0);

	stimulus_height = tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));

	stimulus_bgwidth = ratio_bgwidth_height * stimulus_height;
	stimulus_visiblewidth = ratio_visiblewidth_height * stimulus_height;
	stimulus_width = stimulus_bgwidth;


}

float adjustAmbient(double textDepth, float maxInt, double rateAmbvsDiff_flat, double rateAmbvsDiff_deep, double Depth_flat, double Depth_deep){

	double rateAmbvsDiff_new = rateAmbvsDiff_flat + (rateAmbvsDiff_deep - rateAmbvsDiff_flat) * (textDepth - Depth_flat)/(Depth_deep - Depth_flat);
	float newAmbient = maxInt * (rateAmbvsDiff_new/(rateAmbvsDiff_new + 1));

	return newAmbient;
}


void initBlock()
{
	trial.init(parameters);
	// initialize the trial matrix
	if(session_full_vs_extra){

		repetition = 3;
		totalTrNum = 6 * 8 * repetition;

	}else{

		repetition = 4;
		totalTrNum = 12 * repetition;
	}
	trial.next();

	trialNum = 1;
	
}

// Initialize the streams, open the file and write to it
void initStreams()
{
	ifstream parametersFile_subj;

	parametersFile_subj.open(parametersFileName_subj.c_str());
	parameters_subj.loadParameterFile(parametersFile_subj);
	subjectName = parameters_subj.find("SubjectName");
	interoculardistance = str2num<double>(parameters_subj.find("IOD"));
	display_distance = str2num<double>(parameters_subj.find("dispDepth"));


	string session = parameters_subj.find("TRAINING_Session");
	sessionNum = str2num<int>(session);

	string dirName = experiment_directory + subjectName;
	mkdir(dirName.c_str()); // windows syntax


	// Principal streams files
	if (util::fileExists(dirName + "/" + subjectName + "_s" + session + "_Grasp.txt") && subjectName != "junk")
	{
		string error_on_file_io = string("file already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.", NULL, NULL);
		shutdown();
	}

	int targetCueID = str2num<int>(parameters_subj.find("TRAINING_Cue"));
	if(targetCueID > 0){
		reinforce_texture_disparity = false;
	}else{
		reinforce_texture_disparity = true;
	}




	if(sessionNum > 0){

		if (util::fileExists(experiment_directory + "/" + subjectName + "_s0_Grasp.txt") && subjectName != "junk") {
			session_full_vs_extra = true;
			ifstream parametersFile;
			parametersFile.open(parametersFileName.c_str());
			parameters.loadParameterFile(parametersFile);
		}
		else {
			string error_on_file_io = string("skipped s0. plese complete s0 before the s1");
			cerr << error_on_file_io << endl;
			MessageBox(NULL, (LPCSTR)"s0 is REQUIRED before s1\n Please check the subject's parameters file.", NULL, NULL);
			shutdown();
		}
 
	}else{
		session_full_vs_extra = false;
		ifstream parametersFile;
		parametersFile.open(parametersFileName_prep.c_str());
		parameters.loadParameterFile(parametersFile);

	}

	string responseFileName = dirName + "/" + subjectName + "_s" + session + "_Grasp.txt";
	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;
	
	//globalTimer.start();
}


//Central function for projecting image onto the screen
void drawGLScene()
{
		glDrawBuffer(GL_BACK_RIGHT);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.0,0.0,0.0,1.0);
		cam.setEye(eyeRight);
		

		drawStimulus();
		//drawPanel_Leye(true, display_distance_jittered, depth_disp);
		drawInfo();
		drawTaskGuide();
		
		
		// Draw right eye view
		glDrawBuffer(GL_BACK_LEFT);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.0,0.0,0.0,1.0);
		cam.setEye(eyeLeft);
		

		drawStimulus();
		//drawPanel_Reye(true, display_distance_jittered, depth_disp);
		drawInfo();
		drawTaskGuide();
		

	glutSwapBuffers();
}

void update(int value)
{
    glutPostRedisplay();
    glutTimerFunc(TIMER_MS, update, 0);
}

void drawProgressBar() {
	
	glPushMatrix();
	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	glColor3f(0.2, 0.2, 0.2);
	glBegin(GL_LINE_LOOP);
	glVertex3f(-50, 5, 0);
	glVertex3f(50, 5, 0);
	glVertex3f(50, -5, 0);
	glVertex3f(-50, -5, 0);
	glEnd();

	glColor3f(0.1, 0.3, 0.1);
	glBegin(GL_POLYGON);
	glVertex3f(-50, 5, 0);
	glVertex3f(-50 + percentComplete, 5, 0);
	glVertex3f(-50 + percentComplete, -5, 0);
	glVertex3f(-50, -5, 0);
	glEnd();
	glPopMatrix();
}


void drawInfo_alignment(GLText curText) {

	curText.draw(" ");
	curText.draw(" ");
	curText.draw(" ");
	if (abs(mirrorAlignment - 45.0) < 0.2)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("# Mirror Alignment = " + stringify<double>(mirrorAlignment));

	// check if monitor is calibrated
	if (screenAlignmentY < 89.0)
		glColor3fv(glRed);
	else
		glColor3fv(glGreen);
	curText.draw("# Screen Alignment Y = " + stringify<double>(screenAlignmentY));
	if (abs(screenAlignmentZ) < 89.0)
		glColor3fv(glRed);
	else
		glColor3fv(glGreen);
	curText.draw("# Screen Alignment Z = " + stringify<double>(screenAlignmentZ));
}

void drawInfo_calibrationMarkers(GLText curText) {

	curText.draw(" ");
	curText.draw(" ");

	if (isVisible(markers[calibration_T].p) )
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("Thumb Calibration Point " + stringify< Eigen::Matrix<double, 1, 3> >(markers[calibration_T].p.transpose()));	
	
	if (isVisible(markers[calibration_I].p) )
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);	
	curText.draw("Index Calibration Point " + stringify< Eigen::Matrix<double, 1, 3> >(markers[calibration_I].p.transpose()));

	curText.draw(" ");

	if (allVisibleThumb)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("thumb");


	if (allVisibleIndex)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("index");

	curText.draw("--------------------");	
	curText.draw("centeredObjEdge_velmexM_diff" + stringify<Eigen::Matrix<double, 1, 3> >(centeredObjEdge_velmexM_diff));
	if (isVisible(markers[obj_marker].p) )
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);	
	curText.draw("obj Marker " + stringify<int>(obj_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[obj_marker].p.transpose()) + " [mm]");
curText.draw("edge_marker " + stringify<int>(edge_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[edge_marker].p.transpose()) + " [mm]");

}


void drawInfo_fingers(GLText curText) {
	curText.draw(" ");
	curText.draw(" ");
	curText.draw("--------------------");
	if (allVisibleIndex)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("Index= " + stringify< Eigen::Matrix<double, 1, 3> >(ind.transpose()));
	if (allVisibleThumb)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("Thumb= " + stringify< Eigen::Matrix<double, 1, 3> >(thm.transpose()));

	glColor3fv(glRed);
	curText.draw("--------------------");
	curText.draw(" ");
	curText.draw("# dist thm <-> home = " + stringify<double>(dist_thm_home));
	curText.draw("# handNearHome = " + stringify<bool>(handNearHome));
	curText.draw(" ");
	curText.draw("# dist thm <-> target = " + stringify<double>(dist_thm_target));
	curText.draw("# handNearObject = " + stringify<bool>(handNearObject));
	curText.draw("# handOnObject = " + stringify<bool>(handOnObject));
	curText.draw(" ");
	curText.draw("# Vel thresh = " + stringify<double>(thresholdVelthm_steady));
	curText.draw("# handSteady = " + stringify<bool>(handSteady));

	curText.draw(" ");
	curText.draw("# GA = " + stringify<double>(grip_aperture));
	curText.draw("# gripSmall = " + stringify<bool>(gripSmall));

	curText.draw(" ");
	curText.draw("# Vel GA tresh = " + stringify<double>(thresholdVelGA_steady));
	curText.draw("# gripSteady = " + stringify<bool>(gripSteady));
	curText.draw(" ");

}


void drawTaskGuide() {
	if (task_guide_info) {
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		GLText text;
		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);


		text.enterTextInputMode();
		
		switch (current_stage) {

			case trial_visualStimulus:
			{
				glColor3fv(glWhite);
				text.draw("                                                                           Home:");
				glColor3fv(glRed);
				if (!handNearHome)
					text.draw("                                                                           | | > H < | |");
				if (!gripSmall)
					text.draw("                                                                           T --> <-- X");
				//if (!gripSteady)
				//	text.draw("                                                                           Stop Changing Grip  ->  <-");
			}
			break;


			case trial_MSE:
			{
				glColor3fv(glWhite);
				text.draw("                                                                           ESTIMATE:");
				text.draw("                                                                           Press + to enter");
				glColor3fv(glRed);

				if (!handNearHome)
					text.draw("                                                                           | | > H < | |");

				if (!allVisibleFingers)
					text.draw("                                                                            TT    XX");

				if(attemped_MSE && !gripSteady)
					text.draw("                                                                           Hold ...");
			}
			break;

			case trial_waitToGrasp:
			{
				glColor3fv(glWhite);
				text.draw("                                                                           Home:");
				glColor3fv(glRed);
				if (!handNearHome)
					text.draw("                                                                           | | > H < | |");
				if (!gripSmall)
					text.draw("                                                                           T --> <-- X");
				//if (!gripSteady)
				//	text.draw("                                                                           Stop Changing Grip  ->  <-");
			}
			break;

			case trial_grasp:
			{
				//if(handOnObject && gripSteady){
				glColor3fv(glWhite);
				if(holdCount_target < 3)
					text.draw("                                                                           GRASP:");
				else
					text.draw("                                                                           HOLD Obj:");

				glColor3fv(glRed);
				if (handNearObject && !handOnObject)
					text.draw("                                                                           > > Obj < <");
				if (holdCount_target > 10 && !gripSteady)
					text.draw("                                                                           Hold ...");

			}
			break;


			case trial_handReturn:
			{
				//if(handNearHome && handSteady){
				glColor3fv(glWhite);

				text.draw("                                                                           Return Home:");

				glColor3fv(glRed);
				if (!handNearHome)
					text.draw("                                                                           Not Home |||>");

				if (holdCount_home > 1 && !handSteady && handNearHome)
					text.draw("                                                                           Hold");

			}
			break;

			
		}
		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);
	}
}

void drawInfo()
{
	// displays relevant information to the screen
	if ( visibleInfo )
	{
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		GLText text;
		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

	
		text.enterTextInputMode();

		switch(current_stage){

			case exp_initializing:

				glColor3fv(glWhite);

				switch (currentInitStep) {

					case to_GetCalibrationPoints:
					
						text.draw("Press F to record calibration point positions.");
						//drawInfo_alignment(text);
						drawInfo_calibrationMarkers(text);
					
						break;

					case to_CalibrateFingerTips:
						text.draw("Press F with index and thumb TIPS on calibration points.");
						drawInfo_calibrationMarkers(text);
						break;

					case to_CalibrateFingerJoints:
						text.draw("Press F with index and thumb JOINTS on calibration points.");
						drawInfo_calibrationMarkers(text);
						break;

					case to_MarkHomePos:
						text.draw("Press F with hand at home position.");
						drawInfo_calibrationMarkers(text);
						break;

					case to_MoveApparatus:
						text.draw("Press F to set apparatus in place.");
						drawInfo_calibrationMarkers(text);
						break;

					case to_confirmReady:
						text.draw("Press F to begin!");		
						//drawInfo_fingers(text);
						drawInfo_alignment(text);
						//drawInfo_calibrationMarkers(text);
						break;
				}

				break;

			case stimulus_preview:				
				glColor3fv(glWhite);
				text.draw("Welcome! press + to start training");
				text.draw("# Name: " + subjectName);
				text.draw("# IOD: " + stringify<double>(interoculardistance));
				glColor3fv(glRed);
				if(reinforce_texture_disparity){
					text.draw("----------------------------- P ----------");
				}else{
					text.draw("--------- E ------------------------------");
				}

				//text.draw("# depth texture: " + stringify<double>(depth_text));
				//text.draw("# depth stereo: " + stringify<double>(depth_disp));
				//text.draw("                           ");
				//text.draw("# LIGHT amb intensity: " + stringify<double>(amb_intensity));
				//text.draw("# LIGHT z: " + stringify<float>(lightDir_z));
				//text.draw("                           ");
				//text.draw("#visualTarget_X: " + stringify<double>(visualTarget_X));	

				//text.draw("centeredObjEdge" + stringify<Eigen::Matrix<double, 1, 3> >(centeredObjEdge));
				//text.draw("obj diff" + stringify<Eigen::Matrix<double, 1, 3> >(centeredObjEdge_objM_diff));
				//text.draw("velmex diff" + stringify<Eigen::Matrix<double, 1, 3> >(centeredObjEdge_velmexM_diff));
				break;

			case trial_fixate:
			case trial_visualStimulus:
			case trial_MSE:
			case trial_waitToGrasp:
			case trial_grasp:
			case trial_handReturn:
				
				text.draw("# phidget_position_read: " + stringify<int>(phidget_position_read));
				text.draw("# est_haptic_depth: " + stringify<double>(est_haptic_depth));

				text.draw("# liinAct_correction: " + stringify<int>(liinAct_correction));
				text.draw(" ");
				text.draw("# current stage: " + stringify<int>(current_stage));
				text.draw("# trial Num: " + stringify<int>(trialNum));
				text.draw("# depth haptic: " + stringify<double>(depth_haptic));

				text.draw("bump ID " + stringify<int>(bump_current));
				text.draw("# depth texture: " + stringify<double>(depth_text));
				text.draw("# depth stereo: " + stringify<double>(depth_disp));

				if (isVisible(markers[obj_marker].p) )
					glColor3fv(glGreen);
				else
					glColor3fv(glRed);
				text.draw("obj Marker " + stringify<int>(obj_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[obj_marker].p.transpose()) + " [mm]");

				if (isVisible(markers[velmex_marker].p) )
					glColor3fv(glGreen);
				else
					glColor3fv(glRed);
				text.draw("velmex Marker " + stringify<int>(velmex_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[velmex_marker].p.transpose()) + " [mm]");

				//text.draw("centeredObjEdge" + stringify<Eigen::Matrix<double, 1, 3> >(centeredObjEdge));


				break;


			case break_time:

				switch(current_error_state){
					case no_error:
						glColor3fv(glRed);
						text.draw("Break time! Press + to continue");
						break;

					case training_exceeded:
						glColor3fv(glWhite);
						text.draw("Pls call the experimenter!!!!!!!!!!!!! training exceed");
						break;

					case linAct_error:
						glColor3fv(glWhite);
						text.draw("Pls call the experimenter!!!!!!!!!!!!! linear actuator error");
						text.draw("# depth haptic: " + stringify<double>(depth_haptic));
						break;

					case blocked_objMarkers:
						glColor3fv(glWhite);
						text.draw("Pls call the experimenter!!!!!!!!!!!!! blocking object marker");
						if (isVisible(markers[obj_marker].p) )
							glColor3fv(glGreen);
						else
							glColor3fv(glRed);
						text.draw("obj Marker " + stringify<int>(obj_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[obj_marker].p.transpose()) + " [mm]");

						if (isVisible(markers[velmex_marker].p) )
							glColor3fv(glGreen);
						else
							glColor3fv(glRed);
						text.draw("velmex Marker " + stringify<int>(velmex_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[velmex_marker].p.transpose()) + " [mm]");

						break;

					case velmex_error:
						text.draw("Pls call the experimenter!!!!!!!!!!!!! velmex error");
						text.draw("# centeredObj_y_aiming: " + stringify<double>(centeredObj_y_aiming));
						text.draw("# centeredObj_y_now: " + stringify<double>(centeredObj_y_now));

						break;

				}
				break;

			case exp_completed:
				glColor3fv(glWhite);
				text.draw("The experiment is over. Thank you! :)");
				break;
		}
		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);
	}
}


// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,SCREEN_WIDTH, SCREEN_HEIGHT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

}



void drawStimulus()
{
	//enum Stages {stimulus_preview, prep_trial, trial_fixate_first, trial_present_first,
	//trial_fixate_second, trial_present_second, trial_respond, break_time, exp_completed};

	//draw_objEdge();

	if(Fingers_Calibrated){
		draw_thumb_dots();
	}	

	switch(current_stage){

		case stimulus_preview:
			// testing texture, build vertices for curved surface
			drawThumbTarget();
			drawVertices(texnum, display_distance_jittered, depth_disp);
			//drawFixation(display_distance_jittered);
			
			break;

		case trial_fixate:

			drawFixation(display_distance_jittered);

			break;

		case trial_visualStimulus:
		case trial_MSE:
		case trial_waitToGrasp:
			drawVertices(texnum, display_distance_jittered, depth_disp);
			break;

		case trial_grasp:
			drawThumbTarget();
			drawVertices(texnum, display_distance_jittered, depth_disp);

			break;

		case break_time:
			drawProgressBar();
			break;


	}
}




void initTrial()
{
	current_stage = prep_trial;
				
	//initProjectionScreen(display_distance);

	holdCount_target = 0;
	holdCount_home = 0;
	liinAct_correction = 0;
	attemped_MSE = false;

	if(training){

		depth_mean = depth_training_min + depth_inc * (rand() % 13);
		//depth_mean = depth_thresh_flat + (rand() % 13);
		depth_disp = depth_mean;
		depth_text = depth_mean;
	}
	else {

		depth_mean = trial.getCurrent()["meanDepth"] + (rand() % 3 - 1.0);
		if(sessionNum > 0){
			depth_delta = trial.getCurrent()["DepthDelta"];
		
			depth_text = depth_mean + depth_delta;
			depth_disp = depth_mean - depth_delta;
		}else{
			depth_text = depth_mean;
			depth_disp = depth_mean;
		}

	}
	
	if(reinforce_texture_disparity){
		depth_haptic = depth_text;
	}else{
		depth_haptic = depth_disp;
	}

	change_haptic_object(depth_haptic);
		
	//jitter_z = 0;
	display_distance_jittered = display_distance;

	normalizer_to_uv = normalizer_to_uv_base + ((rand() % 13) - 6); // from base-6 to base+6

	if(abs(depth_text - depth_disp) < 0.1){
		buildVertices_congruent(depth_text, normalizer_to_uv);
	}else{
		buildVertices_incongruent(depth_text, depth_disp, display_distance_jittered, normalizer_to_uv);
	}		

	texnum = rand() % 50 + 1;
	amb_intensity = adjustAmbient(depth_text, max_intensity, 1.0, 0.6, 20, 40);
	

	linActRedFlag = check_linAct(linAct_offset_sm);

	if (linActRedFlag > 0) {

		phidget_position_read = phidgets_linear_move_slowNearTarget(net_phidget_distance + 5.0, 6.0, 10);
		phidget_position_read = phidgets_linear_move_slowNearTarget(net_phidget_distance, 6.0, 10);
		prev_phidget_ditance = net_phidget_distance;
		linAct_measured_distance = phidget_position_read / 10.0;
		est_haptic_depth = linAct_measured_distance + grasping_phidget_depthDiff;

		liinAct_correction++;
	}

	//check_haptic();

	if(current_error_state < 1){
		trial_timer.reset();				
		trial_timer.start();
		ElapsedTime = 0;
		current_stage = trial_fixate;
	}else{
		current_stage = break_time;
		visibleInfo = true;
	}
}


void change_haptic_object(double hapticDepth){


	// set the target y and depth
	if(hapticDepth < depth_thresh_flat){ // depth_thresh_flat = 25
		bump_current = flatBump;
		centeredObj_y_aiming = obj_y_offset_flat;
		grasping_phidget_depthDiff = grasping_phidget_depthDiff_flat;
		
	}else if(hapticDepth > depth_thresh_deep){ // depth_thresh_deep = 38
		bump_current = deepBump;
		centeredObj_y_aiming = obj_y_offset_deep;
		grasping_phidget_depthDiff = grasping_phidget_depthDiff_deep;
	}else{
		bump_current = mediumBump;
		centeredObj_y_aiming = obj_y_offset_medium;
		grasping_phidget_depthDiff = grasping_phidget_depthDiff_medium;
	}

	// move the linear actuator to change depth
	net_phidget_distance = hapticDepth - grasping_phidget_depthDiff;

	if(abs(net_phidget_distance - prev_phidget_ditance)> 2){

		phidget_position_read = phidgets_linear_move_slowNearTarget(net_phidget_distance, 6.0, 10);

	}else{

		phidget_position_read = phidgets_linear_move_slowNearTarget(net_phidget_distance + 4.0, 6.0, 10);
		phidget_position_read = phidgets_linear_move_slowNearTarget(net_phidget_distance, 6.0, 10);
	
	}

	linAct_measured_distance = phidget_position_read / 10.0;
	prev_phidget_ditance = net_phidget_distance;
	est_haptic_depth = linAct_measured_distance + grasping_phidget_depthDiff;

	// move velmex
	centeredObj_y_now = find_centerObj_y();
	// move the velmex to change y and bump
	if(centeredObj_y_now < 900 && abs(centeredObj_y_aiming - centeredObj_y_now) > 2){
		moveObject(Vector3d(0,centeredObj_y_aiming - centeredObj_y_now, 0), 7000);
	}

}


int check_linAct(double offsetAllowed) {

	if (abs(linAct_measured_distance - net_phidget_distance) > offsetAllowed) {
		return 1;
	}
	else {
		return 0;
	}

}


double find_centerObj_y(){

	double y_now;

	if(isVisible(markers.at(obj_marker).p)){
		y_now = markers.at(obj_marker).p.y() + centeredObjEdge_objM_diff[1];

	}else if(isVisible(markers.at(velmex_marker).p)){
		y_now = markers.at(velmex_marker).p.y() + centeredObjEdge_velmexM_diff[1];		
	}else{
		beepOk(24);
		y_now = 999;
		current_error_state = blocked_objMarkers;
	}

	return y_now;

}


int check_velmex(){

	centeredObj_y_now = find_centerObj_y();

	if((current_error_state < 1) && abs(centeredObj_y_aiming - centeredObj_y_now) > 2){
		moveObject(Vector3d(0,centeredObj_y_aiming - centeredObj_y_now, 0), 7000);
		return 1;
	}else{
		return 0;
	}
}

void check_haptic(){

	current_error_state = no_error;

	centeredObj_y_now = find_centerObj_y();

	if(current_error_state == blocked_objMarkers){
		beepOk(24);
		current_stage = break_time;
		visibleInfo = true;
	}else{
		if(linActRedFlag > 0)
			linActRedFlag = check_linAct(linAct_offset_lg);

		if(abs(centeredObj_y_aiming - centeredObj_y_now) > 4){
			beepOk(24);
			current_error_state = velmex_error;
			current_stage = break_time;
			visibleInfo = true;	
		}else if(linActRedFlag > 0){
			beepOk(24);
			current_error_state = linAct_error;
			current_stage = break_time;
			visibleInfo = true;	
		}else{
			current_error_state = no_error;
		}
	}

}


void onlineTrial(){
	//stimulus_preview, prep_trial, trial_fixate_first, trial_present_first, trial_fixate_second, trial_present_second, trial_respond,

	/*
		handNearHome = (dist_thm_home < thresholdDist_near_home);
		handNearObject = (dist_thm_target < thresholdDist_near_target);
		handOnObject = (dist_thm_target < thresholdDist_on_target);
		handSteady = (abs(vel_dist_target) < thresholdVelthm_steady);
		gripSmall = (grip_aperture < thresholdGA_small);
		gripSteady = (abs(vel_grip_change) < thresholdVelGA_steady);
	*/

	  
	switch(current_stage){

	case trial_fixate:

		if (ElapsedTime > fixateTime) {	

			current_stage = trial_visualStimulus;
		}		
		break;

	case trial_visualStimulus:

		if(testHapticDevice){			
			velmexRedFlag = check_velmex();
			current_stage = trial_MSE;
		}else{

			// ready for MSE?
			if(handNearHome && gripSmall && gripSteady){
				beepOk(21);
				timestamp_MSEstart = ElapsedTime;
				velmexRedFlag = check_velmex();
				current_stage = trial_MSE;
			}

		}

		break;

	case trial_MSE:

		break;

	case trial_waitToGrasp:
		// MSE registered, ready for grasp?
		if(handNearHome && gripSmall && gripSteady){
		//if((dist_thm_home < thresholdDist_near_home) && (grip_aperture < thresholdGA_small) && (abs(vel_grip_change) < thresholdVelGA_steady)){
			beepOk(22);
			timestamp_graspstart = ElapsedTime;
			current_stage = trial_grasp;
		}
		break;

	case trial_grasp:
		if(handOnObject && gripSteady){
		//if((dist_thm_target < thresholdDist_on_target) && (abs(vel_grip_change) < thresholdVelGA_steady)){
			holdCount_target++;
		}

		if(holdCount_target > threshHoldCout_target){
			beepOk(9);
			timestamp_graspend = ElapsedTime;
			current_stage = trial_handReturn;
		}
		break;

	case trial_handReturn:
		if(handNearHome && handSteady){
		//if((dist_thm_home < thresholdDist_near_home) && (grip_aperture < thresholdGA_small) && (abs(vel_dist_target) < thresholdVelthm_steady)){
			holdCount_home++;
		}

		if(holdCount_home > 30){
			advanceTrial();
		}
		break;


	}

}

void advanceTrial()
{
		responseFile << fixed <<
		subjectName << "\t" <<		
		interoculardistance << "\t" <<
		blkNum << "\t" <<
		trialNum << "\t" <<
		display_distance << "\t" <<
		visual_angle << "\t" <<
		stimulus_height << "\t" <<
		texnum << "\t" <<
		normalizer_to_uv << "\t" <<
		depth_mean << "\t" <<
		depth_delta << "\t" <<
		depth_text << "\t" <<
		depth_disp << "\t" <<
		reinforce_texture_disparity  << "\t" <<
		depth_haptic << "\t" <<
		grip_aperture_MSE << "\t" <<
		timestamp_MSEend - timestamp_MSEstart << "\t" <<
		timestamp_graspend - timestamp_graspstart << "\t" <<
		liinAct_correction  << "\t" <<
		calibrationNum << endl;

	//subjName\tIOD\tblockN\ttrialN\tdisplayDistance\tvisualAngle\tclyHorizontal\ttexnum\ttextNomralizer\ttestDepth\tprobeStart\tprobeDepth\ttime
	if(training){
		if(trialNum < trainNum_cap){
			beepOk(4);
			trialNum++;
			initTrial();
		}else{
			beepOk(24);
			trialNum++;
			current_error_state = training_exceeded;
			current_stage = break_time;
			visibleInfo = true;			
		}

	}else{

		//trial.next(respond_cmp_deeper);	

		if (!trial.isEmpty() && (trialNum < trialNum_max)){

			if(trialNum % 24 == 0){
				beepOk(4);				
				percentComplete = trialNum /(totalTrNum/100.0);	
				trial.next();
				current_stage = break_time;
				visibleInfo = true;

			}else{
				beepOk(4);
				trialNum++;
				trial.next();
				initTrial();
			}
			
		}else{
			beepOk(1);
			responseFile.close();
			visibleInfo=true;
			current_stage = exp_completed;
		}

	}


}



void handleKeypress(unsigned char key, int x, int y)
{   
	switch (key){ // key presses that work regardless of the stage
	
		case 27:	//corrisponde al tasto ESC
			phidget_position_read = phidgets_linear_move_slowNearTarget(init_phidget_distance, 6.0, 10);

			if(resetScreen_betweenRuns){
				homeEverything(5000,4500);
				shutdown();	
			}else{
				shutdown();	
			}
				
		break;

		case 'a':
			grip_aperture_MSE = 999;
			advanceTrial();
			break;

		case 'i':
			visibleInfo=!visibleInfo;
		break;

		case 'o':
			panel_state = panelStates((panel_state + 1)%3);
			break;

		case '1':
			
			if(current_stage == stimulus_preview){
				if (depth_text > depth_inc)
					depth_text = depth_text - depth_inc;

				amb_intensity = adjustAmbient(depth_text, max_intensity, 1.0, 0.6, 20, 40);
				initPreviewStimulus(depth_text, depth_disp);

			}
			
			break;

		case '2':
		
			if(current_stage == stimulus_preview){
				depth_text = depth_text + depth_inc;
				amb_intensity = adjustAmbient(depth_text, max_intensity, 1.0, 0.6, 20, 40);
				initPreviewStimulus(depth_text, depth_disp);
			}

			break;

		case 'R':
		case 'r':

			switch(current_error_state){
				case no_error:
					initTrial();
					break;

				case linAct_error:
					phidget_position_read = phidgets_linear_move_slowNearTarget(init_phidget_distance, 6.0, 10);
					phidget_position_read = phidgets_linear_move_slowNearTarget(net_phidget_distance, 6.0, 10);
					prev_phidget_ditance = net_phidget_distance;
					linAct_measured_distance = phidget_position_read / 10.0;
					est_haptic_depth = linAct_measured_distance + grasping_phidget_depthDiff;
					liinAct_correction++;
					check_haptic();

					if(current_error_state < 1){

						trial_timer.reset();				
						trial_timer.start();
						ElapsedTime = 0;
						current_stage = trial_fixate;
					}

					break;

				case blocked_objMarkers:
					check_haptic();
					if(current_error_state < 1){

						trial_timer.reset();				
						trial_timer.start();
						ElapsedTime = 0;
						current_stage = trial_fixate;
					}
					break;

				case velmex_error:
					centeredObj_y_now = find_centerObj_y();
					moveObject(Vector3d(0,centeredObj_y_aiming - centeredObj_y_now, 0), 7000);
					check_haptic();
					if(current_error_state < 1){

						trial_timer.reset();				
						trial_timer.start();
						ElapsedTime = 0;
						current_stage = trial_fixate;
					}
					break;

			}
			visibleInfo = false;
			break;


		case 'f':
		case 'F':
			{ 
				switch (currentInitStep)
				{ 
					case to_GetCalibrationPoints:
					case to_CalibrateFingerTips:
					case to_CalibrateFingerJoints:
						
						calibrate_fingers();
						
					break;

					case to_MoveApparatus:
					case to_MarkHomePos:
					case to_confirmReady:

						calibrate_system();

					break;									
				}
			}
		break;

		case 'q':
			Fingers_Calibrated = false;
			currentInitStep = to_GetCalibrationPoints;
			current_stage = exp_initializing;
			visibleInfo = true;
			break;
			
		case '+':
			switch(current_stage){
				case stimulus_preview:
					
					beepOk(5);
					visibleInfo = false;
					initBlock();
					initTrial();					
				break;	

				case trial_MSE:

				if(!testHapticDevice){

					attemped_MSE = true;

					if((grip_aperture < 200) && handNearHome && gripSteady){
							
						beepOk(4);
						grip_aperture_MSE = grip_aperture;
						timestamp_MSEend = ElapsedTime;

						check_haptic();
						if((int)current_error_state < 1){
							current_stage = trial_waitToGrasp;
						}
					}

				}else{
					check_haptic();

					if((int)current_error_state < 1){
						advanceTrial();
					}
				}
						
				break;

				case trial_waitToGrasp:
					if(testHapticDevice){
							advanceTrial();
						}
					break;

				case break_time:
					if (abs(mirrorAlignment - 45.0) < 0.2){
						beepOk(5);
						visibleInfo = false;
						trialNum++;
						initTrial();
					}else{
						beepOk(20);
					}
				break;				
			}
			break;

		case 'T':
		case 't':
			if(training && (current_stage != stimulus_preview)){
				training = false;
				task_guide_info = false;
				beepOk(6);
				trialNum = 0;
				threshHoldCout_target = 40;

				if(current_error_state == training_exceeded){
					current_error_state = no_error;
				}

				visibleInfo = true;
				current_stage = break_time;
			}
			break;

		case '7':
			
			if(current_stage == stimulus_preview){
				
				if (depth_disp > depth_inc)
					depth_disp = depth_disp - depth_inc;

				initPreviewStimulus(depth_text, depth_disp);
				
			}

			break;

		case '8':
			
			if(current_stage == stimulus_preview){
				
				depth_disp = depth_disp + depth_inc;

				initPreviewStimulus(depth_text, depth_disp);
				
			}
			
			break;

		case 'h':
			task_guide_info = true;
			break;


		case 'm':
			edgeM_x_offset--;
			centeredObjEdge = markers.at(edge_marker).p + Vector3d(edgeM_x_offset, 0, edgeM_z_offset);
			centeredObjEdge_velmexM_diff = centeredObjEdge - markers.at(velmex_marker).p;
			centeredObjEdge_objM_diff = centeredObjEdge - markers.at(obj_marker).p;

			break;

		case 'n':
			edgeM_x_offset++;
			centeredObjEdge = markers.at(edge_marker).p + Vector3d(edgeM_x_offset, 0, edgeM_z_offset);
			centeredObjEdge_velmexM_diff = centeredObjEdge - markers.at(velmex_marker).p;
			centeredObjEdge_objM_diff = centeredObjEdge - markers.at(obj_marker).p;

			break;

		case 'j':
			edgeM_z_offset--;
			centeredObjEdge = markers.at(edge_marker).p + Vector3d(edgeM_x_offset, 0, edgeM_z_offset);
			centeredObjEdge_velmexM_diff = centeredObjEdge - markers.at(velmex_marker).p;
			centeredObjEdge_objM_diff = centeredObjEdge - markers.at(obj_marker).p;

			break;

		case 'k':
			edgeM_z_offset++;
			centeredObjEdge = markers.at(edge_marker).p + Vector3d(edgeM_x_offset, 0, edgeM_z_offset);
			centeredObjEdge_velmexM_diff = centeredObjEdge - markers.at(velmex_marker).p;
			centeredObjEdge_objM_diff = centeredObjEdge - markers.at(obj_marker).p;
			break;

			
	}

}

/***** SOUNDS *****/
void beepOk(int tone)
{

	switch(tone)
	{

	case 1: //high pitch beep
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-8_lowpass.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 4: //mellow and good for trials
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-440-pluck.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 2: //reject like buzz
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-10.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 3: //reject short
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-reject.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 5: //"go"
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\spoken-go.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 6: //mellow and good for trials
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-lowBubblePop.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 8: //spoken mirror
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\spoken-mirror.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 9: //mellow and good for trials
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-highBubblePop.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 15:
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-rising.wav",
			NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 16:
		PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-falling.wav",
			NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 17:
	PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-440-pluck-5below.wav",
		NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 18: // light click
	PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\beep-click3MS.wav",
		NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 20: // alighnment
	PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\spoken-alignment.wav",
		NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 21: // estimate
	PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\spoken-estimate-short.wav",
		NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 22: // grasp
	PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\spoken-grasp-short.wav",
		NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 23: // home
	PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\spoken-home.wav",
		NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 24: // help
	PlaySound((LPCSTR) "C:\\cncsvision\\data\\beep\\spoken-help.wav",
		NULL, SND_FILENAME | SND_ASYNC);
	break;
	}
	return;
}

void idle()
{
	updateTheMarkers();
	online_apparatus_alignment();
	online_fingers();	
	onlineTrial();
	ElapsedTime = trial_timer.getElapsedTimeInMilliSec();
	
}

/*** Online operations ***/
void online_apparatus_alignment()
{
    // mirror alignment check
	if(isVisible(markers.at(mirror1).p) && isVisible(markers.at(mirror2).p) ){
		mirrorAlignment = asin(
			abs((markers.at(mirror1).p.z() - markers.at(mirror2).p.z())) /
			sqrt(
			pow(markers.at(mirror1).p.x() - markers.at(mirror2).p.x(), 2) +
			pow(markers.at(mirror1).p.z() - markers.at(mirror2).p.z(), 2)
			)
		   ) * 180 / M_PI;
	}else{
		mirrorAlignment = 999;
	}

    // screen Y alignment check
	if(isVisible(markers.at(screen1).p) && isVisible(markers.at(screen3).p) ){
		screenAlignmentY = asin(
			abs((markers.at(screen1).p.y() - markers.at(screen3).p.y())) /
			sqrt(
			pow(markers.at(screen1).p.x() - markers.at(screen3).p.x(), 2) +
			pow(markers.at(screen1).p.y() - markers.at(screen3).p.y(), 2)
			)
			) * 180 / M_PI;
	}else{
		screenAlignmentY = 999;
	}

    // screen Z alignment check
	if(isVisible(markers.at(screen1).p) && isVisible(markers.at(screen2).p) ){
		screenAlignmentZ = asin(
			abs(markers.at(screen1).p.z() - markers.at(screen2).p.z()) /
			sqrt(
			pow(markers.at(screen1).p.x() - markers.at(screen2).p.x(), 2) +
			pow(markers.at(screen1).p.z() - markers.at(screen2).p.z(), 2)
			)
			) * 180 / M_PI *
			abs(markers.at(screen1).p.x() - markers.at(screen2).p.x()) /
			(markers.at(screen1).p.x() - markers.at(screen2).p.x());
	}else{
		screenAlignmentZ = 999;
	}
}

int LoadGLTextures()  // Load PNG And Convert To Textures
{

	for (int i = 1; i <= 50; i++) {
		std::stringstream ss;
		ss << i;

		string texturePath = "C:/cncsvision/experimentsbrown/ShapeConstancy/textures/polkadots/0/polkadots" + ss.str() + ".png";
		 loaded_textures[i] = SOIL_load_OGL_texture
		(
			texturePath.c_str(),
			SOIL_LOAD_AUTO,
			SOIL_CREATE_NEW_ID,
			SOIL_FLAG_MULTIPLY_ALPHA // | SOIL_FLAG_MIPMAPS
		);
	}


	return true; // Return Success
}

void calibrate_fingers()
{
	switch (currentInitStep)
	{
		case to_GetCalibrationPoints: // mark calibration reference points
		
			if (isVisible(markers[calibration_T].p) && isVisible(markers[calibration_I].p))
			{
				{
					// calibrate for grasping along z axis (update the offset number)
					thumbCalibrationPoint = markers.at(calibration_T).p + thumbCalibration_offset;// calibration point is the touching point
					indexCalibrationPoint = markers.at(calibration_I).p + indexCalibration_offset;// calibration point is the touching point
				}

				beepOk(1);
				//homePos = (thumbCalibrationPoint + indexCalibrationPoint)/2 + Vector3d(-10, 25, 0);

				currentInitStep = to_CalibrateFingerTips;
			}
			else {
				beepOk(3);
			}
		
			break;

		case to_CalibrateFingerTips: // mark finger tips

			if(allVisibleFingers)
			{
				indexCoords.init(indexCalibrationPoint, markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
				thumbCoords.init(thumbCalibrationPoint, markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);
				
				beepOk(1);
				if(fingerCalibration_TIPandJOINT){
					currentInitStep = to_CalibrateFingerJoints;
				}
				else {
					calibrationNum++;
					if(Exp_Initialized){
						Fingers_Calibrated = true;	
						initTrial();
					}else{
						currentInitStep = to_MarkHomePos; //to_MarkHomePos;
						Fingers_Calibrated = true;	
					}
				}
			} 
			else {
				beepOk(3);
			}
		
			break;

		case to_CalibrateFingerJoints: // mark finger joints

			if(allVisibleFingers)
			{
				indexJointCoords.init(indexCalibrationPoint, markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
				thumbJointCoords.init(thumbCalibrationPoint, markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);

				beepOk(1);
				currentInitStep = to_MarkHomePos;
				Fingers_Calibrated = true;
			}
			else {
				beepOk(3);
			}
			break;
	

	}
}


void calibrate_system(){
	switch (currentInitStep){ 
		case to_MarkHomePos: 
			// ask hand to rest at a designated place and press F to mark that as the home position
			if (allVisibleFingers) {
				beepOk(1);
				homePos = (ind + thm) / 2;

				if (Exp_Initialized) {
					initTrial();
					visibleInfo = false;
				}
				else {
					currentInitStep = to_MoveApparatus;
				}
			}
			else {
				beepOk(3);
			}
			break;

		case to_MoveApparatus:
	
			// push the physical object back to clear the grasping space
			if(isVisible(markers[obj_marker].p)){

				beepOk(1);

				if(known_edge_objM_diff){
					centeredObjEdge = markers.at(obj_marker).p + centeredObjEdge_objM_diff;
				}else{
					centeredObjEdge = markers.at(edge_marker).p + Vector3d(edgeM_x_offset, 0, edgeM_z_offset);
				}

				moveObjectAbsolute(Vector3d(centeredObjEdge[0],0, display_distance), centeredObjEdge, 6200);

				initProjectionScreen(display_distance);

				currentInitStep = to_confirmReady;
			}else{
				beepOk(3);
			}
			break;

		case to_confirmReady:
			if (abs(mirrorAlignment - 45.0) < 0.2){
				beepOk(1);


				if(known_edge_objM_diff){
					
					if(isVisible(markers[obj_marker].p)){
						centeredObjEdge = markers.at(obj_marker).p + centeredObjEdge_objM_diff;	
					}else{
						centeredObjEdge[1] = 0;
						centeredObjEdge[2] = display_distance;
					}
				}else{
					centeredObjEdge = markers.at(edge_marker).p + Vector3d(edgeM_x_offset, 0, edgeM_z_offset);
					centeredObjEdge_velmexM_diff = centeredObjEdge - markers.at(velmex_marker).p;
					centeredObjEdge_objM_diff = centeredObjEdge - markers.at(obj_marker).p;
				}

				visualTarget_X = centeredObjEdge[0] - stimulus_visiblewidth/2;
				visualTarget_Y = centeredObjEdge[1];

				visTarget = Vector3d(visualTarget_X, visualTarget_Y, display_distance); 
				
				thmTarget = centeredObjEdge + Vector3d(-12.5,0,0);
				Exp_Initialized = true;

				//visibleInfo = false;
				initPreviewStimulus(depth_text, depth_disp);
				current_stage = stimulus_preview;
			}else{
				beepOk(8);
			}
			break;
	}

}

void online_fingers()
{
	allVisibleIndex = isVisible(markers.at(ind1).p) && isVisible(markers.at(ind2).p) && isVisible(markers.at(ind3).p);
	allVisibleThumb = isVisible(markers.at(thu1).p) && isVisible(markers.at(thu2).p) && isVisible(markers.at(thu3).p);
	allVisibleFingers = allVisibleIndex && allVisibleThumb;

	if (Fingers_Calibrated) {

		if(allVisibleThumb){
			thumbCoords.update(markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);		
			thm = thumbCoords.getP1();
			
			thmToHome = homePos - thm;
			thmToTarget = thmTarget - thm;

			old_dist_thm_home = dist_thm_home;
			dist_thm_home = thmToHome.norm();
			vel_dist_home = dist_thm_home - old_dist_thm_home;

			old_dist_thm_target = dist_thm_target;
			dist_thm_target = thmToTarget.norm();
			vel_dist_target = dist_thm_target - old_dist_thm_target;

		}else{

			old_dist_thm_home = dist_thm_home;
			dist_thm_home = 999;
			vel_dist_home = 999;

			old_dist_thm_target = dist_thm_target;
			dist_thm_target = 999;
			vel_dist_target = 999;
		}

		if(allVisibleFingers){
			indexCoords.update(markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
			ind = indexCoords.getP1();

			vec_ind_thm = ind - thm;
			old_grip_aperture = grip_aperture;
			grip_aperture = vec_ind_thm.norm();
			vel_grip_change = grip_aperture - old_grip_aperture;
		}else{
			grip_aperture = 999;
			old_grip_aperture = grip_aperture;
			vel_grip_change = 999;
		}

		handNearHome = (dist_thm_home < thresholdDist_near_home);
		handNearObject = (dist_thm_target < thresholdDist_near_target);
		
		handSteady = (abs(vel_dist_target) < thresholdVelthm_steady);
		gripSmall = (grip_aperture < thresholdGA_small);
		gripSteady = (abs(vel_grip_change) < thresholdVelGA_steady);
	
		handOnObject = (dist_thm_target < thresholdDist_on_target);

	}

}

void draw_objEdge(){
	// this is to draw the object edge
		glColor3f(1.0f,1.0f,1.0f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(centeredObjEdge.x(), centeredObjEdge.y(), centeredObjEdge.z()+ 1.0);
		glBegin(GL_LINES);
		glVertex3d(0, 4, 0);
		glVertex3d(0, -4, 0);
		glVertex3d(-1.5, 0, 0);
		glVertex3d(1.5, 0, 0);
		glEnd();
		glPopMatrix();
}

void draw_fingers_dots()
{  
	// Index dot
	if(allVisibleFingers){
		glColor3f(1.0f,0.0f,0.0f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(ind.x(), ind.y(), ind.z());
		glutSolidSphere(.8, 10, 10);
		glPopMatrix();

		// Thumb dot
		glPushMatrix();
		glLoadIdentity();
		glTranslated(thm.x(), thm.y(), thm.z());
		glutSolidSphere(.8, 10, 10);

		glPopMatrix();
	} else {
		glColor3f(0.0f,0.8f,0.0f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(0, 0, display_distance);
		glutSolidSphere(.8, 10, 10);

		glPopMatrix();
	}

}

void draw_thumb_dots()
{  
	// Index dot
	if(allVisibleThumb){
		if(!handOnObject){
			glColor3f(1.0f,0.0f,0.0f);
			// Thumb dot
			glPushMatrix();
			glLoadIdentity();
			glTranslated(thm.x(), thm.y(), thm.z());
			glutSolidSphere(2, 10, 10);
			//glutWireSphere(3, 8, 8);

			glPopMatrix();
		}
	} else {
		glColor3f(0.0f,0.8f,0.0f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(thmTarget[0] - stimulus_width, thmTarget[1] - 10, thmTarget[2] + 5);
		//glutSolidSphere(.8, 10, 10);

		double T_length = 4;
		glBegin(GL_LINES);
		glVertex3d(-T_length / 2, T_length / 2, 0);
		glVertex3d(T_length / 2, T_length / 2, 0);
		glVertex3d(0, T_length / 2. , 0);
		glVertex3d(0, -T_length / 2. , 0);
		glEnd();

		glPopMatrix();
	}

	if(!allVisibleIndex){
		glColor3f(0.0f,0.8f,0.0f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(thmTarget[0] - stimulus_width, thmTarget[1] + 10, thmTarget[2] + 5);
		double cross_length = 4;
		glBegin(GL_LINES);
		glVertex3d(-cross_length / 2, cross_length / 2, 0);
		glVertex3d(cross_length / 2, -cross_length / 2, 0);
		glVertex3d(cross_length / 2, cross_length / 2. , 0);
		glVertex3d(-cross_length / 2, -cross_length / 2. , 0);
		glEnd();

		glPopMatrix();
	}

}


void drawMarker(int Marker_ID){

	if(isVisible(markers[Marker_ID].p)){
		glColor3f(0.7f,0.7f,0.7f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(markers[Marker_ID].p[0], markers[Marker_ID].p[1], markers[Marker_ID].p[2]);
		glutSolidSphere(1.5, 10, 10);
		glPopMatrix();
	}
}

void drawThumbTarget(){

	if(!handOnObject){	
		/*
		glColor3f(0.4f,0.4f,0.4f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(thmTarget[0], thmTarget[1], thmTarget[2] + 1);
		glutSolidSphere(2., 10, 10);
		glPopMatrix();
		*/

		glColor3f( (cos(ElapsedTime/time_var) + 1.0)/2.0, 0.0f,0.0f);
		glPushMatrix();
		glLoadIdentity();
		glTranslated(thmTarget[0], thmTarget[1], thmTarget[2] + 1);
		//glutSolidSphere(2., 10, 10);
		glutSolidCube(5.0);
		glPopMatrix();

	}
	
}

// this is run at compilation because it's titled f'main'
int main(int argc, char*argv[])  
{
	//functions from cncsvision packages
	mathcommon::randomizeStart();
		
	// initializing glut (to use OpenGL)
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STEREO);
	//glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STEREO | GLUT_MULTISAMPLE);

	glutGameModeString("1024x768:32@85"); //resolution  
	glutEnterGameMode();
	glutFullScreen();
	
	// initializes optotrak and velmex motors
	initRotationM();
	initOptotrak();
	initMotors();
	
	initRendering(); // initializes the openGL parameters needed for creating the stimuli
	
	initStreams(); // streams as in files for writing data

	initVariables();

	LoadGLTextures();


	//initProjectionScreen(display_distance);

	// glut callback, OpenGL functions that are infinite loops to constantly run 

	glutDisplayFunc(drawGLScene); // keep drawing the stimuli

	glutKeyboardFunc(handleKeypress); // check for keypress

	glutReshapeFunc(handleResize);

	glutIdleFunc(idle);

	glutTimerFunc(TIMER_MS, update, 0);

	glutSetCursor(GLUT_CURSOR_NONE);

	//boost::thread initVariablesThread(&initVariables); 

	glutMainLoop();

	//cleanup();
	return 0;
}
