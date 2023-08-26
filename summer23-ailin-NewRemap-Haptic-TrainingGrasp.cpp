// this script aims to use probe adjustment task to measure the gain/strength of either texture or disparity as a depth cue
#include "summer23-ailin-NewRemap-Haptic-TrainingGrasp.h"

double getZ(double shapeHeight, double shapeDepth, double Y) {

	double Z;

	Z = shapeDepth * cos(M_PI * Y / shapeHeight);

	return (Z);
}


double getTg(double shapeHeight, double shapeDepth, double Y) {
	return (-shapeDepth * sin(M_PI * Y / shapeHeight) * M_PI / shapeHeight);
}

float adjustDiffLight(double textDepth, float maxInt, float ambInt, double Depth_flat, double Depth_deep) {
	float newDiff = ambInt;

	if (textDepth > Depth_flat)
		newDiff = newDiff + (textDepth - Depth_flat) / (Depth_deep - Depth_flat) * (maxInt - 2 * ambInt);
	else
		newDiff = newDiff + (textDepth - Depth_flat) / (Depth_deep - textDepth) * (maxInt - 2 * ambInt);

	return newDiff;
}

double NewtonSolver_fz(double z, double Depth, double zCoeff, double distShapeToEye) {
	double val = z / Depth - cos(zCoeff * (z - distShapeToEye));
	return val;
}

double NewtonSolver_dfz(double z, double Depth, double zCoeff, double distShapeToEye) {
	double val = 1 / Depth + sin(zCoeff * (z - distShapeToEye)) * zCoeff;
	return val;
}

double SolveForZ_projected(double theHeight, double newDepth, double distShapeToEye, double y0, double z0) {

	double z_new, z, f_z, df_z;
	double C = M_PI * y0 / (theHeight * (z0 - distShapeToEye));

	z = z0;

	for (int i = 0; i < 100; i++) {

		f_z = NewtonSolver_fz(z, newDepth, C, distShapeToEye);
		df_z = NewtonSolver_dfz(z, newDepth, C, distShapeToEye);

		if (abs(f_z) < 1e-10) {

			break;
		}
		else if (abs(df_z) < 1e-10) {

			break;
		}
		else {
			z_new = z - f_z / df_z;
			z = z_new;
		}
	}

	if (abs(z - z0) > 40)
		z = 0;

	return z;

}

Vector3d projectPoint(double shapeHeight, double newDepth, double distShapeToEye, Vector3d fromPoint) {

	Vector3d ToPoint = fromPoint;
	if (abs(abs(fromPoint.y()) - shapeHeight / 2) > 0.01) {
		double z = SolveForZ_projected(shapeHeight, newDepth, distShapeToEye, fromPoint.y(), fromPoint.z());
		double w = (z - distShapeToEye) / (fromPoint.z() - distShapeToEye);
		ToPoint = Vector3d(w * fromPoint.x(), w * fromPoint.y(), z);
	}

	return ToPoint;
}

double normalCDF(double value)
{
	double M_SQRT1_2 = sqrt(0.5);
	return 0.5 * erfc(-value * M_SQRT1_2);
}

double blurEdge(double dropoff, double pointDist, double intersectDist, double baseCol, double maxCol) {

	// given an Radius, I will choose the intersectDist to be 2/(1+dropoff)
	double addCol = maxCol - baseCol;
	return(((pointDist / intersectDist) < dropoff) ? baseCol : (baseCol + addCol * normalCDF(((pointDist / intersectDist - dropoff) / (1 - dropoff) * 6) - 3)));
}

void scanCurve(double shapeHeight, double shapeDepth, CurveYLMap& output_curve_ylmap) {

	// go through the cosine curve with input height and depth, equally sample nr_curve_map dots and mark the paired y_l values at these points
	output_curve_ylmap = {};

	double y, z, l, y_prev, z_prev;
	double step_size = (shapeHeight / (nr_curve_map - 1));

	output_curve_ylmap.curve_depth = shapeDepth;
	output_curve_ylmap.curve_height = shapeHeight;
	output_curve_ylmap.step_size = step_size;

	// the first point
	y = -shapeHeight / 2;
	z = 0;
	l = 0;
	y_prev = y, z_prev = z;
	output_curve_ylmap.y_vec.push_back(y);
	output_curve_ylmap.l_vec.push_back(l);


	for (int j = 1; j < nr_curve_map; j++) {
		y = -shapeHeight / 2 + j * step_size;
		z = getZ(shapeHeight, shapeDepth, y);
		l = l + sqrt(pow(y - y_prev, 2) + pow(z - z_prev, 2));

		output_curve_ylmap.y_vec.push_back(y);
		output_curve_ylmap.l_vec.push_back(l);

		y_prev = y; z_prev = z;
	}


}

void projectCurve(const CurveYLMap& curve_map_proj, double distShapeToEye, const CurvePtsData& origin_curve, CurvePtsData& output_curve_proj) {

	output_curve_proj = {};

	double newDepth = curve_map_proj.curve_depth;
	double height = curve_map_proj.curve_height;
	double step_y_ylmap = curve_map_proj.step_size;

	output_curve_proj.curve_height = height;
	output_curve_proj.curve_depth = newDepth;

	double y_p, z_p, l_p, tg_p;


	for (int jj = 0; jj < origin_curve.y_vec.size(); jj++) {

		double y_o = origin_curve.y_vec[jj];
		double z_o = origin_curve.z_vec[jj];
		z_p = SolveForZ_projected(height, newDepth, distShapeToEye, y_o, z_o);
		double w = (z_p - distShapeToEye) / (z_o - distShapeToEye);
		y_p = w * y_o;
		int i_c = (y_p - curve_map_proj.y_vec[0]) / step_y_ylmap;
		l_p = curve_map_proj.l_vec[i_c];

		output_curve_proj.y_vec.push_back(y_p);
		output_curve_proj.z_vec.push_back(z_p);
		output_curve_proj.l_vec.push_back(l_p);

	}

}

int buildCurve_byDelY(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve) {

	output_curve = {};

	double depth = input_curve_ylmap.curve_depth;
	output_curve.curve_depth = depth;
	double height = input_curve_ylmap.curve_height;
	output_curve.curve_height = height;
	double y, l;

	double stpsz_J = height / (nr_points_height_default - 1);
	double stpsz_ycurve_precise = height / (nr_curve_map - 1);
	for (int j = 0; j < nr_points_height_default; j++) {
		int k = stpsz_J * j / stpsz_ycurve_precise;
		y = input_curve_ylmap.y_vec[k];
		output_curve.y_vec.push_back(y);
		output_curve.z_vec.push_back(getZ(height, depth, y));
		output_curve.l_vec.push_back(input_curve_ylmap.l_vec[k]);
	}

	return output_curve.y_vec.size();
}

bool generateTexture(float TM_X, float TM_Y, float dotDensity, float dotRadius, float dotSeparationRatio_init, int nr_X_Lattice, float dotJitterScale_Lattice, TextureDotsData& outputTexDots) {

	// the hybrid here refers to employing two methods to generate dots: 
	// method 1: lattice with some jitter
	// method 2: random placement with minimum distance constraint
	// 
	// INPUT:
	// TM_X & TM_Y control the dimension of texture map
	// dotDensity controls how dense the dots are
	// dotSeparationRatio_init controls the minimum distance between two dots, but it is an initial value, it may decrease if it constrains placing all dots in desingiated area
	// nr_X_Lattice controls the number of dots on each row of the lattice
	// dotJitterScale_Lattice controls the scale of jitter of dots on lattice
	// OUTPUT:
	// outputTexDots are TextureDotsData, the most important component is a vector of dot centers (coordinate origin is at the center of the TM) 
	// the dot center x takes value from -TM_X/2 to TM_X/2, the dot center y takes value from -TM_Y/2 to TM_Y/2

	outputTexDots = {};

	outputTexDots.TexMapSize = Vec2{ TM_X, TM_Y };
	outputTexDots.Radius = dotRadius;
	outputTexDots.margin_y = R_intersect_factor * dotRadius;

	std::uniform_real_distribution<float> dist(0.f, 1.f); // require <random>


	vector<Vec2> dc_vec;

	int num_dot = TM_X * TM_Y * dotDensity;

	// prep lattice
	int nr_X = nr_X_Lattice;
	int nr_Y = floor(TM_Y * nr_X / TM_X);
	if (nr_Y % 2 == 0) {
		nr_Y++;
	}
	float lat_step_x = TM_X / (float)nr_X;
	float lat_step_y = TM_Y / (float)nr_Y;



	// step 1: generate lattice dots
	for (int j = 0; j < nr_Y; j++) {
		for (int i = 0; i < nr_X; i++) {
			float rx = dist(rng);
			float ry = dist(rng);
			float cx_lat = ((rx - 0.5) * dotJitterScale_Lattice + i + 0.5) * lat_step_x;
			float cy_lat = ((ry - 0.5) * dotJitterScale_Lattice + j + 0.5) * lat_step_y;

			dc_vec.push_back(Vec2{ cx_lat, cy_lat });
		}
	}


	// step 2: random placement with minimum distance constraint
	float dotSeparationRatio_adjust = dotSeparationRatio_init;
	float dot_separation = dotSeparationRatio_adjust * 2 * (dotRadius * R_intersect_factor);

	int reset_count = 0;
	int dotplacement_runs = 0;

	for (int i_dot = nr_X * nr_Y; i_dot < num_dot; i_dot++) {

		dotplacement_runs++;

		// step 2.1: check whether needs to reset or exit
		// too many unsuccessful placements leads to reset, too many resets leads to exit()
		if (dotplacement_runs > 10000) {

			reset_count++;

			if (reset_count < 10000) {

				//too many unsuccessful placements leads to reset
				dotplacement_runs = 0;
				dc_vec.clear();

				for (int j = 0; j < nr_Y; j++) {
					for (int i = 0; i < nr_X; i++) {

						float rx = dist(rng);
						float ry = dist(rng);
						float cx_lat = ((rx - 0.5) * dotJitterScale_Lattice + i + 0.5) * lat_step_x;
						float cy_lat = ((ry - 0.5) * dotJitterScale_Lattice + j + 0.5) * lat_step_y;

						dc_vec.push_back(Vec2{ cx_lat, cy_lat });
					}
				}

				i_dot = nr_X * nr_Y;

				int adjust_count = floor(reset_count / (float)50); // make one adjustment after repeating fifty attempts
				dotSeparationRatio_adjust = dotSeparationRatio_adjust - 0.000005 * (float)adjust_count * (float)num_dot;
				dot_separation = dotSeparationRatio_adjust * 2 * (dotRadius * R_intersect_factor);
			}
			else {
				return false;
			}

		}


		// step 2.2: placing one dot at a time
		// pick random xy values for the circle center
		float cx = dist(rng); // 0-1
		float cy = dist(rng); // 0-1
		cx *= TM_X;
		cy *= TM_Y;

		// checking whether the current circle intersects withe the previous circles already pushed
		bool intersect = false;
		for (int k = 0; k < dc_vec.size(); k++) {
			Vec2 prev_c = dc_vec[k];
			if ((cx - prev_c.x) * (cx - prev_c.x) + (cy - prev_c.y) * (cy - prev_c.y) < dot_separation * dot_separation) {
				intersect = true;
				break;
			}
		}
		// if intersect, then break and pick a new circle center
		if (intersect) {
			i_dot--;
			continue;
		}

		// if not intersect, add this circle to the circles vector
		dc_vec.push_back(Vec2{ cx, cy });
	}


	// step 3: sort the dots by their y. Get the correponding indices
	int n_dots = dc_vec.size();

	vector<float> dc_y_vec;
	for (int k = 0; k < n_dots; k++) {
		dc_y_vec.push_back(dc_vec[k].y);
	}
	vector<int> sortedDC_ind_vec(n_dots);
	std::iota(sortedDC_ind_vec.begin(), sortedDC_ind_vec.end(), 0); //Initializing
	std::sort(sortedDC_ind_vec.begin(), sortedDC_ind_vec.end(), [&](int i, int j) {return dc_y_vec[i] < dc_y_vec[j]; });



	// step 4: fill in outputTexDots dot center vectors, in the order of from low y to high y
	float x_offset = TM_X / 2.;
	float y_offset = TM_Y / 2.;
	for (int k = 0; k < n_dots; k++) {
		Vec2 dc_temp = dc_vec[sortedDC_ind_vec[k]];
		outputTexDots.dot_center_vec.push_back(Vec2{ dc_temp.x - x_offset, dc_temp.y - y_offset });
	}

	return true;

}

void buildContour_Texture(double ContourWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, float distShapeToEye, ContourData& new_contours_vert) {

	new_contours_vert = {};

	for (int i_v = 0; i_v < dispYCurve.y_vec.size(); i_v++) {

		float x_t_L = -ContourWidth / 2;
		float x_t_R = ContourWidth / 2;

		float z_t = textYCurve.z_vec[i_v];

		float y_d = dispYCurve.y_vec[i_v];
		float z_d = dispYCurve.z_vec[i_v];

		float w = (distShapeToEye - z_d) / (distShapeToEye - z_t);
		//float w = (distShapeToEye - z_d) / (distShapeToEye - (z_t + z_d) / 2.0);
		float x_v_L = w * x_t_L;
		float x_v_R = w * x_t_R;

		new_contours_vert.vert_Lcontour.push_back(Vector3f(x_v_L, y_d, z_d));
		new_contours_vert.vert_Rcontour.push_back(Vector3f(x_v_R, y_d, z_d));
	}
}

void buildVertices_Texture(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnText, VerticesData& vertices_data) {

	vertices_data = {};

	GLuint i_ind = 0;
	int nr_J = dispYCurve.y_vec.size();
	double stpsz_I = shapeWidth / (nr_points_width - 1);
	double height = textYCurve.curve_height;
	double depth_text = textYCurve.curve_depth;
	float x_offset = TexDotsOnText.TexMapSize.x / 2;

	// for texture colors
	float vertex_col = 1.0f;
	int nr_dots = TexDotsOnText.dot_center_vec.size();
	float R = TexDotsOnText.Radius;

	float L_start = -textYCurve.l_vec.back() / 2.;
	float l_margin = TexDotsOnText.margin_y;

	std::uniform_real_distribution<float> dist(0.f, 1.f); // require <random>
	float L_offset = dist(rng) - 0.5f;
	//L_offset = L_offset * (TexDotsOnText.TexMapSize.y - textYCurve.l_vec.back() - 2 * TexDotsOnText.margin_y);
	L_offset = L_offset * 2 * R;
	L_start = L_start + L_offset;

	int TexDot_Ind_L = 0;
	int TexDot_Ind_H = 0;
	vector<Vec2> nearTexDots_dc_vec;

	for (int jj = 0; jj < nr_J; jj++) {

		float y_d = dispYCurve.y_vec[jj];
		float z_d = dispYCurve.z_vec[jj];

		float y_t = textYCurve.y_vec[jj];
		float z_t = textYCurve.z_vec[jj];

		float tg_t = getTg(height, depth_text, y_t);

		float w = (distShapeToEye - z_d) / (distShapeToEye - z_t);
		float x_d;

		double TM_y_t = textYCurve.l_vec[jj] + L_start;

		// find the dots that are near TM_y_t
		while (((TexDotsOnText.dot_center_vec[TexDot_Ind_L].y) < ((float)TM_y_t - l_margin)) && TexDot_Ind_L < (nr_dots - 1)) {
			TexDot_Ind_L++;
		}


		TexDot_Ind_H = TexDot_Ind_L;
		while (((TexDotsOnText.dot_center_vec[TexDot_Ind_H].y) < ((float)TM_y_t + l_margin)) && TexDot_Ind_H < (nr_dots - 1)) {
			TexDot_Ind_H++;
		}

		nearTexDots_dc_vec.clear();
		for (int k = TexDot_Ind_L; k < TexDot_Ind_H + 1; k++) {
			nearTexDots_dc_vec.push_back(TexDotsOnText.dot_center_vec[k]);
		}
		int nr_dots_near = nearTexDots_dc_vec.size();


		for (int ii = 0; ii < nr_points_width; ii++) {

			double pt_x = stpsz_I * ii - x_offset;
			double pt_y = TM_y_t;
			vertex_col = texture_col_max;



			for (int k = 0; k < nr_dots_near; k++) {

				double pt_val = sqrt((pt_x - nearTexDots_dc_vec[k].x) * (pt_x - nearTexDots_dc_vec[k].x) +
					(pt_y - nearTexDots_dc_vec[k].y) * (pt_y - nearTexDots_dc_vec[k].y));


				if (pt_val < R_intersect_factor * R) {
					double vertex_col_tentative = blurEdge(drop_off_rate, pt_val, R_intersect_factor * R, texture_col_min, texture_col_max);
					vertex_col = float(vertex_col_tentative);
					break;
				}

			}

			x_d = w * pt_x;


			vertices_data.vertices_vec.push_back(x_d);
			vertices_data.vertices_vec.push_back(y_d);
			vertices_data.vertices_vec.push_back(z_d);

			vertices_data.light_normals_vec.push_back(0);
			vertices_data.light_normals_vec.push_back(-tg_t);
			vertices_data.light_normals_vec.push_back(1);


			vertices_data.colors_vec.push_back(vertex_col);
			vertices_data.colors_vec.push_back(0);
			vertices_data.colors_vec.push_back(0);

			if (ii < nr_points_width - 1 && jj < nr_J - 1) {

				// using vector
				vertices_data.indices_draw_triangle_vec.push_back(i_ind);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);

				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width + 1);

			}

			i_ind++;
		}
	}

}

bool buildTextureSurface(double shapeWidth, double shapeHeight, double dispDepth, double textDepth, double distShapeToEye, double contourPanelSeparation, VerticesData& vertices_data, ContourData& contours_vert)
{

	// part 1: generate TexDots and project to Disp Surface
	CurveYLMap ylMap_Text;
	scanCurve(shapeHeight, textDepth, ylMap_Text);
	float l_text = ylMap_Text.l_vec.back();


	// part 3: generate surface vertices
	CurvePtsData y_curve_data_text_m, y_curve_data_disp_m;
	nr_points_height = buildCurve_byDelY(ylMap_Text, y_curve_data_text_m);

	if (abs(dispDepth - textDepth) < 0.1) {
		y_curve_data_disp_m = y_curve_data_text_m;
	}
	else {
		CurveYLMap ylMap_Disp;
		scanCurve(shapeHeight, dispDepth, ylMap_Disp);
		projectCurve(ylMap_Disp, distShapeToEye, y_curve_data_text_m, y_curve_data_disp_m);
	}


	//Tex_dot_radius = Tex_dot_radius_base + (float)(rand() % 11) * 0.012;
	TextureDotsData Tex_Dots_text;
	bool texdots_ready = generateTexture(shapeWidth, lengthFactor_TM * l_text, Tex_dot_density, Tex_dot_radius, Tex_dot_separation_ratio, TexDot_Lat_nr, TexDot_Lat_jitter, Tex_Dots_text);

	if (!texdots_ready) {
		beepOk(24);
		current_stage = trial_error;
		current_error = visual_error;
		visibleInfo = true;
		return false;
	}

	buildVertices_Texture(shapeWidth, y_curve_data_disp_m, y_curve_data_text_m, distShapeToEye, Tex_Dots_text, vertices_data);

	buildContour_Texture(contourPanelSeparation, y_curve_data_disp_m, y_curve_data_text_m, distShapeToEye, contours_vert);

	dot_number = Tex_Dots_text.dot_center_vec.size() / lengthFactor_TM;

	return true;

}

void drawTextureSurface(double distShapeToEye, const VerticesData& vertices_data, const ContourData& contours_vert) {

	//setting the light
	glShadeModel(GL_SMOOTH); // enable Smooth Shading
	glEnable(GL_LIGHTING); // enable lighting
	glEnable(GL_LIGHT1);
	glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces	

	// Light source parameters
	GLfloat LightAmbient[] = { light_amb, 0.0f, 0.0f, 1.0f }; // non-directional & overall light (r,g,b,alpha): dark part
	GLfloat LightDiffuse[] = { light_dif, 0.0f, 0.0f, 1.0f }; // light created by the light source (directional light; r,g,b,alpha): bright part
	GLfloat LightPosition[] = { 0.0f, 1.f, lightDir_z, 0.0f }; // Light Position (x, y, z, 1.0f); if w==0, directional; if w==1, positional lights. Attenuation can be applied only to the positional light 

	//setting the light
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient); //setup the ambient light
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse); //setup the diffuse light
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition); //position the light

	glPushMatrix();
	glLoadIdentity();

	glTranslated(visualTarget_X, visualTarget_Y, -distShapeToEye);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	// activate and specify pointer to vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	//using vector
	glVertexPointer(3, GL_FLOAT, 0, &vertices_data.vertices_vec[0]);
	glNormalPointer(GL_FLOAT, 0, &vertices_data.light_normals_vec[0]); //
	glColorPointer(3, GL_FLOAT, 0, &vertices_data.colors_vec[0]);
	glDrawElements(GL_TRIANGLES, vertices_data.indices_draw_triangle_vec.size(), GL_UNSIGNED_INT, &vertices_data.indices_draw_triangle_vec[0]);

	// deactivate vertex arrays after drawing
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	glDisable(GL_LIGHTING);

	drawContours(contours_vert);
	glPopMatrix();

}



void initStimulus(double dispDepth, double textDepth) {

	stimulus_built = false;

	display_distance_jittered = display_distance + jitter_z;
	dist_toEye = -(display_distance_jittered - dispDepth);

	stimulus_built = buildTextureSurface(stimulus_width, stimulus_height, dispDepth, textDepth, dist_toEye, stimulus_visiblewidth, my_verts, my_contour_data);
	
	light_dif = adjustDiffLight(textDepth, max_intensity, light_amb, light_depthMin, light_depthMax);
	dot_number = ratio_visiblewidth_height / ratio_width_height * dot_number;
}



void drawContours(const ContourData& contours_vert) {

	int n;
	float panel_width = 40;
	float panel_height_extra = 20;


	glTranslated(0, 0, 2);
	n = int(contours_vert.vert_Lcontour.size());
	glColor3f(0.0f, 0.0f, 0.0f);
	if (n > 0) {

		// Right panels
		glBegin(GL_QUAD_STRIP);

		glVertex3f(contours_vert.vert_Rcontour.at(0)[0] + panel_width, contours_vert.vert_Rcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Rcontour.at(0)[2]); //0
		glVertex3f(contours_vert.vert_Rcontour.at(0)[0], contours_vert.vert_Rcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Rcontour.at(0)[2]); //1

		for (int i = 0; i < n; i++)
		{
			glVertex3f(contours_vert.vert_Rcontour.at(i)[0] + panel_width, contours_vert.vert_Rcontour.at(i)[1], contours_vert.vert_Rcontour.at(i)[2]); //0
			glVertex3f(contours_vert.vert_Rcontour.at(i)[0], contours_vert.vert_Rcontour.at(i)[1], contours_vert.vert_Rcontour.at(i)[2]); //1

		}

		glVertex3f(contours_vert.vert_Rcontour.at(n - 1)[0] + panel_width, contours_vert.vert_Rcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Rcontour.at(n - 1)[2]); //0
		glVertex3f(contours_vert.vert_Rcontour.at(n - 1)[0], contours_vert.vert_Rcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Rcontour.at(n - 1)[2]); //1

		glEnd();

		// Left panels
		glBegin(GL_QUAD_STRIP);

		glVertex3f(contours_vert.vert_Lcontour.at(0)[0], contours_vert.vert_Lcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Lcontour.at(0)[2]); //0
		glVertex3f(contours_vert.vert_Lcontour.at(0)[0] - panel_width, contours_vert.vert_Lcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Lcontour.at(0)[2]); //1

		for (int i = 0; i < n; i++)
		{
			glVertex3f(contours_vert.vert_Lcontour.at(i)[0], contours_vert.vert_Lcontour.at(i)[1], contours_vert.vert_Lcontour.at(i)[2]); //0
			glVertex3f(contours_vert.vert_Lcontour.at(i)[0] - panel_width, contours_vert.vert_Lcontour.at(i)[1], contours_vert.vert_Lcontour.at(i)[2]); //1

		}

		glVertex3f(contours_vert.vert_Lcontour.at(n - 1)[0], contours_vert.vert_Lcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Lcontour.at(n - 1)[2]); //0
		glVertex3f(contours_vert.vert_Lcontour.at(n - 1)[0] - panel_width, contours_vert.vert_Lcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Lcontour.at(n - 1)[2]); //1

		glEnd();
	}

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


bool change_haptic_object(double hapticDepth) {

	// set the target y and depth
	if (hapticDepth < depth_thresh_flat) { // depth_thresh_flat = 25
		current_bump = flatBump;
		haptic_offset = haptic_offset_flat;
		obj_y_offset = obj_y_offset_flat;
	}
	else if (hapticDepth > depth_thresh_deep) { // depth_thresh_deep = 38
		current_bump = deepBump;
		haptic_offset = haptic_offset_deep;
		obj_y_offset = obj_y_offset_deep;
	}
	else {
		current_bump = mediumBump;
		haptic_offset = haptic_offset_medium;
		obj_y_offset = obj_y_offset_medium;
	}

	bool obj_position_changed = initHaptic_velmex();
	if (!obj_position_changed) {
		return false;
	}

	bool obj_depth_changed = initHaptic_phidget();
	if (!obj_depth_changed) {
		return false;
	}
	else {
		return true;
	}
}

bool updateObjLocation() {

	updateTheMarkers();

	if (isVisible(markers.at(obj_marker).p)) {
		centeredObjEdge = markers.at(obj_marker).p + centeredObjEdge_objM_diff;
		return true;
	}
	else if (isVisible(markers.at(velmex_marker).p)) {
		centeredObjEdge = markers.at(velmex_marker).p + centeredObjEdge_velmexM_diff;
		return true;
	}
	else {
		beepOk(24);
		current_stage = trial_error;
		current_error = blocked_objMarkers;
		visibleInfo = true;
		return false;
	}
}

bool initHaptic_velmex() {

	//current_stage = trial_prep_velmex;

	// check physical objects position, if cannot update position, stop here
	bool ojb_updated_before = updateObjLocation();
	if (!ojb_updated_before)
		return false;

	// move physical objects then check positions again
	moveObject(Vector3d(0, 0 - (centeredObjEdge.y() + obj_y_offset), 0), 6200);
	bool ojb_updated_after = updateObjLocation();
	if (!ojb_updated_after)
		return false;

	// check whether it is moved to the right position
	double error_margin = 3;
	if (abs(centeredObjEdge.y() + obj_y_offset) > error_margin) {
		beepOk(24);
		current_stage = trial_error;
		current_error = velmex_error;
		visibleInfo = true;
		return false;
	}
	else {
		visualTarget_Y = centeredObjEdge.y() + obj_y_offset;
		return true;
	}


}


bool initHaptic_phidget() {

	//current_stage = trial_prep_phidget;
	target_phidget_distance = depth_haptic - haptic_offset;

	if (abs(target_phidget_distance - current_phidget_distance) < 3) {
		moveNum = 1;

		// double intermediate_phidget_distance = max(init_phidget_distance, target_phidget_distance - 3);
		double intermediate_phidget_distance = target_phidget_distance - 3;
		if (intermediate_phidget_distance < init_phidget_distance)
			intermediate_phidget_distance = target_phidget_distance + 3;

		current_phidget_distance = phidgets_linear_move_custom(intermediate_phidget_distance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
	}

	bool phidget_indicator = phidget_move_readjust(target_phidget_distance, max_attempt_good, max_attempt_pass);

	if (!phidget_indicator) {
		beepOk(24);
		current_stage = trial_error;
		current_error = phidget_error;
		visibleInfo = true;
	}

	return phidget_indicator;
}


bool phidget_move_readjust(double setToDistance, int MaxAttempt_good, int MaxAttempt_pass) {

	bool move_complete_indicator = false;

	moveNum = 1;
	current_phidget_distance = phidgets_linear_move_custom(setToDistance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
	movement_offset = current_phidget_distance - setToDistance;

	if (abs(movement_offset) <= moveoffset_thresh_good) {
		move_complete_indicator = true;
	}

	while (!move_complete_indicator) {

		moveNum++;

		double use_thresh = moveoffset_thresh_good;
		if (moveNum > MaxAttempt_good) {
			use_thresh = moveoffset_thresh_pass;
		}

		//double intermediate_phidget_distance = max(init_phidget_distance, target_phidget_distance - 4);
		double intermediate_phidget_distance = target_phidget_distance - 3;
		if (intermediate_phidget_distance < init_phidget_distance)
			intermediate_phidget_distance = target_phidget_distance + 3;
		current_phidget_distance = phidgets_linear_move_custom(intermediate_phidget_distance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
		current_phidget_distance = phidgets_linear_move_custom(setToDistance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
		movement_offset = current_phidget_distance - setToDistance;


		if (abs(movement_offset) <= use_thresh) {
			move_complete_indicator = true;
		}

		if (moveNum > (MaxAttempt_good + MaxAttempt_pass)) {
			move_complete_indicator = false;
			break;
		}
	}

	return move_complete_indicator;
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

	double depth_default = 30;
	stimulus_height = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance - depth_default));
	stimulus_visiblewidth = ratio_visiblewidth_height * stimulus_height;
	stimulus_width = ratio_width_height * stimulus_height;

}



void initBlock()
{
	trial.init(parameters);
	// initialize the trial matrix
	if(session_full_vs_extra){

		repetition = 3;
		totalTrNum = 4 * 8 * repetition;
		trialNum_break = 24;

	}else{

		repetition = 4;
		totalTrNum = 8 * repetition;
		trialNum_break = 16;
	}
	trial.next();

	trialNum = 1;
	
}


void makeParsFileCopy(string filename_original, string filename_copy) {

	ifstream ini_file{ filename_original }; // This is the original file
	ofstream out_file{ filename_copy };
	string line;
	if (ini_file && out_file) {
		while (getline(ini_file, line)) {
			out_file << line << "\n";
		}
	}
	else {
		printf("Cannot read File");
	}
	ini_file.close();
	out_file.close();
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

	targetCueID = str2num<int>(parameters_subj.find("Train_Cue"));
	if(targetCueID > 0){
		reinforce_texture_disparity = false;
	}else{
		reinforce_texture_disparity = true;
	}

	string session = parameters_subj.find("GRASP_Session");
	sessionNum = str2num<int>(session);

	string dirName = experiment_directory + subjectName;
	mkdir(dirName.c_str()); // windows syntax


	// check file to avoid overwrite
	if (util::fileExists(dirName + "/" + subjectName + "_s" + session + "_Grasp.txt") && subjectName != "junk")
	{
		string error_on_file_io = string("file already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.", NULL, NULL);
		shutdown();
	}


	if(sessionNum > 0){
		// only proceed to main training after init prep training is completed
		if (util::fileExists(experiment_directory + "/" + subjectName + "_s0_Grasp.txt") && subjectName != "junk") {

			auto t = std::time(nullptr);
			auto tm = *std::localtime(&t);
			std::ostringstream oss;
			oss << std::put_time(&tm, "%m%d%Y");
			string parametersFileName_copy = dirName + "/" + subjectName + "_ParsCopy_GraspMain_" + oss.str() + ".txt";
			makeParsFileCopy(parametersFileName, parametersFileName_copy);


			session_full_vs_extra = true;
			ifstream parametersFile;
			parametersFile.open(parametersFileName.c_str());
			parameters.loadParameterFile(parametersFile);
			visual_angle = str2num<double>(parameters.find("visualAng"));
		}
		else {
			string error_on_file_io = string("skipped s0. plese complete s0 before the s1");
			cerr << error_on_file_io << endl;
			MessageBox(NULL, (LPCSTR)"s0 is REQUIRED before s1\n Please check the subject's parameters file.", NULL, NULL);
			shutdown();
		}
 
	}else{
		auto t = std::time(nullptr);
		auto tm = *std::localtime(&t);
		std::ostringstream oss;
		oss << std::put_time(&tm, "%m%d%Y");
		string parametersFileName_copy = dirName + "/" + subjectName + "_ParsCopy_GraspPrep_" + oss.str() + ".txt";
		makeParsFileCopy(parametersFileName_prep, parametersFileName_copy);

		session_full_vs_extra = false;
		ifstream parametersFile;
		parametersFile.open(parametersFileName_prep.c_str());
		parameters.loadParameterFile(parametersFile);
		visual_angle = str2num<double>(parameters.find("visualAng"));

	}

	string responseFileName = dirName + "/" + subjectName + "_s" + session + "_Grasp.txt";
	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;

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
	curText.draw("Mirror1 Marker " + stringify< Eigen::Matrix<double, 1, 3> >(markers[mirror1].p.transpose()));
	curText.draw("Mirror2 Marker " + stringify< Eigen::Matrix<double, 1, 3> >(markers[mirror2].p.transpose()));
	if (abs(mirrorAlignment - 45.0) < 0.2)
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("# Mirror Alignment = " + stringify<double>(mirrorAlignment));

	// check if monitor is calibrated
	/*
	if (screenAlignmentY < 89.0)
		glColor3fv(glRed);
	else
		glColor3fv(glGreen);
	curText.draw("# Screen Alignment Y = " + stringify<double>(screenAlignmentY));
	*/
	if (abs(screenAlignmentZ) < 89.0)
		glColor3fv(glRed);
	else
		glColor3fv(glGreen);
	curText.draw("# Screen Alignment Z = " + stringify<double>(screenAlignmentZ));

	curText.draw("--------------------");
	if (isVisible(markers[obj_marker].p))
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	curText.draw("obj Marker " + stringify<int>(obj_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[obj_marker].p.transpose()) + " [mm]");
	curText.draw("velmex_marker " + stringify<int>(velmex_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[velmex_marker].p.transpose()) + " [mm]");
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
	curText.draw(stringify<double>(grip_aperture));
	curText.draw("");

	curText.draw("--------------------");	
	if (isVisible(markers[obj_marker].p) )
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);	
	curText.draw("obj Marker " + stringify<int>(obj_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[obj_marker].p.transpose()) + " [mm]");
	curText.draw("velmex_marker " + stringify<int>(velmex_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[velmex_marker].p.transpose()) + " [mm]");

}


void drawInfo_fingers(GLText curText) {
	curText.draw(" ");
	curText.draw(" ");
	curText.draw(" ");
	curText.draw(" ");
	curText.draw(" ");
	curText.draw(" ");
	curText.draw(" ");
	curText.draw(" ");
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
	//curText.draw("# Vel thresh = " + stringify<double>(thresholdVelthm_steady));
	curText.draw("# handSteady = " + stringify<bool>(handSteady));

	curText.draw(" ");
	curText.draw("# GA = " + stringify<double>(grip_aperture));
	//curText.draw("# gripSmall = " + stringify<bool>(gripSmall));

	curText.draw(" ");
	//curText.draw("# Vel GA tresh = " + stringify<double>(thresholdVelGA_steady));
	//curText.draw("# gripSteady = " + stringify<bool>(gripSteady));
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

			case break_time:
			{
				glColor3fv(glWhite);
				text.draw("Break time! Press + to continue");
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

				break;

			case trial_fixate:
			case trial_visualStimulus:
			case trial_MSE:
			case trial_waitToGrasp:
			case trial_grasp:
			case trial_handReturn:
			{
				text.draw("# current phidget read: " + stringify<double>(current_phidget_distance));
				text.draw(" ");
				text.draw("# current stage: " + stringify<int>(current_stage));
				text.draw("# trial Num: " + stringify<int>(trialNum));
				text.draw("# depth haptic: " + stringify<double>(depth_haptic));

				//text.draw("bump ID " + stringify<int>(current_bump));


				if (isVisible(markers[obj_marker].p))
					glColor3fv(glGreen);
				else
					glColor3fv(glRed);
				text.draw("obj Marker " + stringify<int>(obj_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[obj_marker].p.transpose()) + " [mm]");

				if (isVisible(markers[velmex_marker].p))
					glColor3fv(glGreen);
				else
					glColor3fv(glRed);
				text.draw("velmex Marker " + stringify<int>(velmex_marker) + ":  " + stringify< Eigen::Matrix<double, 1, 3> >(markers[velmex_marker].p.transpose()) + " [mm]");

			}
				break;


			case trial_error:

				switch(current_error){
					case no_error:
						glColor3fv(glRed);
						text.draw("no error");
						break;

					case training_exceeded:
						glColor3fv(glWhite);
						text.draw("Pls call the experimenter!!!!!!!!!!!!! training exceed");
						break;

					case phidget_error:
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
						text.draw("# target obj y: " + stringify<double>(centeredObjEdge.y() + obj_y_offset));
						break;

					case visual_error:
						text.draw("Pls call the experimenter!!!!!!!!!!!!! Rebuild VISUAL stimulus");
						text.draw("# target obj y: " + stringify<double>(centeredObjEdge.y() + obj_y_offset));
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
			drawTextureSurface(dist_toEye, my_verts, my_contour_data);
			//drawFixation(display_distance_jittered);
			
			break;

		case trial_fixate:

			drawFixation(display_distance_jittered);

			break;

		case trial_visualStimulus:
		case trial_MSE:
		case trial_waitToGrasp:
			drawTextureSurface(dist_toEye, my_verts, my_contour_data);
			break;

		case trial_grasp:
			drawThumbTarget();
			drawTextureSurface(dist_toEye, my_verts, my_contour_data);

			break;

		case break_time:
			drawProgressBar();
			break;


	}
}




void initTrial()
{
	current_stage = trial_prep;
	
	//initProjectionScreen(display_distance);

	holdCount_target = 0;
	holdCount_home = 0;
	moveNum = 0;
	attemped_MSE = false;

	if(training){

		depth_mean = depth_training_min + depth_inc * (rand() % 13);
		depth_disp = depth_mean;
		depth_text = depth_mean;
	}
	else {
		depth_mean = trial.getCurrent()["meanDepth"] + (rand() % 3 - 1.0);
		if(session_full_vs_extra){
			depth_delta = trial.getCurrent()["DepthDelta"];
		
			depth_text = depth_mean + depth_delta;
			depth_disp = depth_mean - depth_delta;
		}else{
			depth_text = depth_mean;
			depth_disp = depth_mean;
		}

	}

	if (reinforce_texture_disparity) {
		depth_haptic = depth_text;
	}
	else {
		depth_haptic = depth_disp;
	}
	bool haptic_ready = change_haptic_object(depth_haptic);
	if (!haptic_ready)
		return;
	
	initStimulus(depth_disp, depth_text);
	if (!stimulus_built)
		return;


	trial_timer.reset();				
	trial_timer.start();
	ElapsedTime = 0;
	current_stage = trial_fixate;
	visibleInfo = false;
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
			current_stage = trial_MSE;
		}else{
			// ready for MSE?
			if(handNearHome && gripSmall && gripSteady){
				beepOk(21);
				timestamp_MSEstart = ElapsedTime;
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
	//"subjName\ttrainCue\tmainTraining\tIOD\tblockN\ttrialN\tdisplayDistance\tvisualAngle\tshapeHeight\tshapeWidth\tDepthMean\tDepthDelta\tDepth_disp\tDepth_text\tDepth_haptic\tmvID\tMSE1\tRT_MSE1\tRT_G\tnum_texDots\tradius_texDots\tLAcorrection\tcalibNum";

		responseFile << fixed <<
		subjectName << "\t" <<	
		(int)reinforce_texture_disparity << "\t" <<
		(int)session_full_vs_extra << "\t" <<
		interoculardistance << "\t" <<
		blkNum << "\t" <<
		trialNum << "\t" <<
		display_distance << "\t" <<
		visualTarget_X << "\t" <<
		depth_mean << "\t" <<
		depth_delta << "\t" <<
		depth_disp << "\t" <<
		depth_text << "\t" <<		
		depth_haptic << "\t" <<
		moveNum << "\t" <<
		grip_aperture_MSE << "\t" <<
		timestamp_MSEend - timestamp_MSEstart << "\t" <<
		timestamp_graspend - timestamp_graspstart << "\t" <<
		(int)dot_number << "\t" <<
		Tex_dot_radius << "\t" <<
		liinAct_reset  << "\t" <<
		calibrationNum << endl;

	if(training){
		if(trialNum < trainNum_cap){
			beepOk(4);
			trialNum++;
			initTrial();
		}else{
			beepOk(24);
			trialNum++;
			current_error = training_exceeded;
			current_stage = break_time;
			visibleInfo = true;			
		}

	}else{


		if (!trial.isEmpty()){

			if(trialNum % trialNum_break == 0){
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

		case '1':

			if (current_stage == stimulus_preview) {

				if (depth_disp > depth_inc)
					depth_disp = depth_disp - depth_inc;

				initStimulus(depth_disp, depth_text);

			}

			break;

		case '2':

			if (current_stage == stimulus_preview) {

				depth_disp = depth_disp + depth_inc;

				initStimulus(depth_disp, depth_text);

			}

			break;

		case '4':
			
			if(current_stage == stimulus_preview){
				if (depth_text > depth_inc)
					depth_text = depth_text - depth_inc;

				initStimulus(depth_disp, depth_text);

			}
			
			break;

		case '5':
		
			if(current_stage == stimulus_preview){
				depth_text = depth_text + depth_inc;
				initStimulus(depth_disp, depth_text);
			}

			break;

		case '32': //space bar

			switch(current_error){
				case no_error:
					initTrial();
					break;

				case visual_error:
					Tex_dot_density = Tex_dot_density - 0.01;
					initTrial();
					break;

				case phidget_error:
					liinAct_reset++;
					if(liinAct_reset % 2 == 1)
						current_phidget_distance = phidgets_linear_move_custom(init_phidget_distance, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
					else
						current_phidget_distance = phidgets_linear_move_custom(20, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);

					initTrial();
					break;

				case blocked_objMarkers:
					initTrial();
					break;

				case velmex_error:
					moveObject(Vector3d(0, -10, 0), 6200);
					initTrial();
					break;

			}
			
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

		case 'X':
		case 'x':
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

						if((int)current_error < 1){
							current_stage = trial_waitToGrasp;
						}
					}

				}else{
					
					if((int)current_error < 1){
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

				if(current_error == training_exceeded){
					current_error = no_error;
				}

				visibleInfo = true;
				current_stage = break_time;
			}
			break;


		case 'G':
		case 'g':
			task_guide_info = true;
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
	/*
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
*/
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

				if (grip_aperture > (thresholdGA_small - 2) ) {
					beepOk(16);
					currentInitStep = to_CalibrateFingerTips;
					Fingers_Calibrated = false;
				}else{

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
			}
			else {
				beepOk(3);
			}
			break;

		case to_MoveApparatus:
	
			// push the physical object back to clear the grasping space
			if(isVisible(markers[obj_marker].p) && isVisible(markers.at(velmex_marker).p)){
				beepOk(1);
				centeredObjEdge = markers.at(obj_marker).p + centeredObjEdge_objM_diff;
				centeredObjEdge_velmexM_diff = centeredObjEdge - markers.at(velmex_marker).p;

				// move physical objects then check positions again
				moveObject(Vector3d(0, 0 - centeredObjEdge.y(), display_distance - centeredObjEdge.z()), 6200);

				current_phidget_distance = phidgets_linear_move_custom(depth_haptic - haptic_offset, distance_speed_cutoff, distance_buffer_inc, distance_buffer_dec);
				
				initProjectionScreen(display_distance);

				currentInitStep = to_confirmReady;
			}else{
				beepOk(3);
			}
			break;

		case to_confirmReady:

			bool ojb_updated_after = updateObjLocation();
			if (!ojb_updated_after) {
				beepOk(3);
			}
			else {

				double error_margin = 10;
				Vector3d target_pos(centeredObjEdge.x(), 0, display_distance);
				Vector3d diff = target_pos - centeredObjEdge;
				if (abs(diff.norm()) > error_margin) {
					beepOk(24);
					moveObject(diff, 6200);
				}
				else if (abs(mirrorAlignment - 45.0) > 0.2) {
					beepOk(8);
				}
				else {
					
					initStimulus(depth_disp, depth_text);
					visualTarget_X = centeredObjEdge[0] - stimulus_visiblewidth / 2;
					visualTarget_Y = centeredObjEdge[1];

					thmTarget = centeredObjEdge + Vector3d(-12.5, 0, 0);
					Exp_Initialized = true;
					//visibleInfo = false;
					beepOk(1);
					current_stage = stimulus_preview;
				}
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
		glTranslated(centeredObjEdge.x(), centeredObjEdge.y()+obj_y_offset, centeredObjEdge.z()+ 1.0);
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
