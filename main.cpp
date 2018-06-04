#define _CRT_SECURE_NO_WARNINGS

#include <io.h>
#include <stdio.h>
#include <string.h>
#include <conio.h>
#include <cstdlib>
#include <ctime>

#define GLEW_STATIC
#include <GL/glew.h>

#include <GLFW/glfw3.h>

#include "Shader.h"
#include "Camera.h"
#include "Model.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "SOIL2/SOIL2.h"

#include <dlib/opencv.h>
#include <opencv2/opencv.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include "render_face.hpp"
#include "textModel.h"

//
#include<conio.h>

#define FACE_DOWNSAMPLE_RATIO 4
#define SKIP_FRAMES 2
#define OPENCV_FACE_RENDER
#define LANDMARK_SIZE 68

using namespace std;

double K[9] = { 6.5308391993466671e+002, 0.0, 3.1950000000000000e+002, 0.0, 6.5308391993466671e+002, 2.3950000000000000e+002, 0.0, 0.0, 1.0 };
double D[5] = { 7.0834633684407095e-002, 6.9140193737175351e-002, 0.0, 0.0, -1.3073460323689292e+000 };

void KeyCallback(GLFWwindow *window, int key, int scancode, int action, int mode);

void GetRadius_User(int index, dlib::full_object_detection shape);
void GetRadius_Model(int index, vector<cv::Point2d> points, int Model_index);

void GetVariance_User(int index, dlib::full_object_detection shape);
void GetVariance_Model(cv::Point2d points);

void Get_Normalized_Point_User(float model_radius, cv::Point2d ori_p);
void Get_Normalized_Point_Model(float radius, cv::Point2d ori_p);

int Get_first_index(int index) {
	switch (index) {
	case 0:
		return 36;
	case 1:
		return 42;
	default:
		return 48;
	}
}
int Get_last_index(int index) {
	switch (index) {
	case 0:
		return 42;
	case 1:
		return 48;
	default:
		return 68;
	}
}
int land_marks[68] = { 0 };
float x, y;
// Properties
const GLuint WIDTH = 800, HEIGHT = 600;
int SCREEN_WIDTH, SCREEN_HEIGHT;

// Camera
Camera camera(glm::vec3(0.0f, 10.0f, 14.0f));
bool keys[1024];
bool firstMouse = true;

GLfloat deltaTime = 0.0f;
GLfloat lastFrame = 0.0f;
Model current_model;
vector<textModel> models;	//모든 표정 모델 모아놓은것
vector<float> weights;	//MSE 결과물
float temp_weight = 0.0f;		//MSE 구하기 전단계 변수
bool firstcheck = true;

std::vector<cv::Point3d> model_landmark_3d;	//world좌표계의 model랜드마크 좌표
std::vector<cv::Point2d> model_landmark_2d;	//opencv 좌표상의 model 랜드마크 2차원좌표
std::vector<float> landmark_variances;

cv::Point2d model_center;	//cluster된 landmark들의 중심
cv::Point2d user_center;

cv::Point2d model_temp_point;	//interpolation된 좌표
cv::Point2d user_temp_point;

cv::Point2d model_trans_point;	//평행이동시킨 좌표

float Max_weight, Min_weight;
cv::Point3d init_point;
glm::vec3 init_vertex;

float User_variance = 0.0f;
float Model_variance = 0.0f;

float temp_u_var = 0.0f;
float temp_m_var = 0.0f;

float radius_user = 0.0f;
float radius_model = 0.0f;

int main()
{
	cv::VideoCapture cap(0);
	if (!cap.isOpened())
	{
		std::cout << "Unable to connect to camera" << std::endl;
		return EXIT_FAILURE;
	}
	dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
	dlib::shape_predictor predictor;
	dlib::deserialize("shape_predictor_68_face_landmarks.dat") >> predictor;

	cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, K);
	cv::Mat dist_coeffs = cv::Mat(5, 1, CV_64FC1, D);

	std::vector<cv::Point3d> object_pts;
	object_pts.push_back(cv::Point3d(6.825897, 6.760612, 4.402142));     //#33 left brow left corner
	object_pts.push_back(cv::Point3d(1.330353, 7.122144, 6.903745));     //#29 left brow right corner
	object_pts.push_back(cv::Point3d(-1.330353, 7.122144, 6.903745));    //#34 right brow left corner
	object_pts.push_back(cv::Point3d(-6.825897, 6.760612, 4.402142));    //#38 right brow right corner
	object_pts.push_back(cv::Point3d(5.311432, 5.485328, 3.987654));     //#13 left eye left corner
	object_pts.push_back(cv::Point3d(1.789930, 5.393625, 4.413414));     //#17 left eye right corner
	object_pts.push_back(cv::Point3d(-1.789930, 5.393625, 4.413414));    //#25 right eye left corner
	object_pts.push_back(cv::Point3d(-5.311432, 5.485328, 3.987654));    //#21 right eye right corner
	object_pts.push_back(cv::Point3d(2.005628, 1.409845, 6.165652));     //#55 nose left corner
	object_pts.push_back(cv::Point3d(-2.005628, 1.409845, 6.165652));    //#49 nose right corner
	object_pts.push_back(cv::Point3d(2.774015, -2.080775, 5.048531));    //#43 mouth left corner
	object_pts.push_back(cv::Point3d(-2.774015, -2.080775, 5.048531));   //#39 mouth right corner
	object_pts.push_back(cv::Point3d(0.000000, -3.116408, 6.097667));    //#45 mouth central bottom corner
	object_pts.push_back(cv::Point3d(0.000000, -7.415691, 4.070434));    //#6 chin corner

	std::vector<cv::Point2d> image_pts;

	//result
	cv::Mat rotation_vec;                           //3 x 1
	cv::Mat rotation_mat;                           //3 x 3 R
	cv::Mat translation_vec;                        //3 x 1 T
	cv::Mat pose_mat = cv::Mat(3, 4, CV_64FC1);     //3 x 4 R | T
	cv::Mat euler_angle = cv::Mat(3, 1, CV_64FC1);

	std::vector<cv::Point3d> reprojectsrc;
	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, 10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(10.0, -10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, 10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, 10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, -10.0));
	reprojectsrc.push_back(cv::Point3d(-10.0, -10.0, 10.0));

	reprojectsrc.push_back(cv::Point3d(0, 0, 20));

	
	//reprojected 2D points
	std::vector<cv::Point2d> reprojectdst;
	reprojectdst.resize(8);

	std::vector<cv::Point2d> transed_landmark_2d;
	//temp buf for decomposeProjectionMatrix()
	cv::Mat out_intrinsics = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_rotation = cv::Mat(3, 3, CV_64FC1);
	cv::Mat out_translation = cv::Mat(3, 1, CV_64FC1);

	//text on screen
	std::ostringstream outtext;

	// Init GLFW
	glfwInit();
	// Set all the required options for GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, "Facial Avatar", nullptr, nullptr);

	if (nullptr == window)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();

		return EXIT_FAILURE;
	}

	glfwMakeContextCurrent(window);

	glfwGetFramebufferSize(window, &SCREEN_WIDTH, &SCREEN_HEIGHT);

	glfwSetKeyCallback(window, KeyCallback);


	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	glewExperimental = GL_TRUE;
	if (GLEW_OK != glewInit())
	{
		std::cout << "Failed to initialize GLEW" << std::endl;
		return EXIT_FAILURE;
	}
	double* camera_position = new double[3];
	for (int i = 0; i < 3; i++)
		camera_position[i] = 0;
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);

	//오픈쥐엘 옵션
	glEnable(GL_DEPTH_TEST);

	Shader shader("res/shaders/modelLoading.vs", "res/shaders/modelLoading.frag");
	WIN32_FIND_DATA FindData;
	HANDLE hFind;

	// 기본 모델 셋팅
	Model hair("res/models/claire_OBJ/claire_OBJ/hair.obj");
	Model left_eye("res/models/claire_OBJ/claire_OBJ/left_eye.obj");
	Model right_eye("res/models/claire_OBJ/claire_OBJ/right_eye.obj");
	Model lower_teeth("res/models/claire_OBJ/claire_OBJ/lower_teeth.obj");
	Model upper_teeth("res/models/claire_OBJ/claire_OBJ/upper_teeth.obj");
	Model base("res/models/claire_OBJ/claire_OBJ/face.obj");
	char* path_obj = (char*)malloc(sizeof(char) * 100);
	strcpy(path_obj, "res/text/*.txt");
	hFind = FindFirstFile((LPCSTR)path_obj, &FindData);
	do {
		char* obj_name = (char*)malloc(sizeof(char) * 100);
		strcpy(obj_name, "res/text/");
		strcat(obj_name, FindData.cFileName);
		FILE *fp = fopen(obj_name, "r");
		textModel model;
		
		float tx, ty, tz;
		for (size_t i = 0; i < 8754; i++)
		{
			fscanf(fp, "%f %f %f\n",  &tx, &ty, &tz);
			model.position[i] = glm::vec3(tx, ty, tz);
		}
		models.push_back(model);
		weights.push_back(0.0f);
		fclose(fp);
		cout << obj_name << endl;
		float** temp_landmark = new float*[68];
		for (int j = 0; j < 68; j++) {
			temp_landmark[j] = new float[3];
			for (int k = 0; k < 3; k++)
				temp_landmark[j][k] = 0.0f;
		}
		if (strcmp(FindData.cFileName, "rt_blink.txt") == 0) {
			current_model = base;
			model.index_finding();
			for (int i = 0; i < 68; i++) {
				land_marks[i] = model.land[i];
			}
		}
	} while (FindNextFile(hFind, &FindData));

	FindClose(hFind);
	current_model.All_size(models.size());
	current_model.Set_vert();
	for (int i = 0; i < models.size(); i++) {
		for (int j = 0; j < LANDMARK_SIZE; j++) {
			init_vertex = models[i].position[land_marks[j]];
			init_point.x = init_vertex.x;
			init_point.y = init_vertex.y;
			init_point.z = init_vertex.z;
			model_landmark_3d.push_back(init_point);
			landmark_variances.push_back(0.0f);
		}
	}

	glm::mat4 projection = glm::perspective(camera.GetZoom(), (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 0.1f, 100.0f);

	glm::vec3 init_trans = { 0.0f, -3.0f, 0.0f };
	glm::vec3 init_scale = { 0.5f, 0.5f, 0.5f };

	/////////////////
	////////////////

	// Game loop
	while (!glfwWindowShouldClose(window))
		////////////////
	{
		GLfloat currentFrame = glfwGetTime();
		deltaTime += 0.2;
		lastFrame = currentFrame;

		// Check and call events

		// Clear the colorbuffer
		glClearColor(255.00f, 255.00f, 255.00f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		shader.Use();
		projection = glm::perspective(camera.GetZoom(), (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 0.1f, 100.0f);
//		glm::mat4 view = camera.GetViewMatrix();
		glm::mat4 view;
		double length = 0.0;
		for (int i = 0; i < 3; i++) {
			length += pow(camera_position[i], 2);
		}
		length = sqrt(length);
		if (firstcheck) {
		//	view = camera.GetViewMatrix();
			view = glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 2.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
		}
		else
			view = glm::lookAt(glm::vec3((camera_position[0] / length) * -15.0f, (camera_position[1] / length) * 15.0f, (camera_position[2] /length)* 15.0f), glm::vec3(0, 3, 0), glm::vec3(0, 1, 0));
		glUniformMatrix4fv(glGetUniformLocation(shader.Program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
		glUniformMatrix4fv(glGetUniformLocation(shader.Program, "view"), 1, GL_FALSE, glm::value_ptr(view));

		// Draw the loaded model
		glm::mat4 model;
		model = glm::translate(model, init_trans);
		model = glm::scale(model, init_scale);
		glUniformMatrix4fv(glGetUniformLocation(shader.Program, "model"), 1, GL_FALSE, glm::value_ptr(model));
		
		hair.Draw(shader);
		left_eye.Draw(shader);
		right_eye.Draw(shader);
		lower_teeth.Draw(shader);
		upper_teeth.Draw(shader);
		current_model.Draw(shader);
		glfwPollEvents();

		// Swap the buffers
		glfwSwapBuffers(window);

		cv::Mat temp;
		cap >> temp;
		dlib::cv_image<dlib::bgr_pixel> cimg(temp);

		// Detect faces
		std::vector<dlib::rectangle> faces = detector(cimg);

		// Find the pose of each face
		if (faces.size() > 0)
		{
			//track features
			dlib::full_object_detection shape = predictor(cimg, faces[0]);

			//draw features
			for (unsigned int i = 0; i < 68; ++i)
			{
				cv::circle(temp, cv::Point(shape.part(i).x(), shape.part(i).y()), 2, cv::Scalar(0, 0, 255), -1);
			}

			image_pts.push_back(cv::Point2d(shape.part(17).x(), shape.part(17).y())); //#17 left brow left corner
			image_pts.push_back(cv::Point2d(shape.part(21).x(), shape.part(21).y())); //#21 left brow right corner
			image_pts.push_back(cv::Point2d(shape.part(22).x(), shape.part(22).y())); //#22 right brow left corner
			image_pts.push_back(cv::Point2d(shape.part(26).x(), shape.part(26).y())); //#26 right brow right corner
			image_pts.push_back(cv::Point2d(shape.part(36).x(), shape.part(36).y())); //#36 left eye left corner
			image_pts.push_back(cv::Point2d(shape.part(39).x(), shape.part(39).y())); //#39 left eye right corner
			image_pts.push_back(cv::Point2d(shape.part(42).x(), shape.part(42).y())); //#42 right eye left corner
			image_pts.push_back(cv::Point2d(shape.part(45).x(), shape.part(45).y())); //#45 right eye right corner
			image_pts.push_back(cv::Point2d(shape.part(31).x(), shape.part(31).y())); //#31 nose left corner
			image_pts.push_back(cv::Point2d(shape.part(35).x(), shape.part(35).y())); //#35 nose right corner
			image_pts.push_back(cv::Point2d(shape.part(48).x(), shape.part(48).y())); //#48 mouth left corner
			image_pts.push_back(cv::Point2d(shape.part(54).x(), shape.part(54).y())); //#54 mouth right corner
			image_pts.push_back(cv::Point2d(shape.part(57).x(), shape.part(57).y())); //#57 mouth central bottom corner
			image_pts.push_back(cv::Point2d(shape.part(8).x(), shape.part(8).y()));   //#8 chin corner

		
			cv::solvePnP(object_pts, image_pts, cam_matrix, dist_coeffs, rotation_vec, translation_vec);
			cv::projectPoints(reprojectsrc, rotation_vec, translation_vec, cam_matrix, dist_coeffs, reprojectdst);
			cv::projectPoints(model_landmark_3d, rotation_vec, translation_vec, cam_matrix, dist_coeffs, model_landmark_2d);
			for (int i = 0; i < model_landmark_2d.size(); i++) {
				cv::Point2d p(0, 0);
				transed_landmark_2d.push_back(p);
			}

			cv::Mat R;	//카메라의 방향
			cv::Rodrigues(rotation_vec, R);
			cv::Mat R_inv = R.inv();
			cv::Mat P = -R_inv*translation_vec;
			camera_position = (double *)P.data;

			/////Mean Square Error////// 눈부분 입부분 3개 따로따로
			if (deltaTime < 0.3) {
				for (int i = 0; i < 3; i++) {
					GetRadius_User(i, shape);
					int first = Get_first_index(i);
					int last = Get_last_index(i);
					for (int j = 0; j < models.size(); j++) {
						weights[j] = 0.0f;
						GetRadius_Model(i, model_landmark_2d, j);
						GetVariance_User(i, shape);
						User_variance = temp_u_var;
						Model_variance = 0.0f;
						for (int k = first; k < last; k++) {
							model_trans_point.x = model_landmark_2d[j * LANDMARK_SIZE + k].x + (user_center.x - model_center.x);
							model_trans_point.y = model_landmark_2d[j * LANDMARK_SIZE + k].y + (user_center.y - model_center.y);
							Get_Normalized_Point_Model(radius_user / radius_model, model_trans_point);
							transed_landmark_2d[j * LANDMARK_SIZE + k] = model_temp_point;

							GetVariance_Model(model_temp_point);
							Model_variance += temp_m_var;
						}
						weights[j] = 1.0f / pow(User_variance - Model_variance, 2);
					}

					Max_weight = weights[0];
					Min_weight = weights[0];

					for (int k = 0; k < weights.size(); k++) {
						if (Max_weight < weights[k]) {
							Max_weight = weights[k];
						}
						if (Min_weight > weights[k]) {
							Min_weight = weights[k];
						}
					}

					for (int k = 0; k < weights.size(); k++) {
						weights[k] = (2.0f / (Max_weight - Min_weight))*(weights[k] - Min_weight);
					}
					current_model.blending(models, weights, i);
				}
			}
			else if (deltaTime < 1.0f) {
				current_model.Interpolation(deltaTime, 1.0);
			}
			else {
				deltaTime = 0.0f;
			}
			
		//	camera.faceMoving(((shape.part(36).x() + shape.part(45).x()) / 2) - ((reprojectdst[0].x + reprojectdst[7].x + reprojectdst[4].x + reprojectdst[3].x) / 4), ((shape.part(36).y() + shape.part(45).y()) / 2) - (reprojectdst[0].y + reprojectdst[3].y + reprojectdst[4].y + reprojectdst[7].y) / 4);
			firstcheck = false;
			image_pts.clear();
		}
		cv::imshow("Face Maker", temp);

		
		unsigned char key = cv::waitKey(1);
		if (key == 27)
		{
			break;
		}

	}

	glfwTerminate();
	return 0;
}

void KeyCallback(GLFWwindow *window, int key, int scancode, int action, int mode)
{
	if (GLFW_KEY_ESCAPE == key && GLFW_PRESS == action)
	{
		glfwSetWindowShouldClose(window, GL_TRUE);
	}

	if (key >= 0 && key < 1024)
	{
		if (action == GLFW_PRESS)
		{
			keys[key] = true;
		}
		else if (action == GLFW_RELEASE)
		{
			keys[key] = false;
		}
	}
}


void GetRadius_User(int index, dlib::full_object_detection shape) {
	radius_user = 0.0f;
	user_center.x = 0.0f;
	user_center.y = 0.0f;
	int first = Get_first_index(index);
	int last = Get_last_index(index);
	for (int k = first; k < last; k++) {
		user_center.x += shape.part(k).x();
		user_center.y += shape.part(k).y();
	}
	user_center.x /= (float)(last - first);
	user_center.y /= (float)(last - first);
	for (int k = first; k < last; k++) {
		if (radius_user < sqrt(pow(user_center.x - shape.part(k).x(), 2) + pow(user_center.y - shape.part(k).y(), 2))) {
			radius_user = sqrt(pow(user_center.x - shape.part(k).x(), 2) + pow(user_center.y - shape.part(k).y(), 2));
		}
	}
}
void GetRadius_Model(int index, vector<cv::Point2d> points, int Model_index) {
	radius_model = 0.0f;
	model_center.x = 0.0f;
	model_center.y = 0.0f; 
	int first = Get_first_index(index);
	int last = Get_last_index(index);
	for (int k = first; k < last; k++) {
		model_center.x += points[Model_index * LANDMARK_SIZE + k].x;
		model_center.y += points[Model_index * LANDMARK_SIZE + k].y;
	}
	model_center.x /= (float)(last - first);
	model_center.y /= (float)(last - first);
	for (int k = first; k < last; k++) {
		if (radius_model < sqrt(pow(model_center.x - points[Model_index * LANDMARK_SIZE + k].x, 2) + pow(model_center.y - points[Model_index * LANDMARK_SIZE + k].y, 2))) {
			radius_model = sqrt(pow(model_center.x - points[Model_index * LANDMARK_SIZE + k].x, 2) + pow(model_center.y - points[Model_index * LANDMARK_SIZE + k].y, 2));
		}
	}
}
void GetVariance_User(int index, dlib::full_object_detection shape) {
	temp_u_var = 0.0f;
	int first = Get_first_index(index);
	int last = Get_last_index(index);
	for (int i = first; i < last; i++) {
		temp_u_var += sqrt(pow(shape.part(i).x() - user_center.x, 2) + pow(shape.part(i).y() - user_center.y, 2));
	}
}
void GetVariance_Model(cv::Point2d point) {
	temp_m_var = 0.0f;
	temp_m_var = sqrt(pow(point.x - user_center.x, 2) + pow(point.y - user_center.y, 2));
}
void Get_Normalized_Point_User(float model_radius, cv::Point2d ori_p) {
	user_temp_point.x = model_radius*(ori_p.x - user_center.x) + user_center.x;
	user_temp_point.y = model_radius*(ori_p.y - user_center.y) + user_center.y;
}
void Get_Normalized_Point_Model(float radius, cv::Point2d ori_p) {
	model_temp_point.x = radius*(ori_p.x - user_center.x) + user_center.x;
	model_temp_point.y = radius*(ori_p.y - user_center.y) + user_center.y;
}