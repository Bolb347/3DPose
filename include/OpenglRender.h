#pragma once

#include <vector>
//#include <iostream>
#include <string>

#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#define GLEW_STATIC
#include <GL/glew.h>
#include <opencv2/core.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "defines.h"
#include "utility.h"
#include "model_buffer.h"
#include "ModelImporter.h"

class OpenGLRender
{
public:
	OpenGLRender(CameraParameters const& in_camParams);
	~OpenGLRender();
	cv::Mat getColorImgFromBuff();
	cv::Mat getDepthImgFromBuff();
	void renderColorToFrontBuff(uint16_t in_modelIndice, glm::vec3 camPositon, float in_rotate = 0.0f,
	                            float in_x = 0.0f, float in_y = 0.0f);
	void renderColorToFrontBuff(uint16_t in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec);
	void renderDepthToFrontBuff(uint16_t in_modelIndice, glm::vec3 camPositon, float in_rotate = 0.0f,
	                            float in_x = 0.0f, float in_y = 0.0f);
	void renderDepthToFrontBuff(uint16_t in_modelIndice, glm::mat4 in_rotMat, glm::vec3 in_traVec);
	void creatModBuffFromFiles(std::string const& in_modelLocation);
	void readModelFile(std::string const& in_file, Model& in_model);

private:
	SDL_Window* window;
	std::vector<ModelBuffer> modBuff;
	cv::Mat renderedColorImg;
	cv::Mat renderedDepthImg;
	uint16_t width;
	uint16_t height;
	float fieldOfView;
	glm::vec3 position;
	glm::mat4 projection;
	glm::mat4 view;
	glm::mat4 modelMat;
	glm::mat4 viewProj;
	glm::vec3 up;
	GLuint colorShaderProgram;
	GLuint depthShaderProgram;
	GLuint fbo;
	GLuint modelViewProjMatrixLocationColor;
	GLuint modelViewProjMatrixLocationDepth;
	float cx;
	float cy;

	void setupSDLWindow();
	void setupOpenGL();
	void setupFramebuffer();
	void setupShader();
	std::vector<glm::vec3> zipVectors(const std::vector<glm::vec3>& a, const std::vector<glm::vec3>& b);
	void translateCam(glm::vec3 in_vec, float in_rotate, float in_x, float in_y);
};
