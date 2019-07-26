#pragma once
#include "defines.h"
#include <vector>
#include <glm/glm.hpp>

const float goldenRatio = 1.61803398875;

class CameraViewPoints
{
	float radius;
	float icosahedronPointA;
	float icosahedronPointB;
	uint8_t numSubdivisions;

	std::vector<glm::vec3> vertices;
	std::vector<Index> indices;
public:
	CameraViewPoints();
	~CameraViewPoints();

	void createCameraViewPoints(float in_radius, uint8_t in_subdivions);
	void readModelProperties(std::string in_modelFile);

	uint32_t getNumVertices();
	uint32_t getNumIndices();
	std::vector<glm::vec3>& getVertices();
	std::vector<Index>& getIndices();

private:
	ModelProperties modProps;

	void removeSuperfluousVertices();
	void icosahedronPointsFromRadius();
	void createVerticesForRotSym();
	void createIcosahedron();
	int32_t checkForDuplicate(uint32_t vertSize);
	void subdivide();
	void adjustVecToRadius(uint32_t index);
};
