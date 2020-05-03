#include <iostream>
#include <fstream>
#include <array>
#include <vector> 

#include "Eigen.h"

#include "VirtualSensor.h"

struct Face{
	int x;
	int y;
	int z;
};

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool IsFace(Vertex* vertices, int i, int j, int k, const float threshold)
{
	return (vertices[i].position - vertices[j].position).norm() < threshold && 
		   (vertices[i].position - vertices[k].position).norm() < threshold &&
		   (vertices[j].position - vertices[k].position).norm() < threshold;
}

bool IsValidVertex(const Vertex& vertex)
{
	return vertex.position[0] != MINF && 
		   vertex.position[1] != MINF &&
		   vertex.position[2] != MINF;
}

void WriteFace(std::ofstream& outFile, int i, int j, int k){
	outFile << 3 << " "
			<< i << " " 
			<< j << " " 
			<< k << std::endl;
}

void WriteVertex(std::ofstream& outFile, const Vertex& vertex)
{
	outFile << vertex.position[0] << " " 
			<< vertex.position[1] << " " 
			<< vertex.position[2] << " " 
			<< (int)vertex.color[0] << " " 
			<< (int)vertex.color[1] << " " 
			<< (int)vertex.color[2] << " " 
			<< (int)vertex.color[3] << std::endl;
}

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	
	std::vector<Face> faces;
	for (size_t i = 0; i < height - 1; i++)
	{
		for (size_t j = 0; j < width - 1; j++)
		{
			int topLeft = i * width + j;
			int bottomLeft = (i + 1) * width + j;
			int topRight = topLeft + 1;
			int bottomRight = bottomLeft + 1;
			if (IsFace(vertices, topLeft, bottomLeft, topRight, edgeThreshold))
			{
				faces.push_back({topLeft, bottomLeft, topRight});
			}
			if (IsFace(vertices, topLeft, bottomRight, topRight, edgeThreshold))
			{
				faces.push_back({topLeft, bottomRight, topRight});
			}
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << faces.size() << " 0" << std::endl;

	// TODO: save vertices
	
	for (size_t i = 0; i < width * height; i++)
	{	
		if (IsValidVertex(vertices[i]))
		{
			WriteVertex(outFile, vertices[i]);
		}else {
			WriteVertex(outFile, {Vector4f(0, 0, 0, 0), vertices[i].color});
		}
	}

	// TODO: save valid faces
	for (size_t i = 0; i < faces.size(); i++){
		WriteFace(outFile, faces[i].x, faces[i].y, faces[i].z);
	}


	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "../../data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "meshes/mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap

		MatrixXf identity = MatrixXf::Identity(4, 3);
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		int depth_height = sensor.GetDepthImageHeight();
		int depth_width = sensor.GetDepthImageWidth();
		Vertex* vertices = new Vertex[depth_height * depth_width];
		for (size_t i = 0; i < depth_height; i++) {
			for (size_t j = 0; j < depth_width; j++){
				size_t idx = i * depth_width + j;
				if (depthMap[idx] == MINF){
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				}else {
					Vector3f pImage(j * depthMap[idx], i * depthMap[idx], depthMap[idx]);
					Vector4f pCamera = identity * depthIntrinsicsInv * pImage;
					Vector4f pWorld = trajectoryInv * depthExtrinsicsInv * pCamera;
					vertices[idx].position = pWorld;
					vertices[idx].color = Vector4uc(
						colorMap[4 * idx + 0], colorMap[4 * idx + 1],
						colorMap[4 * idx + 2], colorMap[4 * idx + 3]);	
				}
			}
		}		

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
