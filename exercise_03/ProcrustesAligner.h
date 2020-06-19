#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);

		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		// TODO: Compute the transformation matrix by using the computed rotation and translation.
		// You can access parts of the matrix with .block(start_row, start_col, num_rows, num_cols) = elements
		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;
		return estimatedPose;
	}

private:

	//  _._ _..._ .-',     _.._(`))
	// '-. `     '  /-._.-'    ',/
	//    )         \            '.
	//   / \    /    |  LAZY       \
	//  |  0    0    /              |
	//  \   .-.                     ;  
	//   '-('' ).-'       ,'       ;
	//      '-;           |      .'
	//         \           \    /
	//         | 7  .__  _.-\   \
	//         | |  |  ``/  /`  /
	//        /,_|  |   /,_/   /
	MatrixXf vector2Matrix(const std::vector<Vector3f>& points) {
		size_t n = points.size();
		MatrixXf matrix(n, 3);
		for (size_t i = 0; i < n; i++){
			for (size_t j = 0; j < 3; j++){
				matrix(i, j) = points[i][j];
			}
		}
		return matrix;
	}

	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.
		return vector2Matrix(points).colwise().mean() ;
	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm. 
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		MatrixXf product = vector2Matrix(targetPoints).transpose() * vector2Matrix(sourcePoints);
		JacobiSVD<MatrixXf> svd(product, ComputeFullU |  ComputeFullV);
		MatrixXf rotation = svd.matrixU() * svd.matrixV().transpose();
		if (rotation.determinant() == 0){
			MatrixXf identity = MatrixXf::Identity(3, 3);
			identity(2, 2) = -1;
			rotation = svd.matrixU() * identity * svd.matrixV().transpose(); 
		}
		return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target opints.
		return -rotation * sourceMean + targetMean;
	}
};