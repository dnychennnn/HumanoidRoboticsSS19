#include <linear_algebra/LinearAlgebra.h>
#include <Eigen/Dense>
#include <cmath>

namespace linear_algebra {

/**
 * This function should return the vector (2, 1, 3) as an Eigen vector.
 */
Eigen::Vector3d LinearAlgebra::vectorA() {
	Eigen::Vector3d result;

	// TODO: set the vector "result" to (2, 1, 3).

	return result;
}

/**
 * This function should return the vector (-1, -5, 2) as an Eigen vector.
 */
Eigen::Vector3d LinearAlgebra::vectorB() {
	Eigen::Vector3d result;

	// TODO: set the vector "result" to (-1, -5, 2).

	return result;
}

Eigen::Matrix3d LinearAlgebra::matrixM() {
	Eigen::Matrix3d result;

	// TODO: fill in the matrix elements

	return result;
}

Eigen::Matrix3d LinearAlgebra::invMatrixM(const Eigen::Matrix3d& M) {
	Eigen::Matrix3d result;

	// TODO: return the inverse of matrix M

	return result;
}

Eigen::Matrix3d LinearAlgebra::transposeMatrixM(const Eigen::Matrix3d& M) {
	Eigen::Matrix3d result;

	// TODO: return the transpose of matrix M

	return result;
}

double LinearAlgebra::detOfMatrixM(const Eigen::Matrix3d& M)
{
	double result = 0.0;

	// TODO: return the determinant of matrix M

	return result;
}

double LinearAlgebra::dotProduct(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	double result = 0.0;

	// TODO: return the dot product of vectors a and b.

	return result;
}


bool LinearAlgebra::isLinearIndependent(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
	bool result = false;

	/* TODO: test if the vectors a and b are linear independent.
	   Return true if they are independent, false if they are dependent.*/


	return result;
}

Eigen::Vector3d LinearAlgebra::solveLinearSystem(const Eigen::Matrix3d& M, const Eigen::Vector3d& a) {
	Eigen::Vector3d result;

	// TODO: Solve Mx = a for x and return x.

	return result;
}

}  // namespace linear_algebra
