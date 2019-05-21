#include <icp/ICP.h>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <math.h>
#include <Eigen/SVD> 
#include <numeric>


using namespace std;
namespace icp
{
/**
 * \brief Compute the Euclidean distance between a pair of 2D points.
 * \param[in] p1: The first 2D point.
 * \param[in] p2: The second 2D point.
 * \return The Euclidean distance between the two input 2D points.
 */	
	double ICP::distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
	{
		double result = -1.0;
		//TODO: Calculate the distance between the points.
		
		Eigen::Vector2d diff = p1 - p2;
        result = diff.norm();

		return result;
	}
/**
 * \brief Compute the closest point that lies on a given line to a given 2D point.
 * \param[in] pX: The given 2D point, to which we need to compute the closest point that lies on a line.
 * \param[in] pL1: A point that lies on the line.
 * \param[in] pL2: Another point that lies on the line.
 * \return The closest point on the line.
 */
	Eigen::Vector2d ICP::closestPointOnLine(const Eigen::Vector2d& pX, const Eigen::Vector2d& pL1, const Eigen::Vector2d& pL2)
	{
		Eigen::Vector2d result(0., 0.);
		//TODO: Compute the point on the line (pL1-pL2) that is closest to pX.
		Eigen::Vector2d vector12 = pL2 - pL1;
		Eigen::Vector2d vector1X = pX - pL1;

		double magnitude12 = distance(pL1, pL2);
		double dotproduct121X = vector1X.dot(vector12);
		double distance = dotproduct121X / magnitude12;

		result(1) = pL1(1) + (vector12(1)/magnitude12) *distance;
		result(0) = pL1(0) + (vector12(0)/magnitude12) *distance;

		//std::cout << vector12 << "\t" << vector1X << std::endl;		 
		return result;


	}
/**
 * \brief Get the minimum value within vector.
 * \param[in] dist: A vector of values.
 * \return The minimum value.
 */
	double ICP::minDistance(const std::vector<double>& dist)
	{
		double result = 0.0;
		//TODO: Find and return the minimum value in the vector dist
		
		result = *std::min_element(dist.begin(), dist.end());
		
		return result;
	}	
/**
 * \brief Compute the corresponding points in list P to those points in list Q, using the 'closest point' matching method.
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points.
 * \return A vector of the corresponding 2D points matched to points of list Q in list P.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 * The result vector will have the same length as Q.
 */
	StdVectorOfVector2d ICP::euclideanCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P)
	{
		double dist, mindist;
		Eigen::Vector2d best;
		StdVectorOfVector2d result;
		for (icp::StdVectorOfVector2d::const_iterator itQ = Q.begin(); itQ != Q.end(); ++itQ) {
			mindist = std::numeric_limits<double>::max();
			for (icp::StdVectorOfVector2d::const_iterator itP = P.begin(); itP != P.end(); ++itP) {
				dist = distance(*itQ, *itP);
				if (dist < mindist){
					mindist = dist;
					best = *itP;
				}
			}
			result.push_back(best);
		}
		//TODO: Compute corresponding points using the "closest point" method
		return result;
	}
/**
 * \brief Compute the corresponding points in list P to those points in list Q, using the 'point-to-line' matching method .
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points.
 * \return A vector of the corresponding 2D points matched to points of list Q in list P.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::closestPointToLineCorrespondences(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P)
	{
		StdVectorOfVector2d  result;
		//TODO: Compute corresponding points using the "point-to-line" method
		double dist, mindist;
		Eigen::Vector2d linepoint, best;

		for (icp::StdVectorOfVector2d::const_iterator itQ = Q.begin(); itQ != Q.end(); ++itQ) {
			mindist = std::numeric_limits<double>::max();
			for (icp::StdVectorOfVector2d::const_iterator itP = P.begin(); itP != P.end()-1; ++itP) {
				linepoint = closestPointOnLine(*itQ, *itP, *(itP + 1));
				dist = distance(*itQ, linepoint);
				if (dist < mindist) {
					mindist = dist;
					best = linepoint;
				}
			}
			result.push_back(best);
		}

		return result;

	}

	Eigen::Vector2d calcAvg(const StdVectorOfVector2d& V){
		Eigen::Vector2d mean;
		mean << 0, 0;
		for(size_t i=0; i<V.size();i++){
			mean += V[i];
		}
		return mean/ V.size();
	}
/**
 * \brief Compute the affine transformation matrix needed to align the previously computed corresponding points (list C) to the points of list Q.
 * \param[in] Q: A vector of 2D points.
 * \param[in] C: A vector of 2D points, that corresponds to points in list Q.
 * \return An Affine transformation matrix.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	Eigen::Matrix3d ICP::calculateAffineTransformation(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C)
	{
		Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
		//TODO: Compute the affine transformation matrix
		Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
		Eigen::Vector2d Q_center = calcAvg(Q);
		Eigen::Vector2d C_center = calcAvg(C);

		for (size_t i = 0; i < Q.size(); i++) {
			W += Q[i] * C[i].transpose();
		}

		Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix2d R = svd.matrixU()*svd.matrixV().transpose();

		result.topLeftCorner(2,2) << R;
		result.topRightCorner(2, 1) << (Q_center -R* C_center);
		result(2, 2) = 1;

		std::cout << result << std::endl;

		return result;

	}
/**
 * \brief Apply the affine transformation matrix on the points in list P.
 * \param[in] A: Affine transformation matrix.
 * \param[in] P: A vector of 2D points, on which the affine transformation will be applied.
 * \return The vector of transformed points.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::applyTransformation(const Eigen::Matrix3d& A, const StdVectorOfVector2d& P)
    {
        StdVectorOfVector2d  result;
        Eigen::Vector3d P_H;
        Eigen::Vector3d AP_H;
        Eigen::Vector2d AP;
        //TODO: Apply the affine transformation A to the points in list P.
        for (int i = 0; i<P.size(); i++){
            P_H << P[i], 1;
            AP_H = A*P_H;
            AP << AP_H(0)/AP_H(2), AP_H(1)/AP_H(2);
            result.push_back(AP);
        }
        return result;
    }
/**
 * \brief Compute the error between the points in Q list and the transformed corresponding points.
 * \param[in] Q: A vector of 2D points.
 * \param[in] C: A vector of 2D points corresponding to point in Q.
 * \param[in] A: Affine transformation matrix.
 * \return The error of applying the transformation.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	double ICP::computeError(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& C, const Eigen::Matrix3d& A)
	{
		double result = 0.0;
		StdVectorOfVector2d C_prime = applyTransformation(A, C);
		//TODO: Compute the error after the transformation.
		for(size_t i =0; i<C_prime.size(); i++){
			
			result += pow(distance(Q[i], C_prime[i]), 2);
		}
		return result;		
	}


/**
 * \brief Perform one iteration of ICP and prints the error of that iteration.
 * \param[in] Q: A vector of 2D points.
 * \param[in] P: A vector of 2D points, that we need to transform them to be aligned with points in Q list.
 * \param[out] convergenceFlag: A flag passed by reference to determine whether the alignment error has crossed the convergence threshold or not. The flag should be set to 'true' in case of convergence.
 * \param[in] pointToLineFlag: A flag that states the matching method to be used. Its value is set to 'true' when point-to-line method is needed, and to 'false' when closest point method is needed.
 * \param[in] threshold: The maximum value of acceptable error.
 * \return The vector of transformed points.
 *
 * StdVectorOfVector2d is equivalent to std::vector<Eigen::Vector2d>.
 */
	StdVectorOfVector2d ICP::iterateOnce(const StdVectorOfVector2d& Q, const StdVectorOfVector2d& P, bool & convergenceFlag, const bool pointToLineFlag, const double threshold)
	{
		StdVectorOfVector2d result;
		
		//TODO: Perform one iteration of ICP
	
		double error = 0.0;
		//TODO: Compute the error

		if (error <= threshold) {
			convergenceFlag = true;
		}

		return result;
	}


}
