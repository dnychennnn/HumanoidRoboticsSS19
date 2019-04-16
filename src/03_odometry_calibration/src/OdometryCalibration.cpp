#include <odometry_calibration/OdometryCalibration.h>
#include <cmath>
#include <iostream>

namespace odometry_calibration {

/**
 * \brief Computes the odometry error function.
 * \param[in] groundTruth The ground truth odometry measured by an external sensor (called u* in the slides).
 * \param[in] observation The odometry measurement observed by the robot itself, e.g., using wheel sensors or joint angles (called u in the slides).
 * \param[in] calibrationMatrix The calibration matrix of the current iteration (called X in the slides).
 * \return The error function vector (called e in the slides).
 */
Eigen::Vector3d OdometryCalibration::errorFunction(const Odometry& groundTruth, const Odometry& observation, const Eigen::Matrix3d& calibrationMatrix) {
	Eigen::Vector3d error;
	//TODO: Compute the error vector. 
	Eigen::Vector3d prediction;
	error << groundTruth.ux, groundTruth.uy, groundTruth.utheta;
	prediction << observation.ux, observation.uy, observation.utheta;
	prediction = calibrationMatrix * prediction;
	error = error - prediction;

	return error;
}

/**
 * \brief Computes the Jacobian (matrix derivative) of the error function for a given odometry measurement.
 * \param[in] measurement The odometry measured by the robot (called u in the slides).
 * \return The Jacobian matrix of the error function.
 */
Eigen::Matrix3Xd OdometryCalibration::jacobian(const Odometry& observation) {
	Eigen::Matrix3Xd jacobian(3, 9);
	//TODO: Calculate the 3x9 Jacobian matrix of the error function for the given observation.
	
	jacobian <<  observation.ux, observation.uy, observation.utheta, 0, 0, 0, 0, 0, 0,
				0, 0, 0, observation.ux, observation.uy, observation.utheta, 0, 0, 0,
				0, 0, 0, 0, 0, 0, observation.ux, observation.uy, observation.utheta;

	return jacobian;
}

/**
 * \brief Calibrates the odometry of a robot.
 * \param[in] measurementData A vector containing ground truth and observed odometry measurements.
 * ŗeturn The calibration matrix that can be used to correct the odometry.
 */
Eigen::Matrix3d OdometryCalibration::calibrateOdometry(const std::vector<MeasurementData>& measurements) {
	Eigen::Matrix3d calibrationMatrix = Eigen::Matrix3d::Identity();
	/** TODO: Calculate the calibration matrix. The steps to do are:
	 * - Start with an arbitrary initial calibration matrix
	 * - Iterate through the calibration data
	 * - Compute the error function and Jacobian for each data set
	 * - Accumulate the linear system components H and b
	 * - Solve the linear system
	 * - Update the calibration matrix
	 */
	for(int i=0; i<measurements.size(); i++){
		auto error_i = errorFunction(measurements[i].groundTruth, measurements[i].uncalibrated, calibrationMatrix);
		auto J_i = jacobian(measurements[i].uncalibrated);
	}

	


	return calibrationMatrix;
}

/**
 * \brief Applies the calibration matrix to an odometry measurement in order to get a corrected estimate.
 * \param[in] uncalibratedOdometry An uncalibrated odometry measurement (called u in the slides).
 * \param[in] calibrationMatrix The calibration matrix computed by calibrateOdometry in a previous step (called X in the slides).
 * \return The corrected odometry estimate.
 */
Odometry OdometryCalibration::applyOdometryCorrection(const Odometry& uncalibratedOdometry, const Eigen::Matrix3d& calibrationMatrix) {
	Odometry calibratedOdometry;
	/*TODO: Given the calibration matrix, return the corrected odometry measurement so that the robot has
	 * a better estimate of the location where it is currently.
	 */
	Eigen::Vector3d uncalibrated;
	Eigen::Vector3d calibrated;

	uncalibrated << uncalibratedOdometry.ux, uncalibratedOdometry.uy, uncalibratedOdometry.utheta;
	
	calibrated = calibrationMatrix * uncalibrated;

	calibratedOdometry.ux = calibrated(0);
	calibratedOdometry.uy = calibrated(1);
	calibratedOdometry.utheta = calibrated(2);

	return calibratedOdometry;
}

/**
 * \brief Converts an odometry reading into an affine 2D transformation.
 * \param[in] odometry The odometry reading.
 * \returns The corresponding affine transformation as a 3x3 matrix.
 */
Eigen::Matrix3d OdometryCalibration::odometryToAffineTransformation(const Odometry& odometry) {
	Eigen::Matrix3d transformation;
	//TODO: Convert the odometry measurement to an affine transformation matrix.

	transformation << cos(odometry.utheta), -sin(odometry.utheta), odometry.ux,
					sin(odometry.utheta), cos(odometry.utheta), odometry.uy,
					0.0, 0.0, 1.0;

	return transformation;
}

/**
 * \brief Converts an affine 2D transformation matrix into a 2D robot pose (x, y, and theta).
 * \param[in] transformation An affine transformation as a 3x3 matrix.
 * \returns The corresponding 2D pose (x, y, and theta).
 */
Pose2D OdometryCalibration::affineTransformationToPose(const Eigen::Matrix3d& transformation) {
	Pose2D pose;
	/* TODO: replace the following lines by the x and y position and the rotation of the robot.
	 * Hint: x and y can be directly read from the matrix. To get the rotation, use the acos/asin
	 * functions on the rotation matrix and take extra care of the sign, or (better) use the
	 * atan2 function.
	 */
	pose.x = 0.0;
	pose.y = 0.0;
	pose.theta = 0.0;

	pose.x += transformation(0,2);
	pose.y += transformation(1,2);
	pose.theta = atan2(transformation(1,0), transformation(0,0));

	return pose;
}
/**
 * \brief Calculate the robot's trajectory in Cartesian coordinates given a list of calibrated odometry measurements.
 * \param[in] calibratedOdometry Odometry measurements that have already been corrected using the applyOdometryCorrection method.
 * \returns A vector of 2D poses in the global coordinate system.
 */
std::vector<Pose2D> OdometryCalibration::calculateTrajectory(const std::vector<Odometry>& calibratedOdometry) {
	std::vector<Pose2D> trajectory;
	/* TODO: Compute the trajectory of the robot.
	 * - Start at the position x = 0, y = 0, theta = 0. (Do not add this point to the trajectory).
	 * - Iterate through the odometry measurements.
	 * - Convert each odometry measurement to an affine transformation using the
	 *   odometryToAffineTransformation method from above.
	 * - Chain the affine transformation to get the next pose.
	 * - Convert the affine transformation back to a 2D pose using the
	 *   affineTransformationToPose method from above.
	 * - Store the pose in the trajectory vector.
	 */

	Eigen::Matrix3d affine = Eigen::Matrix3d::Identity(3, 3);

	Pose2D pose;

	for( auto co:calibratedOdometry){
		
		std::cout << odometryToAffineTransformation(co) << std::endl;
		affine *= odometryToAffineTransformation(co);
		std::cout << affine << std::endl;
		pose = affineTransformationToPose(affine);
		trajectory.push_back(pose);
	
	} 


	return trajectory;
}




} /* namespace odometry_calibration */
