#include <inverse_kinematics/InverseKinematics.h>
#include <iostream>
#include <angles/angles.h>

namespace inverse_kinematics {

/**
 * Computes the Moore-Penrose inverse (pseudo inverse) of a Jacobi matrix.
 * @param a the Jacobi matrix
 * @return the pseudo inverse.
 */
Eigen::MatrixXd pseudoInverse(const Jacobian &a)
{
	// This method is fully implemented, nothing to do here.
	// You can use this method in your code.

	const double epsilon = std::numeric_limits<Jacobian::Scalar>::epsilon();

	if (a.rows() < a.cols()) {
		throw std::runtime_error("cannot compute pseudoinverse because the system is underdetermined");
	}

	const Eigen::JacobiSVD<Jacobian> svd = a.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

	const Jacobian::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

	return svd.matrixV() * Jacobian((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().
		array().inverse(), 0)).asDiagonal() * svd.matrixU().adjoint();
}

/**
 * \brief return a homogeneous matrix representing a rotation in 2D.
 * \param[in] angle The rotation angle.
 * \return Homogeneous transformation matrix as an Eigen::Matrix3d.
 */
Cartesian2DTransform InverseKinematics::rotation(const double& angle) const {
	Cartesian2DTransform result = Eigen::Matrix3d::Zero();
	result << cos(angle), -sin(angle), 0,
			  sin(angle),  cos(angle), 0,
			           0,           0, 1;
	return result;
}

/**
 * \brief return a homogeneous matrix representing a translation in 2D.
 * \param[in] distance_x The translation distance in x direction.
 * \param[in] distance_y The translation distance in y direction.
 * \return Homogeneous transformation matrix as an Eigen::Matrix3d.
 */
Cartesian2DTransform InverseKinematics::translation(const double& distance_x, const double& distance_y) const {
	Cartesian2DTransform result = Eigen::Matrix3d::Zero();
	result << 1, 0, distance_x,
			  0, 1, distance_y,
			  0, 0, 1;
	return result;
}

/**
 * \brief Computes the endeffector pose from the current joint angles.
 * \param[in] q The current joint angles (as an Eigen::VectorXd of size 2).
 * \return The pose (e_x, e_y) of the hand (as an Eigen::VectorXd of size 2).
 */
EndeffectorPose InverseKinematics_2Links::forwardKinematic(const JointAngles& q) const {
	EndeffectorPose e = Eigen::VectorXd::Zero(2);
	// TODO: Calculate the forward kinematics.

	/* Use the class member variables a0, a1, and h for the lengths given on the exercise sheet. */
	Eigen::Vector3d v;
	v << h, 0, 1;

	Eigen::Vector3d e_hom;
	e_hom = rotation(q(0))*translation(a0,0)*rotation(q(1))*translation(a1,0)*v;
	e(0) = e_hom(0);
	e(1) = e_hom(1);

	return e;
}

/**
 * \brief Computes the Jacobian of the endeffector pose with respect to the joint angles.
 * \param[in] q The current joint angles as an Eigen::VectorXd of length 2.
 * \return The Jacobian as an Eigen::MatrixXd of size 2x2.
 */
Jacobian InverseKinematics_2Links::jacobian(const JointAngles& q) const {
	Jacobian result = Eigen::MatrixXd::Zero(2, 2);
	/* TODO: Fill in the Jacobian that you calculated by hand. */

	/* Use the class member variables a0, a1, and h for the lengths given on the exercise sheet. */
	result << -a0*sin(q(0))-(a1+h)*(sin(q(0))*cos(q(1))+cos(q(0))*sin(q(1))), -(a1+h)*(cos(q(0))*sin(q(1))+sin(q(0))*cos(q(1))),
			   a0*cos(q(0))+(a1+h)*(cos(q(0))*cos(q(1))-sin(q(0))*sin(q(1))),  (a1+h)*(-sin(q(0))*sin(q(1))+cos(q(0))*cos(q(1)));

	return result;
}

/**
 * \brief Computes a small change that moves the endeffector towards the goal.
 * \param[in] e The current endeffector pose as an Eigen::VectorXd of size 2.
 * \param[in] g The goal pose for the endeffector as an Eigen::VectorXd of size 2.
 * \return A vector (delta_e_x, delta_e_y) as an Eigen::VectorXd of size 2.
 */
EndeffectorPose InverseKinematics::chooseStep(const EndeffectorPose& e, const EndeffectorPose& g) const {
	EndeffectorPose delta_e = Eigen::VectorXd::Zero(e.size());

	/* TODO: Choose the small step of the endeffector towards the goal (delta_e). */

	
	/* The step size alpha is given as a class member variable. */
	delta_e = alpha*(g - e);

	return delta_e;
}

/**
 * \brief Computes how the joint angles have to change in order to reach the step of the endeffector chosen in chooseStep() above.
 * \param[in] delta_e The step of the endeffector choosen by the chooseStep() method as an Eigen::VectorXd.
 * \param[in] jacobian The Jacobian of the endeffector pose with respect to the current joint angles as an Eigen::MatrixXd.
 * \return The change of the joint angles delta_q as an Eigen::VectorXd.
 */
JointAngles InverseKinematics::computeJointChange(const EndeffectorPose& delta_e, const Jacobian& jacobian) const {
	JointAngles delta_q = Eigen::VectorXd::Zero(jacobian.cols());
	/* TODO: Calculate the change of joint angles delta_q that is required to achieve the given
	 * change of the endeffector pose delta_e.
	 */

	/* There are two possibilities to get the inverse of the Jacobian:
	 * - jacobian.inverse(): Exact solution, but sensitive to singularities
	 * - pseudoInverse(jacobian): Returns the Moore-Penrose pseudoinverse, approximate solution, but more robust to singularities
	 */
	delta_q = pseudoInverse(jacobian) * delta_e;

	return delta_q;
}

/**
 * \brief Applies delta_q computed by computeJointChange() to the current joint angles.
 * \param[in] q The current joint angles as an Eigen::VectorXd.
 * \param[in] delta_q The desired change of the joint angles as an Eigen::VectorXd of the same length.
 * \return The new joint angles as an Eigen::VectorXd.
 */
JointAngles InverseKinematics::applyChangeToJoints(const JointAngles& q, const JointAngles& delta_q) const {
	JointAngles q_new = Eigen::VectorXd::Zero(q.size());
	// TODO: Calculate the new joint angles.
	q_new = q + delta_q;

	return q_new;
}

/**
 * \brief Performs one iteration of the inverse kinematics algorithm (the body of the while loop).
 * \param[in,out] q The current joint angles as an Eigen::VectorXd.
 * \param[out]    e The current endeffector pose as an Eigen::VectorXd.
 * \param[in]     g The goal pose of the endeffector as an Eigen::VectorXd.
 */
void InverseKinematics::oneIteration(JointAngles& q, EndeffectorPose& e, const EndeffectorPose& g) const {
	/* TODO: Implement one iteration (= the body of the while loop) here
	 * using the methods defined above to compute the new joint angles q and
	 * endeffector pose e.
	 */
	EndeffectorPose delta_e;
	JointAngles delta_q;
	
	delta_e = chooseStep(e, g);
	delta_q = pseudoInverse(jacobian(q)) * delta_e;

	q = applyChangeToJoints(q, delta_q);
	e = forwardKinematic(q);
}

/**
 * \brief The complete inverse kinematics algorithm.
 * \param[in] g The goal endeffector pose.
 * \param[in] maxTranslationalError The maximum acceptable distance between the endeffector pose and the desired goal pose.
 * \return The joint angles for reaching the given goal pose with the endeffector.
 */
JointAngles InverseKinematics_2Links::computeIK(const EndeffectorPose& g, const double& maxTranslationalError) const {
	JointAngles q = Eigen::VectorXd::Zero(2);
	/* TODO: Initialize q appropriately and call the oneIteration() method
	 * from above in a while loop.
	 */
	q(0) = std::rand();
	q(1) = std::rand();
	EndeffectorPose e = forwardKinematic(q);

	while ((g-e).norm() > maxTranslationalError){
		oneIteration(q, e, g);
	}
	
	return q;
}



/* ============================================================================
 * | The following methods are for part d)-f) with a robot arm with 3 links   |
 * ============================================================================*/

/**
 * \brief Computes the forward kinematics for a robot arm with three links.
 * \param[in] q The current joint angles as an Eigen::VectorXd of size 3.
 * \return The endeffector pose e = (e_x, e_y, e_theta) as an Eigen::VectorXd of size 3.
 */
EndeffectorPose InverseKinematics_3Links::forwardKinematic(const JointAngles& q) const {
	EndeffectorPose e = Eigen::VectorXd::Zero(3);
	/* TODO: Calculate the endeffector pose e = (e_x, e_y, e_theta) */
	Eigen::Vector3d v;
	v << h, 0, 1;

	Eigen::Vector3d e_hom;
	e_hom = rotation(q(0))*translation(a0,0)*rotation(q(1))*translation(a1,0)*rotation(q(2))*translation(a2,0)*v;
	e(0) = e_hom(0);
	e(1) = e_hom(1);
	e(2) = q(0) + q(1) +q(2);

	return e;
}

/**
 * \brief Approximates the Jacobian using the quotient of differences.
 * \param[in] q The current joint angles as an Eigen::VectorXd of size 3.
 * \return The approximated jacobian as an Eigen::MatrixXd of size 3 x 3.
 */
Jacobian InverseKinematics_3Links::jacobian(const JointAngles& q) const {
	Jacobian result = Eigen::MatrixXd::Zero(3, 3);
	const double epsilon = 1e-7;
	JointAngles qEps = Eigen::VectorXd::Zero(3);

	/* TODO: Approximate the Jacobian using the quotient of differences method.
	 * Add the epsilon defined above to the components of q and call forwardKinematic()
	 * to compute the endeffector pose. Then compose the Jacobian from the differences
	 * as described on the exercise sheet.
	 */

	Eigen::Vector3d q_eps;
	Eigen::Vector3d jacobian_column;

	for(int i = 0; i < 3; i++){
		q_eps = q;
		q_eps(i) += epsilon;
		jacobian_column = (forwardKinematic(q_eps)-forwardKinematic(q))/epsilon;
		result(i,0) = jacobian_column(0);
		result(i,1) = jacobian_column(1);
		result(i,2) = jacobian_column(2);
	}

	return result;
}

/**
 * \brief The complete inverse kinematics algorithm.
 * \param[in] g The goal endeffector pose.
 * \param[in] maxTranslationalError The maximum acceptable distance between the endeffector pose and the desired goal pose.
 * \param[in] maxAngularError The maximum acceptable angular error between the endeffector orientation and the desired goal orientation.
 * \return The joint angles for reaching the given goal pose with the endeffector.
 */
JointAngles InverseKinematics_3Links::computeIK(const EndeffectorPose& g, const double& maxTranslationalError, const double& maxAngularError) const {
	JointAngles q = Eigen::VectorXd::Zero(3);

	/* TODO: Initialize q appropriately and call the oneIteration() method
	 * from above in a while loop.
	 */

	q(0) = 1.0;
	q(1) = 1.0;
	q(2) = 1.0;
	EndeffectorPose e = forwardKinematic(q);
	
	Eigen::Vector2d g_trans;
	Eigen::Vector2d e_trans;
	g_trans << g(0), g(1);
	e_trans << e(0), e(1);

	while ( ((g_trans-e_trans).norm() > maxTranslationalError) || (abs(g(2)-e(2)) > maxAngularError )){
		oneIteration(q, e, g);
		e_trans << e(0), e(1);
	}

	return q;
}
}  // namespace inverse_kinematics
