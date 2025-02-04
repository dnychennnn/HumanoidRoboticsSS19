#include <windows-helpers.h>
#include <iostream>
#include <footstep_planning/FileIO.h>
#include <footstep_planning/FootstepPlanning.h>

using namespace footstep_planning;

int main(int /* argc */, char ** /*argv*/)
{
	const std::string packagePath = PROJECT_SOURCE_DIR;

	FileIO fileIO(packagePath);
	FootstepPlanning planning(fileIO.map);
	const FootstepNode * const start = FootstepNode::get(1.0, 0.2, 0.0, LEFT);
	const FootstepNode * const goal = FootstepNode::get(2.0, 2.0, -0.5 * M_PI, RIGHT);
	std::cout << "Planning path (this will take a while) ..." << std::endl;
	const std::deque<const AbstractNode *> path = planning.planPath(start, goal);

	fileIO.logPath(packagePath + "/data/path.txt", path);

	std::cout << std::endl << "Final path: ";
	if (path.empty()) {
		std::cout << "empty" << std::endl;
	} else {
		std::cout << std::endl;
		for (std::deque<const AbstractNode *>::const_iterator it = path.begin(); it != path.end(); ++it) {
			if (!*it) {
				throw std::runtime_error("The path contains a NULL pointer.");
			}
			std::cout << (*it)->toString() << std::endl;
		}
	}

	std::cout << "Done. " << std::endl;
    wait();
	return 0;
}
#include <footstep_planning/FootstepPlanning.h>

namespace footstep_planning {

/**
 * @brief Returns the costs for stepping from one footstep to the next footstep.
 * @param[in] currentFootstep The current foot step.
 * @param[in] successorFootstep The next foot step where the robot intends to step to.
 * @return The cost value for this transition.
 */
double FootstepPlanning::getCosts(const FootstepNode* const currentFootstep, const FootstepNode* const successorFootstep) const {
	// The robot cannot step with the same foot twice in a row. In this case, the costs are infinite.
	if (currentFootstep->foot == successorFootstep->foot) {
		return std::numeric_limits<double>::infinity();
	}
	double result = 0.0;
	//TODO: Fill "result" with the costs for moving from the current foot step to the successor foot step

	/* Available methods and fields:
	 * footstep->x: x position of the footstep
	 * footstep->y: y position of the footstep
	 * footstep->theta: orientation of the foot in radians
	 */
	double dx = currentFootstep->x - successorFootstep->x;
	double dy = currentFootstep->y - successorFootstep->y;
	
	return std::sqrt(dx*dx + dy*dy);
}

/**
 * @brief Calculates the Euclidean distance heuristic between the current foot step and the goal.
 * @param[in] currentFootstep The current foot step.
 * @param[in] goalFootstep The goal foot step.
 * @return The Euclidean distance between the current foot step and the goal.
 */
double FootstepHeuristic::heuristic(const FootstepNode* const currentFootstep, const FootstepNode* const goalFootstep) const {
	double result = 0.0;
	/* TODO: Implement the Euclidean distance footstep heuristic.
	 * The heuristic should only consider the translational distance, the angular difference should not be considered.
	 */

	double dx = currentFootstep->x - goalFootstep->x;
	double dy = currentFootstep->y - goalFootstep->y;
	result = std::sqrt(dx*dx + dy*dy);

	return result;
}

/**
 * @brief Calculates where the robot's foot ends up if it executes a given footstep action.
 * @param[in] currentFootstep The current foot step
 * @param[in] action The footstep action.
 */
FootstepNode* FootstepPlanning::executeFootstep(const FootstepNode * const currentFootstep, const FootstepAction& action) {
	double x = 0.0;
	double y = 0.0;
	double theta = 0.0;

	/* TODO: Calculate the position (x, y, theta) where the robot will end up when it is currently in
	 * currentFootstep and executes the given footstep action. The footstep action gives the position
	 * of the other foot in the coordinate frame of the current foot step.
	 *
	 * Example foot step actions for currentFootstep->foot == LEFT, action.foot == RIGHT:
	 * dx = 0,   dy = -0.1, dtheta =  0.0 --> put the right foot parallel to the left foot ("stand position").
	 * dx = 0.1, dy = -0.1, dtheta =  0.0 --> step forward with the right foot
	 * dx = 0,   dy = -0.2, dtheta =  0.0 --> step to the right
	 * dx = 0,   dy = -0.1, dtheta =  0.1 --> turn the right foot counterclockwise(= inwards)
	 * dx = 0,   dy = -0.1, dtheta = -0.1 --> turn the right foot clockwise (= outwards)
	 */

	/* Available fields and methods:
	 * currentFootstep->x: Cartesian x position of the foot step in world coordinates
	 * currentFootstep->y: Cartesian y position of the foot step in world coordinates
	 * currentFootstep->theta: Orientation of the foot step in world coordinates
	 * action.dx: Forward distance relative to the current foot
	 * action.dy: Sideways distance relative to the current foot (positive = left, negative = right)
	 * action.dtheta: Orientation relative to the current foot (positive = counterclockwise)
	 *
	 * FootstepNode::get(x, y, theta, action.foot): Create a new foot step with the given world coordinates
	 */

	x = currentFootstep->x + cos(currentFootstep->theta)*action.dx + sin(currentFootstep->theta)*action.dy;
	y = currentFootstep->y + cos(currentFootstep->theta) * action.dy - sin(currentFootstep->theta) * action.dx;
	theta = currentFootstep->theta + action.dtheta;

	return FootstepNode::get(x, y, theta, action.foot);
}

/**
 * @brief Returns a vector of foot steps that the robot can execute given the current foot step.
 * @param[in] currentFootstep The current foot step.
  * @return A vector of foot steps that the robot can execute with its other foot.
 */
std::vector<AbstractNode*> FootstepPlanning::getNeighborNodes(const FootstepNode* const currentFootstep) {
	std::vector<AbstractNode*> neighborNodes;
	// Choose the right set of footsteps: If the robot has previously stepped with the left foot, then
	// step with the right foot next (and vice-versa).
	const std::vector<FootstepAction> footstepActions(currentFootstep->foot == LEFT ? rightFootActions : leftFootActions);

	/* TODO: Fill neighborNodes with footsteps reachable from the current foot step.
	 * Make sure not to add footsteps that collide with obstacles.
	 */

	/* Available variables and methods:
	 * footstepActions: Vector of footstep actions, defined above
	 * executeFootstep(FootstepNode *footstep, FootstepAction action): Returns the successor footstep
	 *     for a given footstep and action, defined above
	 * isColliding(FootstepNode *footstep): Returns true if the footstep collides with the environment.
	 */
	FootstepNode* newStep;
	for (size_t i = 0; i < footstepActions.size(); i++) {
		newStep = executeFootstep(currentFootstep, footstepActions[i]);
		if (!isColliding(newStep)) {
			neighborNodes.push_back(newStep);
		}
	}


	return neighborNodes;
}

/**
 * @brief Tests whether the current footstep is close enough to the goal for ending the search.
 * @param[in] currentNode The current footstep node.
 * @param[in] goalNode The goal footstep pose.
 * @returns true if the current footstep is close enough.
 */
bool FootstepPlanning::isCloseToGoal(const FootstepNode * const currentNode, const FootstepNode * const goalNode) {
	const double dx = currentNode->x - goalNode->x;
	const double dy = currentNode->y - goalNode->y;
	return (sqrt(dx * dx + dy * dy) < 0.10 && fabs(angles::shortest_angular_distance(currentNode->theta, goalNode->theta)) < 0.25 * M_PI);
}

}  // namespace footstep_planning
