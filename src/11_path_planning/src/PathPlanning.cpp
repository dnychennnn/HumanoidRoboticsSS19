#include <path_planning/PathPlanning.h>
#include <cmath>

namespace path_planning {

/**
 * @brief Calculates the costs for traveling from currentNode to successorNode.
 * @param[in] currentNode The current grid cell.
 * @param[in] successorNode The next grid cell where the robot will travel to.
 * @return The costs (i.e., the Euclidean distance) for traveling from currentNode to successorNode.
 */
double GridPathPlanning::getCosts(const GridNode * const currentNode, const GridNode * const successorNode) const {
	/* TODO: calculate the costs for traveling from currentNode to successorNode
	 * (= Euclidean distance between the two grid cells) */

	/* Available fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 */
	double dx = currentNode->x - successorNode->x;
	double dy = currentNode->y - successorNode->y;

	return std::sqrt(dx*dx + dy*dy);
}


/**
 * @brief Calculates the straight line heuristic between the current node and the goal.
 * @param[in] currentNode The current grid cell.
 * @param[in] goalNode The goal grid cell.
 * @return The straight line distance between the current cell and the goal.
 */
double StraightLineDistanceHeuristic::heuristic(const GridNode* const currentNode, const GridNode* const goalNode) const {
	double result = 0.0;

	/* TODO: calculate the straight line distance heuristic between the
	 * current node and the goal node. 	 */

	/* Available fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 */
	double dx = currentNode->x - goalNode->x;
	double dy = currentNode->y - goalNode->y;

	return std::sqrt(dx*dx + dy*dy);
}

/**
 * @brief Calculates the Manhattan heuristic between the current node and the goal.
 * @param[in] currentNode The current grid cell.
 * @param[in] goalNode The goal grid cell.
 * @return The Manhattan distance between the current cell and the goal.
 */
double ManhattanDistanceHeuristic::heuristic(const GridNode* const currentNode, const GridNode* const goalNode) const {
	double result = 0.0;

	/* TODO: calculate the Manhattan distance heuristic between the
	 * current node and the goal node.  */

	/* Available fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 */
	double dx = std::abs(currentNode->x - goalNode->x);
	double dy = std::abs(currentNode->y - goalNode->y);

	return dx + dy;
}

/**
 * @brief Returns a vector of neighbor nodes of the current node where the robot can travel to.
 * @param[in] currentNode The current grid cell.
 * @param[in] map The grid map of the environment.
 * @return A vector of neighbor grid cells that are accessible to the robot.
 */
std::vector<AbstractNode *> GridPathPlanning::getNeighborNodes(const GridNode * const currentNode, const GridMap& map) {
	std::vector<AbstractNode *> result;

	/* TODO: Fill the vector "result" with the eight neighbor nodes of currentNode.
	 * Only add nodes that are within the map bounds and that are not occupied by
	 * an obstacle in the map.  */

	/* Available methods and fields:
	 * - node->x: the x index of the cell
	 * - node->y: the y index of the cell
	 * - map.width: width of the map in cells
	 * - map.height: height of the map in cells
	 * - map.isOccupied(int x, int y): returns true iff the cell is occupied by an obstacle
	 * - GridNode::get(int x, int y): creates and returns a new node representing a cell.
	 */
	int next_x; // the index of next neighbor that is going to be add to the queue
	int next_y;

	for (int i = -1; i <= 1 ; i++){
		for (int j = -1; j <= 1; j++){
			if (i==0 && j == 0)
				continue; // exclude currentNode
			next_x = currentNode->x + j;
			next_y = currentNode->y + i;
			if (next_x < map.width && next_y < map.height && !map.isOccupied(next_x, next_y))
				result.push_back(GridNode::get(next_x, next_y));
		}
	}

	return result;
}


/**
 * @brief Expands the current node and adds its neighbor cells to the list of open cells.
 * @param[in] currentNode The current node.
 * @param[in] goalNode The goal node where the robot should travel to.
 * @param[in,out] openList The list of open nodes.
 * @param[in] closedList The list of nodes that are already visited and closed by the algorithm.
 */

/**
 * @brief Expands the current node and adds its neighbor cells to the list of open cells.
 * @param[in] currentNode The current node.
 * @param[in] goalNode The goal node where the robot should travel to.
 * @param[in,out] openList The list of open nodes.
 * @param[in] closedList The list of nodes that are already visited and closed by the algorithm.
 */
void PathPlanning::expandNode(const AbstractNode * const currentNode, const AbstractNode * const goalNode,
		OpenList& openList, const ClosedList& closedList) {

	/* TODO: Expand the currentNode and add the neighbor cells to the openList. */

	/* Available methods and fields:
	 * - getNeighborNodes(AbstractNode* node): defined above, returns a vector of neighbor nodes.
	 * - getCosts(AbstractNode* from, AbstractNode* to): defined above, returns the costs for traveling
	 *     from one node to another node.
	 * - heuristic(AbstractNode* node, AbstractNode* goalNode): defined above, returns the value of
	 *     one of the heuristics
	 *
	 * - node->costs: field for storing cost values, read/write access
	 * - node->setPredecessor(AbstractNode* node): store the predecessor node for a node (required
	 *     later for extracting the path)
	 *
	 * - closedList.contains(AbstractNode* node): returns true if the node is already on list of closed nodes.
	 *
	 * - openList.enqueue(AbstractNode *node, double costs): put a node into the open list queue, the
	 *     node with the lowest costs will be processed next.
	 * - openList.updateCosts(AbstractNode *node, double costs): update the costs of a node that is
	 *     already in the open list queue.
	 * - openList.contains(AbstractNode* node): returns true if the node is already on the list of open nodes.
	 */
	std::vector<AbstractNode*> neighbors = getNeighborNodes(currentNode);
	path_planning::AbstractNode * neighbor;
	double costFromStartToGoal;
	double costFromCurrentToNeighbor;
	double costFromNeighborToGoal;

	for(int i = 0; i<neighbors.size(); i++) {
		neighbor = neighbors[i];

		if (!closedList.contains(neighbor)){
			costFromCurrentToNeighbor = getCosts(currentNode, neighbor);
			costFromNeighborToGoal = heuristic(neighbor, goalNode);
			costFromStartToGoal = currentNode->costs + costFromCurrentToNeighbor + costFromNeighborToGoal;
			if (openList.contains(neighbor)){
				if (costFromStartToGoal < neighbor->costs + costFromNeighborToGoal){
					openList.updateCosts(neighbor, costFromStartToGoal);
					neighbor->setPredecessor(currentNode);
					neighbor->costs = currentNode->costs + costFromCurrentToNeighbor;
				}else
					continue;
			}
			else{
				openList.enqueue(neighbor, costFromStartToGoal);
				neighbor->costs = currentNode->costs + costFromCurrentToNeighbor;
				neighbor->setPredecessor(currentNode);
			}
		}
	}


}


/**
 * @brief Returns true if the current node is close to the goal node.
 * @param[in] currentNode The current node.
 * @param[in] goalNode The goal node.
 * @return True iff the current grid cell equals the goal grid cell.
 */
bool GridPathPlanning::isCloseToGoal(const GridNode * const currentNode, const GridNode * const goalNode) {
	// We already implemented this method for you, nothing to do here.
	return ((currentNode->x == goalNode->x) && (currentNode->y == goalNode->y));
}


/**
 * @brief Plans a path from a start node to a goal node.
 * @param[in] startNode The start node.
 * @param[in] goalNode The goal node.
 * @return The optimal path from the start node to the end node.
 */
std::deque<const AbstractNode*> PathPlanning::planPath(const AbstractNode * const startNode, const AbstractNode * const goalNode) {
	// Create empty lists for open nodes and closed nodes
   	OpenList openList;
   	ClosedList closedList;

   	std::deque<const AbstractNode*> resultPath;

   	/* TODO: Fill resultPath by planning a path from the startNode to the goalNode */

   	/* Available methods:
   	 * - isCloseToGoal(AbstractNode *node, AbstractNode *goalNode): defined above, use this method to check
   	 *     whether the robot has reached the goal.
	 * - openList.enqueue(AbstractNode* node, double costs): put a node into the open list queue, the
	 *     node with the lowest costs will be processed next.
	 * - openList.removeMin(): returns the node with the lowest costs from the open list queue
	 *     and removes the node from the queue.
	 * - closedList.add(AbstractNode* node): put a node into the list of closed nodes
     * - closedList.contains(AbstractNode* node): returns true if the node is already on list of closed nodes.
     * - expandNode(...): defined above, adds the valid neighbors of the current node to the open list.
     * - followPath(AbstractNode* node): defined below, extracts the path from the start node
     *     to the current node by following the chain of predecessors.
   	 */
	const AbstractNode* currentNode;
	
	openList.enqueue(startNode, heuristic(startNode, goalNode));

	do{
		currentNode = openList.removeMin();
		expandNode(currentNode, goalNode, openList, closedList);

		if(!closedList.contains(currentNode)){
			closedList.add(currentNode);
		}

	} while(isCloseToGoal(currentNode,goalNode) != true);

	resultPath = followPath(currentNode);

	return resultPath;
}


/**
 * @brief Extracts the path from the currentNode back to the start node.
 * @param[in] node The current node.
 * @return The path from the start node up to the current node.
 */
std::deque<const AbstractNode*> PathPlanning::followPath(const AbstractNode * const node) {
	std::deque<const AbstractNode *> path;
	/* TODO: Fill the path by following the predecessors from the current node back
	 * to the start node. */

	/* Available methods:
	 * - node->getPredecessor(): returns the predecessor saved with setPredecessor()
	 * - path.push_front(AbstractNode* node): Inserts the node at the beginning of the path
	 * - path.push_back(AbstractNode* node): Inserts the node at the end of the path
	 */
	const AbstractNode* currentNode = node;
	path.push_back(node);
	while(currentNode->getPredecessor()!=NULL){
		path.push_front(currentNode->getPredecessor());
		currentNode = currentNode->getPredecessor();
	}

	return path;
}

}  // namespace path_planning
