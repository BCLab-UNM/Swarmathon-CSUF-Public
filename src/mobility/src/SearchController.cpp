#include "SearchController.h"
#include <queue>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation, float spiralStep, std::queue<geometry_msgs::Pose2D> &q) {
  geometry_msgs::Pose2D goalLocation;

  if (q.size() > 0)
	{
		geometry_msgs::Pose2D mem = q.front();
		q.pop();
		goalLocation.x = mem.x;
		goalLocation.y = mem.y;
		goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
		return goalLocation;
	}
  //select new heading from Gaussian distribution around current heading
  goalLocation.theta = currentLocation.theta + .5;

  //select new position 50 cm from current location
  goalLocation.x = currentLocation.x + (.5 * spiralStep * cos(goalLocation.theta));
  goalLocation.y = currentLocation.y + (.5 * spiralStep * sin(goalLocation.theta));
  

  return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, float spiralStep, std::queue<geometry_msgs::Pose2D> &q) {
  geometry_msgs::Pose2D newGoalLocation;
  

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = oldGoalLocation.x;
  newGoalLocation.y = oldGoalLocation.y;

  return newGoalLocation;
}
