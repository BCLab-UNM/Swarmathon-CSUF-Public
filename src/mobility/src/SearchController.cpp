#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation, float spiralStep) {
  geometry_msgs::Pose2D goalLocation;

  
  //select new heading from Gaussian distribution around current heading
  goalLocation.theta = currentLocation.theta + .25;

  //select new position 50 cm from current location
  goalLocation.x = currentLocation.x + (0.5 + spiralStep * cos(goalLocation.theta));
  goalLocation.y = currentLocation.y + (0.5 + spiralStep * sin(goalLocation.theta));
  

  return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation, float spiralStep) {
  geometry_msgs::Pose2D newGoalLocation;
  

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.5 + spiralStep * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.5 + spiralStep * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));
  

  return newGoalLocation;
}
