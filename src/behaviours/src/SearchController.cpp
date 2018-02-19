#include "SearchController.h"
#include <angles/angles.h>

float locationCounter = 0;
SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
    result.type = waypoint;
    
    // Create 4 searchLocations, which are each of type Point
    Point  searchLocation1, searchLocation2, searchLocation3, searchLocation4;
    
    // locationCounter is used to change the square size
    // if you set locationCounter to some static value the square will be that size
    
    
    // Point 1 of the Square
    searchLocation1.x = centerLocation.x + locationCounter;
    searchLocation1.y = centerLocation.y + locationCounter;
    // Point 2 of the Square
    searchLocation2.x = centerLocation.x - locationCounter;
    searchLocation2.y = centerLocation.y + locationCounter;
    // Point 3 of the Square
    searchLocation3.x = centerLocation.x - locationCounter;
    searchLocation3.y = centerLocation.y - locationCounter;
    // Point 4 of the Square
    searchLocation4.x = centerLocation.x + locationCounter; 
    searchLocation4.y = centerLocation.y - locationCounter;
    
    // result.wpts.waypoints.clear() removes anything that is in the vector
    result.wpts.waypoints.clear();
    
    // incrementing locationCounter for each square completion
    locationCounter++;
    if (locationCounter >= 6){
        locationCounter = 0;
    }
    
    // This adds the corresponding locations to result at the given index.
    // result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation1)
    //    this will insert searchLocation1 at the beginning of the vector result
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation1);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation2);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation3);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation4);
    
    // Return result will send 'result' to the ROSAdapter which will then execute
    //    Whatever its told by result.
    return result;
}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}


