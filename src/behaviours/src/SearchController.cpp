#include "SearchController.h"
#include <angles/angles.h>
#include <array>
#include <cstdlib>
#include <ctime>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

float locationCounter = 1;
float locationCounterTwo = 1;

SearchController::SearchController() 
  {
  rng = new random_numbers::RandomNumberGenerator();
  srand(time(0));
  currentLocation.x = 0; 
  currentLocation.y = 0;
  currentLocation.theta = 0; 
  centerLocation.x = 0;
  centerLocation.y = 0; 
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID; result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;
  centerLocation;
  }

void SearchController::Reset() 
  {
  result.reset = false;
  }

Result SearchController::DoWork()
  {
    //incase something crazy happens and the random number is not a case then default sends back to here: probably not needed
    jump:
    float diffX = this->centerLocation.x - centerLocation.x;
    float diffY = this->centerLocation.y - centerLocation.y;
    this->centerLocation = centerLocation;  
    
  
    centerLocation.x = diffX;
    centerLocation.y = diffY;
    //using a multidimensional array to randomize the pick for which search pattern to use
    int firstPick =  rand()%6;
    int secondPick = rand()%6;
    
    int choice[6][6] = { {1,2,3,4,5,6}, {6,5,4,3,2,1},
                         {6,1,5,2,4,3}, {2,4,6,1,3,5}, 
                         {1,3,5,2,4,6}, {3,2,6,5,4,2} };

    int pattern = choice[firstPick][secondPick];
    
    result.type = waypoint; 
   
    // Create 12 searchLocations, which are each of type Point

    Point  searchLocationOne, searchLocationTwo, searchLocationThree, searchLocationFour, 
           searchLocationFive, searchLocationSix, searchLocationSeven, searchLocationEight,
           searchLocationNine, searchLocationTen, searchLocationEleven, searchLocationTwelve,
	   searchLocationThirteen, searchCluster;
           

    //cloverleaf pattern 1's x,y coordiantes
    int xSearchOne[] =   {0,   1,  4,  5,  4, 0};
    int ySearchOne[] =   {4,   5,  5,  4,  4, 0};
    //cloverleaf pattern 2's x,y coordiantes
    int xSearchTwo[] =   {4,   5,  4,  2,  1,  0};
    int ySearchTwo[] =   {0,  -1,  3, -2, -1,  0};
    //cloverleaf pattern 3's x,y coordiantes
    int xSearchThree[] = {0,  -1, -2, -2, -1,  0};
    int ySearchThree[] = {-4, -5, -4, -2, -1,  0};
    //cloverleaf pattern 4's x,y coordiantes
    int xSearchFour[] =  {-4, -5, -4, -2, -1,  0};
    int ySearchFour[] =  {0,   2,  3,  3,  1,  0};
    //clockwise spiral pattern 1's x,y coordiantes
    int xSearchFive[] =  {0, 2,  0, -2, 0, 4,  0, -4, 0, 6,  0, -6};
    int ySearchFive[] =  {2, 0, -2,  0, 4, 0, -4,  0, 6, 0, -6,  0};
    //counter-clockwise spiral pattern 1's x,y coordiantes
    int xSearchSix[] =   {0, -2,  0, 2, 0, -4,  0, 4, 0, -6,  0, 6};
    int ySearchSix[] =   {2,  0, -2, 0, 4,  0, -4, 0, 6,  0, -6, 0};

    if( ! clusterX.empty() && ! clusterY.empty())
	{
        searchCluster.x = centerLocation.x + clusterX.front();
	searchCluster.y = centerLocation.y + clusterY.front();
        clusterX.pop();
        clusterY.pop();
        result.wpts.waypoints.clear();
        result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchCluster);
	}

    switch(pattern)
      {

        case 1:
    //case 1 is using cloverleaf pattern 1 coordinates

    // Point 1 of search Pattern
            searchLocationOne.x   = centerLocation.x +  xSearchOne[0] + locationCounter; 
            searchLocationOne.y   = centerLocation.y +  ySearchOne[0] + locationCounter; 
    // Point 2 of search pattern
            searchLocationTwo.x   = centerLocation.x +  xSearchOne[1] + locationCounter; 
            searchLocationTwo.y   = centerLocation.y +  ySearchOne[1] + locationCounter; 
    // Point 3 of search pattern
            searchLocationThree.x = centerLocation.x +  xSearchOne[2] + locationCounter; 
            searchLocationThree.y = centerLocation.y +  ySearchOne[2] + locationCounter; 
    // Point 4 of search pattern
            searchLocationFour.x  = centerLocation.x +  xSearchOne[3] + locationCounter; 
            searchLocationFour.y  = centerLocation.y +  ySearchOne[3] + locationCounter; 
    // Point 5 of search pattern
            searchLocationFive.x  = centerLocation.x +  xSearchOne[4] + locationCounter; 
            searchLocationFive.y  = centerLocation.y +  ySearchOne[4] + locationCounter; 
    // Point 6 of search pattern
            searchLocationSix.x   = centerLocation.x +  xSearchOne[5] + locationCounter; 
            searchLocationSix.y   = centerLocation.y +  ySearchOne[5] + locationCounter;

	    searchLocationSeven.x = centerLocation.x;
	    searchLocationSeven.y = centerLocation.y;
    //Return to starting location
            //searchLocationHome.x = homeBase.x;
	    //searchLocationHome.y = homeBase.y;
            break;

        case 2:
    //case 2 is using cloverleaf pattern 2 coordinates

    // Point 1 of search Pattern
            searchLocationOne.x   = centerLocation.x +  xSearchTwo[0] + locationCounter; 
            searchLocationOne.y   = centerLocation.y +  ySearchTwo[0] + locationCounter; 
    // Point 2 of search pattern
            searchLocationTwo.x   = centerLocation.x +  xSearchTwo[1] + locationCounter; 
            searchLocationTwo.y   = centerLocation.y +  ySearchTwo[1] + locationCounter; 
    // Point 3 of search pattern
            searchLocationThree.x = centerLocation.x +  xSearchTwo[2] + locationCounter; 
            searchLocationThree.y = centerLocation.y +  ySearchTwo[2] + locationCounter; 
    // Point 4 of search pattern
            searchLocationFour.x  = centerLocation.x +  xSearchTwo[3] + locationCounter; 
            searchLocationFour.y  = centerLocation.y +  ySearchTwo[3] + locationCounter; 
    // Point 5 of search pattern
            searchLocationFive.x  = centerLocation.x +  xSearchTwo[4] + locationCounter; 
            searchLocationFive.y  = centerLocation.y +  ySearchTwo[4] + locationCounter; 
    // Point 6 of search pattern
            searchLocationSix.x   = centerLocation.x +  xSearchTwo[5] + locationCounter; 
            searchLocationSix.y   = centerLocation.y +  ySearchTwo[5] + locationCounter; 
    //Return to starting location
	    searchLocationSeven.x = centerLocation.x;
	    searchLocationSeven.y = centerLocation.y;
	    break;

        case 3:
    //case 3 is using cloverleaf pattern 3 coordinates

     // Point 1 of search Pattern
            searchLocationOne.x   = centerLocation.x +  xSearchThree[0] + locationCounter; 
            searchLocationOne.y   = centerLocation.y +  ySearchThree[0] + locationCounter; 
    // Point 2 of search pattern
            searchLocationTwo.x   = centerLocation.x +  xSearchThree[1] + locationCounter; 
            searchLocationTwo.y   = centerLocation.y +  ySearchThree[1] + locationCounter; 
    // Point 3 of search pattern
            searchLocationThree.x = centerLocation.x +  xSearchThree[2] + locationCounter; 
            searchLocationThree.y = centerLocation.y +  ySearchThree[2] + locationCounter; 
    // Point 4 of search pattern
            searchLocationFour.x  = centerLocation.x +  xSearchThree[3] + locationCounter; 
            searchLocationFour.y  = centerLocation.y +  ySearchThree[3] + locationCounter; 
    // Point 5 of search pattern
            searchLocationFive.x  = centerLocation.x +  xSearchThree[4] + locationCounter; 
            searchLocationFive.y  = centerLocation.y +  ySearchThree[4] + locationCounter; 
    // Point 6 of search pattern
            searchLocationSix.x   = centerLocation.x +  xSearchThree[5] + locationCounter; 
            searchLocationSix.y   = centerLocation.y +  ySearchThree[5] + locationCounter; 
    //Return to starting location
	    searchLocationSeven.x = centerLocation.x;
	    searchLocationSeven.y = centerLocation.y;
	    break;

        case 4:
    //case 4 is using cloverleaf pattern 4 coordinates

    // Point 1 of search Pattern
            searchLocationOne.x   = centerLocation.x +  xSearchFour[0] + locationCounter; 
            searchLocationOne.y   = centerLocation.y +  ySearchFour[0] + locationCounter; 
    // Point 2 of search pattern
            searchLocationTwo.x   = centerLocation.x +  xSearchFour[1] + locationCounter; 
            searchLocationTwo.y   = centerLocation.y +  ySearchFour[1] + locationCounter; 
    // Point 3 of search pattern
            searchLocationThree.x = centerLocation.x +  xSearchFour[2] + locationCounter; 
            searchLocationThree.y = centerLocation.y +  ySearchFour[2] + locationCounter; 
    // Point 4 of search pattern
            searchLocationFour.x  = centerLocation.x +  xSearchFour[3] + locationCounter; 
            searchLocationFour.y  = centerLocation.y +  ySearchFour[3] + locationCounter; 
    // Point 5 of search pattern
            searchLocationFive.x  = centerLocation.x +  xSearchFour[4] + locationCounter; 
            searchLocationFive.y  = centerLocation.y +  ySearchFour[4] + locationCounter; 
    // Point 6 of search pattern
            searchLocationSix.x   = centerLocation.x +  xSearchFour[5] + locationCounter; 
            searchLocationSix.y   = centerLocation.y +  ySearchFour[5] + locationCounter; 
    //Return to starting location
	    searchLocationSeven.x = centerLocation.x;
	    searchLocationSeven.y = centerLocation.y;
	    break;

	case 5:
    //case 5 is using clockwise spiral pattern coordinates

    // Point 1 of search Pattern
            searchLocationOne.x   = centerLocation.x +  xSearchFive[0]  + locationCounterTwo; 
            searchLocationOne.y   = centerLocation.y +  ySearchFive[0]  + locationCounterTwo; 
    // Point 2 of search pattern
            searchLocationTwo.x   = centerLocation.x +  xSearchFive[1]  + locationCounterTwo; 
            searchLocationTwo.y   = centerLocation.y +  ySearchFive[1]  + locationCounterTwo; 
    // Point 3 of search pattern
            searchLocationThree.x = centerLocation.x +  xSearchFive[2]  + locationCounterTwo; 
            searchLocationThree.y = centerLocation.y +  ySearchFive[2]  + locationCounterTwo; 
    // Point 4 of search pattern
            searchLocationFour.x  = centerLocation.x +  xSearchFive[3]  + locationCounterTwo; 
            searchLocationFour.y  = centerLocation.y +  ySearchFive[3]  + locationCounterTwo; 
    // Point 5 of search pattern
            searchLocationFive.x  = centerLocation.x +  xSearchFive[4]  + locationCounterTwo; 
            searchLocationFive.y  = centerLocation.y +  ySearchFive[4]  + locationCounterTwo; 
    // Point 6 of search pattern
            searchLocationSix.x   = centerLocation.x +  xSearchFive[5]  + locationCounterTwo; 
            searchLocationSix.y   = centerLocation.y +  ySearchFive[5]  + locationCounterTwo; 
    // Point 7 of search Pattern
            searchLocationSeven.x = centerLocation.x +  xSearchFive[6]  + locationCounterTwo; 
            searchLocationSeven.y = centerLocation.y +  ySearchFive[6]  + locationCounterTwo; 
    // Point 8 of search pattern
            searchLocationEight.x = centerLocation.x +  xSearchFive[7]  + locationCounterTwo; 
            searchLocationEight.y = centerLocation.y +  ySearchFive[7]  + locationCounterTwo; 
    // Point 9 of search pattern
            searchLocationNine.x  = centerLocation.x +  xSearchFive[8]  + locationCounterTwo; 
            searchLocationNine.y  = centerLocation.y +  ySearchFive[8]  + locationCounterTwo; 
    // Point 10 of search pattern
            searchLocationTen.x   = centerLocation.x +  xSearchFive[9]  + locationCounterTwo; 
            searchLocationTen.y   = centerLocation.y +  ySearchFive[9]  + locationCounterTwo; 
    // Point 11 of search pattern
            searchLocationEleven.x = centerLocation.x +  xSearchFive[10] + locationCounterTwo; 
            searchLocationEleven.y = centerLocation.y +  ySearchFive[10] + locationCounterTwo; 
    // Point 12 of search pattern
            searchLocationTwelve.x = centerLocation.x +  xSearchFive[11] + locationCounterTwo; 
	    searchLocationTwelve.y = centerLocation.y +  ySearchFive[11] + locationCounterTwo;
    //Return to starting location
	    searchLocationThirteen.x = centerLocation.x;
	    searchLocationThirteen.y = centerLocation.y;
	    break;

	case 6:
    //case 6 is using counterclockwise spiral pattern coordinates

    // Point 1 of search Pattern
            searchLocationOne.x   = centerLocation.x +  xSearchSix[0]  + locationCounterTwo; 
            searchLocationOne.y   = centerLocation.y +  ySearchSix[0]  + locationCounterTwo; 
    // Point 2 of search pattern
            searchLocationTwo.x   = centerLocation.x +  xSearchSix[1]  + locationCounterTwo; 
            searchLocationTwo.y   = centerLocation.y +  ySearchSix[1]  + locationCounterTwo; 
    // Point 3 of search pattern
            searchLocationThree.x = centerLocation.x +  xSearchSix[2]  + locationCounterTwo; 
            searchLocationThree.y = centerLocation.y +  ySearchSix[2]  + locationCounterTwo; 
    // Point 4 of search pattern
            searchLocationFour.x  = centerLocation.x +  xSearchSix[3]  + locationCounterTwo; 
            searchLocationFour.y  = centerLocation.y +  ySearchSix[3]  + locationCounterTwo; 
    // Point 5 of search pattern
            searchLocationFive.x  = centerLocation.x +  xSearchSix[4]  + locationCounterTwo; 
            searchLocationFive.y  = centerLocation.y +  ySearchSix[4]  + locationCounterTwo; 
    // Point 6 of search pattern
            searchLocationSix.x   = centerLocation.x +  xSearchSix[5]  + locationCounterTwo; 
	    searchLocationSix.y   = centerLocation.y +  ySearchSix[5]  + locationCounterTwo;
    // Point 7 of search Pattern
            searchLocationSeven.x = centerLocation.x +  xSearchSix[6]  + locationCounterTwo; 
            searchLocationSeven.y = centerLocation.y +  ySearchSix[6]  + locationCounterTwo; 
    // Point 8 of search pattern
            searchLocationEight.x = centerLocation.x +  xSearchSix[7]  + locationCounterTwo; 
            searchLocationEight.y = centerLocation.y +  ySearchSix[7]  + locationCounterTwo; 
    // Point 9 of search pattern
            searchLocationNine.x  = centerLocation.x +  xSearchSix[8]  + locationCounterTwo; 
            searchLocationNine.y  = centerLocation.y +  ySearchSix[8]  + locationCounterTwo; 
    // Point 10 of search pattern
            searchLocationTen.x   = centerLocation.x +  xSearchSix[9]  + locationCounterTwo; 
            searchLocationTen.y   = centerLocation.y +  ySearchSix[9]  + locationCounterTwo; 
    // Point 11 of search pattern
            searchLocationEleven.x = centerLocation.x +  xSearchSix[10] + locationCounterTwo; 
            searchLocationEleven.y = centerLocation.y +  ySearchSix[10] + locationCounterTwo; 
    // Point 12 of search pattern
            searchLocationTwelve.x = centerLocation.x +  xSearchSix[11] + locationCounterTwo; 
	    searchLocationTwelve.y = centerLocation.y +  ySearchSix[11] + locationCounterTwo;
    //Return to starting location
	    searchLocationThirteen.x = centerLocation.x;
	    searchLocationThirteen.y = centerLocation.y;
	    break;

        default:
            goto jump;
            break;
        } 


    // result.wpts.waypoints.clear() removes anything that is in the vector
    result.wpts.waypoints.clear();
    
    // incrementing locationCounter for each square completion
    locationCounter++;
    locationCounterTwo++;
    if (locationCounter >= 7)
      {
      locationCounter = 1;
      }
    if (locationCounterTwo >= 14)
      {
      locationCounter = 1;
      }
    
    // This adds the corresponding locations to result at the given index.
    // result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation1)
    //    this will insert searchLocation1 at the beginning of the vector result
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationOne);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationTwo);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationThree);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationFour);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationFive);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationSix);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationSeven);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationEight);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationNine);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationTen);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationEleven);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationTwelve);
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocationThirteen);  
    // Return result will send 'result' to the ROSAdapter which will then execute
    //    Whatever its told by result.
    return result;
  }



void SearchController::SetCenterLocation(Point centerLocation) 
  {  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;  
  if (!result.wpts.waypoints.empty()){result.wpts.waypoints.back().x -= diffX; result.wpts.waypoints.back().y -= diffY; }  
  }

void SearchController::SetCurrentLocation(Point currentLocation) 
  {
  this->currentLocation = currentLocation;
  }

void SearchController::ProcessData()
  {
  }
bool SearchController::ShouldInterrupt()
  {
  ProcessData();return false;
  }
bool SearchController::HasWork()
  {
  return true;
  }
void SearchController::SetSuccesfullPickup()
  {
  succesfullPickup = true;
  }








