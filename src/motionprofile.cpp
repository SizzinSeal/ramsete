#include "include/motionprofile.h"

float GetPathLength(std::vector<lemlib::Pose>* p) {
  //calculate path length and each points distance
  float running = 0;
  for (int i = 0; i < p->size()-1; i++) {
    p->at(i).pathDistance = running;
    running+=abs(p->at(i).distance(p->at(i + 1)));
  }
  p->at(p->size() - 1).pathDistance = running; //add the last points runningDis
  return running;
}

//------generate profile---------
void MotionProfile::DetermineTopSpeed() {
  d = GetPathLength(path);

  float attainable = sqrt(2 * accel * d / (1 + (accel / deaccel)));
  topSpeed = std::min(attainable, velConstraint);
}


void MotionProfile::DetermineDistances() {
  //t1 = t3
  //d1 = d3
  //tTotal = t1+t2+t3

  d1 = ((topSpeed*topSpeed) / (2 * accel));
  
  d3 = ((topSpeed * topSpeed) / (2 * deaccel));

  d2 = d - (d1 + d3);
}

void MotionProfile::CalculateVelocities() {
  path->at(0).linearVel = 0;
  path->at(0).angularVel = 0;
  path->at(0).theta = startingTheta;
  float pathDistance = 0;
  float prevPathDistance = 0;
  
  std::vector<lemlib::Pose> copy = *path;

  for (int i = 1; i < path->size(); i++) {
    lemlib::Pose* current = &path->at(i);

    float distBetween = path->at(i - 1).distance(path->at(i));
    float pVel = path->at(i - 1).linearVel;

    pathDistance = path->at(i).pathDistance;

    //Check which phase the point is within
    //Since phase 2 does not involve acceleration, we seperate it from phase 1 and 3
    
    //If Phase 2:
    if (pathDistance <= (d1+d2) && pathDistance > d1) {
      current->linearVel = topSpeed;
    }
    
    //If Phase 1 or 3
    else {
      float _accel = (pathDistance <= d1) ? (accel) : (-deaccel);  //condition: if Phase 1
      float v = sqrt(std::max((pVel * pVel) + 2 * _accel * distBetween, (float)0));
      current->linearVel = v;
    }

    //Insert point at boundary between phase 1 and phase 2
    if (prevPathDistance < d1 && d1 < pathDistance) {
      float t = (d1-prevPathDistance) / (pathDistance - prevPathDistance);
      lemlib::Pose x = path->at(i - 1).lerp(*current, t);
      x.linearVel = topSpeed;
      x.pathDistance = d1;
      auto it = path->begin();
      path->insert(it+i, x);
    }

    prevPathDistance = pathDistance;

    //Calculate Angular Velocity
    
    //Guard Clause: If point is last in line, angularVel = 0
    if (i == path->size() - 1) {current->angularVel = 0; continue;}

    float theta = current->angle(path->at(i + 1));
    current->theta = theta;

    float deltaTheta = theta - path->at(i-1).theta;

    float deltaD = current->pathDistance - path->at(i-1).pathDistance;
    float deltaT = 2*deltaD / (current->linearVel + path->at(i-1).linearVel);

    float angularVel = deltaTheta/deltaT;
    
    current->angularVel = angularVel;

  }

}




MotionProfile::MotionProfile(std::vector<lemlib::Pose>* path, float velConstraint, float accel, float deaccel, float startingTheta) {
  this->path = path;
  this->velConstraint = velConstraint;
  this->accel = accel;
  this->deaccel = deaccel;

  DetermineTopSpeed();
  DetermineDistances();
  CalculateVelocities();
}



