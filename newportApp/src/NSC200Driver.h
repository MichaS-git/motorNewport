/*
FILENAME...  NSC200Driver.h
USAGE...     Motor driver support for the Newport NewStep NSC200 controller.

Michael Sintschuk
December 21, 2020

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

// Using the NSC-JB RS485 Junction Box
#define MAX_NSC200_AXES 32

// No controller-specific parameters yet
#define NUM_NSC200_PARAMS 0  

class epicsShareClass NSC200Axis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  NSC200Axis(class NSC200Controller *pC);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);

private:
  NSC200Controller *pC_;        /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  double currentPosition_;
  double stepSize_;
  double fullStepSize_;
  int    microStepsPerFullStep_;
  double highLimit_;
  double lowLimit_;
  char   stageID_[40];
  
friend class NSC200Controller;
};

class epicsShareClass NSC200Controller : public asynMotorController {
public:
  NSC200Controller(const char *portName, const char *serialPortName, int controllerID, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  NSC200Axis* getAxis(asynUser *pasynUser);
  NSC200Axis* getAxis(int axisNo);
  asynStatus writeNSC200();
  asynStatus writeNSC200(const char *output, double timeout);

private:
  int controllerID_;
  char controllerVersion_[40];

  friend class NSC200Axis;
};
