/*
FILENAME... NSC200Driver.cpp
USAGE...    Motor driver support for the Newport NewStep NSC200 controller.

Michael Sintschuk
December 21, 2020

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include "NSC200Driver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

#define NSC200_TIMEOUT 2.0
#define LINUX_WRITE_DELAY 0.1

/** Creates a new NSC200Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] serialPortName    The name of the drvAsynSerialPort that was created previously to connect to the NSC200 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
NSC200Controller::NSC200Controller(const char *portName, const char *serialPortName, int controllerID, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, 1, NUM_NSC200_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0),  // Default priority and stack size
   controllerID_(controllerID)
                    
{
  asynStatus status;
  static const char *functionName = "NSC200Controller::NSC200Controller";

  /* Connect to NSC200 controller */
  status = pasynOctetSyncIO->connect(serialPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to NSC200 controller\n",
      functionName);
    return;
  }
  
  // Flush any characters that controller has, read firmware version
  sprintf(outString_, "%dVE", controllerID_);
  status = writeReadController();
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot read version information from NSC200 controller\n",
      functionName);
    return;
  }
  strcpy(controllerVersion_, &inString_[4]);

  // Create the axis object
  new NSC200Axis(this);

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new NSC200Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] serialPortName       The name of the drvAsynIPPPort that was created previously to connect to the NSC200 controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" {

int NSC200CreateController(const char *portName, const char *serialPortName, int controllerID, 
                             int movingPollPeriod, int idlePollPeriod)
{
  new NSC200Controller(portName, serialPortName, controllerID, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

} // extern "C" 


/** Writes a string to the controller.
  * Calls writeNSC200() with a default location of the string to write and a default timeout. */ 
asynStatus NSC200Controller::writeNSC200()
{
  return writeNSC200(outString_, NSC200_TIMEOUT);
}

/** Writes a string to the controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus NSC200Controller::writeNSC200(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  // const char *functionName="writeNSC200";
  
  status = pasynOctetSyncIO->write(pasynUserController_, output,
                                   strlen(output), timeout, &nwrite);
                                   
  // On Linux it seems to be necessary to delay a short time between writes
  #ifdef linux
  epicsThreadSleep(LINUX_WRITE_DELAY);
  #endif
                                  
  return status ;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void NSC200Controller::report(FILE *fp, int level)
{
  fprintf(fp, "NSC200 motor driver %s, controllerID=%d, version=\"%s\n"
              "  moving poll period=%f, idle poll period=%f\n", 
    this->portName, controllerID_, controllerVersion_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an NSC200Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
NSC200Axis* NSC200Controller::getAxis(asynUser *pasynUser)
{
  return static_cast<NSC200Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an NSC200Axis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
NSC200Axis* NSC200Controller::getAxis(int axisNo)
{
  return static_cast<NSC200Axis*>(asynMotorController::getAxis(axisNo));
}


// These are the NSC200Axis methods

/** Creates a new NSC200Axis object.
  * \param[in] pC Pointer to the NSC200Controller to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
NSC200Axis::NSC200Axis(NSC200Controller *pC)
  : asynMotorAxis(pC, 0),
    pC_(pC), 
    currentPosition_(0.)
{
  static const char *functionName = "NSC200Axis::NSC200Axis";

  // Read the stage ID
  sprintf(pC_->outString_, "%dID?", pC->controllerID_);
  pC_->writeReadController();
  strcpy(stageID_, &pC_->inString_[3]);
  
  // fixed parameters of the NSA12 linear actuator
  // Full-step 6.4 μm (48 full-steps/revolution) 
  // Micro-step (1/64 of full-step) 0.10 μm (1/64 of full-step)
  
  fullStepSize_ = 6.4 * 1e-6;
  microStepsPerFullStep_ = 64.;
  
  // Calculate stepSize resolution (mm / microstep)
  stepSize_ = fullStepSize_ / microStepsPerFullStep_ / 1000.;
  
  // Read the low and high software limits
  sprintf(pC_->outString_, "%dSL?", pC->controllerID_);
  pC_->writeReadController();
  lowLimit_ = atof(&pC_->inString_[3]);
  sprintf(pC_->outString_, "%dSR?", pC->controllerID_);
  pC_->writeReadController();
  highLimit_ = atof(&pC_->inString_[3]);

  // activate the motor
  sprintf(pC_->outString_, "%dMO", pC_->controllerID_);
  status = pC_->writeNSC200();  

}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void NSC200Axis::report(FILE *fp, int level)
{
  if (level > 0) {

    fprintf(fp, "  stageID=%s\n"
                "  currentPosition=%f\n"
                "  stepSize=%f, lowLimit=%f, highLimit=%f\n"
                "  fullStepSize=%f, microStepsPerFullStep=%d\n",
            stageID_,
            currentPosition_, 
            stepSize_, lowLimit_, highLimit_,
            fullStepSize_, microStepsPerFullStep_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus NSC200Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "NSC200Axis::move";

  sprintf(pC_->outString_, "%dAC%f", pC_->controllerID_, acceleration*stepSize_);
  status = pC_->writeNSC200();
  sprintf(pC_->outString_, "%dVA%f", pC_->controllerID_, maxVelocity*stepSize_);
  status = pC_->writeNSC200();

  if (relative) {
    sprintf(pC_->outString_, "%dPR%f", pC_->controllerID_, position*stepSize_);
  } else {
    sprintf(pC_->outString_, "%dPA%f", pC_->controllerID_, position*stepSize_);
  }
  status = pC_->writeNSC200();
  return status;
}

asynStatus NSC200Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  //static const char *functionName = "NSC200Axis::home";

  // Must go to unreferenced state to home
  sprintf(pC_->outString_, "%dRS", pC_->controllerID_);
  status = pC_->writeNSC200();
  epicsThreadSleep(1.0);

  // The NSC200 supports home velocity and home acceleration. 
  // We only use home velocity here. Homing is always done to the negative hard limit.
  sprintf(pC_->outString_, "%dOH%f", pC_->controllerID_, maxVelocity);
  status = pC_->writeNSC200();

  sprintf(pC_->outString_, "%dOR", pC_->controllerID_);
  status = pC_->writeNSC200();
  return status;
}

asynStatus NSC200Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  double speed;
  //static const char *functionName = "NSC200Axis::moveVelocity";
  
  //The JOG command accepts seven speed values. We use the fifth.
  if (maxVelocity > 0) speed = 5.;
  else                 speed = -5.;
  
  sprintf(pC_->outString_, "%dJA%f", pC_->controllerID_, speed);
  status = pC_->writeNSC200();
  return status;
}

asynStatus NSC200Axis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "NSC200Axis::stop";

  sprintf(pC_->outString_, "%dST", pC_->controllerID_);
  status = pC_->writeNSC200();
  return status;
}

asynStatus NSC200Axis::setPosition(double position)
{
  //static const char *functionName = "NSC200Axis::setPosition";

  return asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus NSC200Axis::poll(bool *moving)
{ 
  int done=1;
  double position;
  unsigned int status;
  unsigned int state;
  int highLimit=0, lowLimit=0;
  int count;
  asynStatus comStatus;

  // Read the current motor position
  sprintf(pC_->outString_, "%dTP?", pC_->controllerID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "1TPxxx"
  position = atof(&pC_->inString_[3]);
  currentPosition_ = position /stepSize_;
  setDoubleParam(pC_->motorPosition_, currentPosition_);

  // Read the moving status of this motor
  //Returns a status number: 
  //Motor on, motion not in progress 81 
  //Motor on, motion in progress 80 
  //Motor off, motion not in progress 64
  sprintf(pC_->outString_, "%dTS?", pC_->controllerID_);
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  
  // TO-DO onside, also read error code TE
  
  // The response string is of the form "1TSabcdef"
  count = sscanf(pC_->inString_, "%*dTS%*4c%x", &status);
  if (count != 1) goto skip;

  state = status & 0xff;
  if ((state == 0x1e) || (state == 0x28)) done = 0;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // The meaning of the error bits is different for the CC, AGP, and PP
  if ((conexModel_ == ModelConexCC) || (conexModel_ == ModelConexPP)) {
    if (status & 0x100) lowLimit = 1;
    if (status & 0x200) highLimit = 1;
  }
  
  setIntegerParam(pC_->motorStatusLowLimit_, lowLimit);
  setIntegerParam(pC_->motorStatusHighLimit_, highLimit);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg NSC200CreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg NSC200CreateControllerArg1 = {"Serial port name", iocshArgString};
static const iocshArg NSC200CreateControllerArg2 = {"Controller ID", iocshArgInt};
static const iocshArg NSC200CreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg NSC200CreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const NSC200CreateControllerArgs[] = {&NSC200CreateControllerArg0,
                                                                &NSC200CreateControllerArg1,
                                                                &NSC200CreateControllerArg2,
                                                                &NSC200CreateControllerArg3,
                                                                &NSC200CreateControllerArg4};
static const iocshFuncDef NSC200CreateControllerDef = {"NSC200CreateController", 5, NSC200CreateControllerArgs};
static void NSC200CreateContollerCallFunc(const iocshArgBuf *args)
{
  NSC200CreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void NSC200Register(void)
{
  iocshRegister(&NSC200CreateControllerDef, NSC200CreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(NSC200Register);
}
