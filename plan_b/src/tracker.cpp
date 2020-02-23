// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <string>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>

#define CTRL_THREAD_PER     0.02        // [s]
#define PRINT_STATUS_PER    1.0         // [s]
#define STORE_POI_PER       2.0         // [s]
#define SWITCH_STATE_PER    10.0        // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

#define STATE_TRACK         0
#define STATE_RECALL        1
#define STATE_WAIT          2
#define STATE_STILL         3

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlThread: public PeriodicThread
{
protected:
    PolyDriver     drvArm, drvGaze;
    IGazeControl  *igaze;
    ICartesianControl *arm;
    

    Vector xd;
    Vector od;
    Vector fp;
    int state;
    int startup_context_id;
    

    double t;
    double t0;
    double t1;
    double t2;
    double t3;
    
    BufferedPort<Bottle> port;

public:
    CtrlThread() : PeriodicThread(1.0) { }

    virtual bool threadInit()
    {
        // open a client interface to connect to the gaze server
        
        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/tracker/gaze");

        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote", "/icubSim/cartesianController/right_arm");
        optArm.put("local","/cartesian_client/right_arm");

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }

        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }


        if (!drvGaze.open(optGaze))
        {
            yError()<<"Unable to open the Gaze Controller";
            return false;
        }

        // open the view
        drvGaze.view(igaze);
        drvArm.view(arm);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the tracking mode, the neck limits and so on.
        igaze->storeContext(&startup_context_id);

        // set trajectory time
        igaze->setNeckTrajTime(0.6);
        igaze->setEyesTrajTime(0.4);
        arm->setTrajTime(3.0);

        // get the torso dofs
        Vector newDof, curDof;
        arm->getDOF(curDof);
        newDof = curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0] = 1;
        newDof[1] = 0;
        newDof[2] = 1;

        // send the request for dofs reconfiguration
        arm->setDOF(newDof, curDof);

        // impose some restriction on the torso pitch
        limitTorsoPitch();

        port.open("/tracker/target:i");

        xd.resize(3);
        od.resize(4);
        fp.resize(3);
        state=STATE_TRACK;

        t=t0=t1=t2=t3=Time::now();

        return true;
    }
//************************************************************//
    virtual void run()
    {
        
        t=Time::now();
        Bottle *pTarget=port.read(false);

        if (state==STATE_TRACK)
        {
            Vector fp(3);        
            fp[0]=-0.4;
            fp[1]=+0.0;
            fp[2]=-0.5;             
            
            igaze->lookAtFixationPoint(fp);

            if (t-t2>=SWITCH_STATE_PER)
            {
                // switch state
                state=STATE_RECALL;
            }
        }

        if (state==STATE_RECALL)
        {
            if (pTarget!=NULL)
            {
                if (pTarget->size()>2)
                {
                    if (pTarget->get(2).asInt()!=0)
                    {
                        Vector px(2);
                        px[0]=pTarget->get(0).asDouble();
                        px[1]=pTarget->get(1).asDouble();

                        // track the moving target within the camera image
                        igaze->lookAtMonoPixel(0,px);   // 0: left image plane is used, 1: for right
                        yInfo()<<"gazing at pixel: "<<px.toString(3,3);

                        if (t-t2>=SWITCH_STATE_PER)
                        {                        
                            // switch state
                            state=STATE_WAIT;
                        }
                    }
                }

            }         
            
        }


        if (state==STATE_WAIT)
        {
            Vector xd(3);        
            xd[0]=-0.3;
            xd[1]=+0.25;
            xd[2]=-0.0;   

            Vector od(4);
            od[0]=0.0; od[1]=0.0; od[2]=0.5; od[3]=-M_PI/2;

            arm->goToPose(xd,od);
            
            //switch state
            state=STATE_STILL;
        }

        if (state==STATE_STILL)
        {
            Vector xd(3);        
            xd[0]=-0.4;
            xd[1]=+0.1;
            xd[2]=-0.0;   

            Vector od(4);
            od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=-M_PI/2;

            arm->goToPose(xd,od);

        }
    }
//*************************************************************************//
    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        arm->getLimits(axis,&min,&max);
        arm->setLimits(axis,min,MAX_TORSO_PITCH);
    }
  
    virtual void threadRelease()
    {
        // we require an immediate stop
        // before closing the client for safety reason
        igaze->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        igaze->restoreContext(startup_context_id);
        
        drvArm.close();
        drvGaze.close();
        port.close();
    }
};


class CtrlModule: public RFModule
{
protected:
    CtrlThread thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        // retrieve command line options
        double period=rf.check("period",Value(0.02)).asDouble();

        // set the thread period in [s]
        thr.setPeriod(period);

        return thr.start();
    }

    virtual bool close()
    {
        thr.stop();
        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        return true;
    }
};



int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    CtrlModule mod;
    return mod.runModule(rf);
}
