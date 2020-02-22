// Tutorial on Inverse Kinematics.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <vector>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/cv/Cv.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/pids.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::cv;
using namespace iCub::ctrl;

/************************************************************/
cv::Point repoint(cv::Mat &imgMat, const Vector &p)
{
    return cv::Point((int)(imgMat.size().width/2.0+p[0]),
                     (int)(imgMat.size().height/2.0-p[1]));
}

/*1.0************************************************************/
void drawAxes(cv::Mat &imgMat, const Matrix &H,
              const cv::Scalar &axis_x=cv::Scalar(255,0,0),
              const cv::Scalar &axis_y=cv::Scalar(0,255,0))
{
    Vector x=20.0*H.getCol(0);
    Vector y=20.0*H.getCol(1);
    Vector p=H.getCol(2);
    cv::line(imgMat,repoint(imgMat,p),repoint(imgMat,p+x),axis_x);
    cv::line(imgMat,repoint(imgMat,p),repoint(imgMat,p+y),axis_y);
}
/************************************************************/
class Link
{
    double length;
    vector<Vector> points;
/************************************************************/
public:
    Link(const double length_) :
       length(length_), points(6,Vector(3,1.0))
    {
        double height=6;

        points[0][0]=0.0;
        points[0][1]=0.0;

        points[1][0]=0.0;
        points[1][1]=height/2.0;

        points[2][0]=length;
        points[2][1]=height/2.0;

        points[3][0]=length;
        points[3][1]=0.0;

        points[4][0]=length;
        points[4][1]=-height/2.0;

        points[5][0]=0.0;
        points[5][1]=-height/2.0;
    }
/************************************************************/
    Matrix draw(const Matrix &H, const double joint, cv::Mat &imgMat) const
    {
        double c=cos(joint);
        double s=sin(joint);

        Matrix H_=eye(3,3);
        H_(0,0)=c; H_(0,1)=-s;
        H_(1,0)=s; H_(1,1)=c;
        H_=H*H_;

        vector<cv::Point> pts;
        for (auto &point:points)
        {
            Vector p=H_*point;
            pts.push_back(repoint(imgMat,p));
        }
        vector<vector<cv::Point>> poly(1,pts);

        cv::fillPoly(imgMat,poly,cv::Scalar(108,27,108));
        cv::circle(imgMat,pts[0],4,cv::Scalar(0,255,0),cv::FILLED);
        cv::circle(imgMat,pts[3],4,cv::Scalar(0,255,0),cv::FILLED);

        Matrix T=eye(3,3); T(0,2)=length;
        return H_*T;
    }
};

/************************************************************/
class Robot
{
    vector<Link> links;
    Integrator *joints;
/************************************************************/
public:
    Robot(const int n, const double length, const double Ts)
    {
        for (int i=0; i<n; i++)
            links.push_back(Link(length));
        joints=new Integrator(Ts,Vector(n,0.0));
    }
/************************************************************/
    void move(const Vector &velocity, ImageOf<PixelRgb> &img)
    {
        Matrix H=eye(3,3);
        joints->integrate(velocity);

        cv::Mat imgMat=toCvMat(img);
        for (int i=0; i<links.size(); i++)
            H=links[i].draw(H,joints->get()[i],imgMat);
	drawAxes(imgMat,H); //2.0
    }
/************************************************************/
    Vector getJoints() const
    {
        return joints->get();
    }
/************************************************************/
    virtual ~Robot()
    {
        delete joints;
    }
};

/************************************************************/
class RobotModule : public RFModule
{
    Robot *robot;

    BufferedPort<ImageOf<PixelRgb>> portEnvironment;
    BufferedPort<Bottle> portMotors;
    BufferedPort<Bottle> portEncoders;
    BufferedPort<Bottle> portTarget;

    Vector velocity;
    Vector target;
    int env_edge;
/******4.0****************************************************/
public:
    bool configure(ResourceFinder &rf)override
    {
        int dof=rf.check("dof",Value(3)).asInt();
        double link_length=rf.check("link-length",Value(60.0)).asDouble();
        env_edge=rf.check("environment-edge",Value(500)).asInt();

        robot=new Robot(dof,link_length,getPeriod());

        portEnvironment.open("/tutorial_inverse-kinematics-robot/environment:o");
        portMotors.open("/tutorial_inverse-kinematics-robot/motors:i");
        portEncoders.open("/tutorial_inverse-kinematics-robot/encoders:o");
        portTarget.open("/tutorial_inverse-kinematics-robot/target:i");

        velocity.resize(dof,0.0);
        target.resize(3,0.0);
        return true;
    }
/************************************************************/
    bool close()override
    {
        delete robot;

        portEnvironment.close();
        portMotors.close();
        portEncoders.close();
        portTarget.close();

        return true;
    }


/************************************************************/
    double getPeriod()override
    {
        return 0.01;
    }
/************************************************************/
    bool updateModule()override
    {
        if (Bottle *vel=portMotors.read(false))
            if (vel->size()>=(int)velocity.length())
                for (int i=0; i<vel->size(); i++)
                    velocity[i]=vel->get(i).asDouble();

        if (Bottle *tar=portTarget.read(false))
        {
            if (tar->size()>=2)
            {
                target[0]=tar->get(0).asDouble();
                target[1]=tar->get(1).asDouble();
            }
        }

        ImageOf<PixelRgb> &env=portEnvironment.prepare();
        env.resize(env_edge,env_edge); env.zero();

        cv::Mat imgMat=toCvMat(env);
        cv::circle(imgMat,repoint(imgMat,target),5,cv::Scalar(0,0,255),cv::FILLED);

        Vector rot(4,0.0);   //3.0
        rot[2]=1.0; rot[3]=target[2];
        Matrix H=axis2dcm(rot).submatrix(0,2,0,2);
        H(0,2)=target[0]; H(1,2)=target[1]; H(2,2)=1.0;
        drawAxes(imgMat,H,cv::Scalar(0,0,255));

	robot->move(velocity,env);

        Vector joints=robot->getJoints();
        Bottle &encoders=portEncoders.prepare();
        encoders.clear();

        for (int i=0; i<(int)joints.length(); i++)
            encoders.addDouble(joints[i]);

        portEnvironment.writeStrict();
        portEncoders.writeStrict();

        return true;
    }
};

/************************************************************/
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

    RobotModule robot;
    return robot.runModule(rf);
}
