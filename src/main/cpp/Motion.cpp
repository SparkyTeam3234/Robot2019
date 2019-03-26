
#include "Motion.h"
#include <cmath>
#include <frc/Timer.h>
#include <wpi/raw_os_ostream.h>

Motion::Motion()
{
    m_x4=0;
    m_y4=0;
    m_dt=0;
    m_fp=0;
    m_sp=0;
    m_done=false;
    m_k=0;
}

Motion::Motion(float k)
{
    m_x4=0;
    m_y4=0;
    m_dt=0;
    m_fp=0;
    m_sp=0;
    m_done=true;
    m_k=k;
}

float Motion::P345(float t)
{
    float t2=t*t;
    float t3=t2*t;
    return (10*t3)-(15*t2*t2)+(6*t3*t2);
}
/*
float Motion::BCurvature(float x1,float x2, float x3, float x4, float y1, float y2, float y3, float y4, float t)
{
    float Tx;
    float Ty;
    float rprmx;
    float rprmy;
    float magrprm;
    float Tprmx;
    float Tprmy;
    float t2=t*t;
    float t3=t2*t;
   
    rprmx= x1*(-3*t2+6*t-3)+3*x2*(3*t2-4*t+1)+3*x3*(2*t-3*t2)+3*x4*t2;
    rprmy= y1*(-3*t2+6*t-3)+3*y2*(3*t2-4*t+1)+3*y3*(2*t-3*t2)+3*y4*t2;
    
    magrprm=sqrt(pow(rprmx,2)+pow(rprmy,2));

    Tx=rprmx/magrprm;
    Ty=rprmy/magrprm;
    
    
}
*/
float Motion::FB(float tp, float cp)
{
    /*
        tp== target position
        cp== current position
        k==  scalar   
    */
   float v=0;
    
    if (cp>tp)
       v=-m_k*sqrt(cp-tp);
    if (tp>cp)
       v=m_k*sqrt(tp-cp);

    if (v>1.0)
        v=1.0;
    if (v<-1.0)
        v=-1.0;

    return v;
}

float Motion::Drive(float cp, float fp, float dt, float tol, float k)
{
    //wpi::errs() << " Drive Function Called! ";
    if(m_fp !=fp) //Target Position Changed
    {
        //wpi::errs() << "\nDrive Initialized!! ";
        m_fp=fp;
        m_sp=cp;
        m_dt=dt;
        m_t.Reset();
        m_t.Start();
        m_done=false;
        m_k=k;
    }
    if(!m_done)
    {
        float t=m_t.Get();
        float tp=(m_fp-m_sp)*P345(t/m_dt)+m_sp;
        //wpi::errs() << "\nP345: " <<P345(t/m_dt);
        //wpi::errs() << " Target Position: " <<tp;
        //wpi::errs() << " Current Position: " <<cp;
        if(t>m_dt) m_done=true;
        if(abs(cp-m_fp)<tol) m_done=true;
        //wpi::errs() << " Output: " << FB(tp,cp);
        return FB(tp,cp);
    }
    else
    {
        return 0;
    }
    
}
float Motion::Curve(float cp, float gangl, float dt, float tol, float k,float x1,float x2, float x3, float x4, float y1, float y2, float y3, float y4, bool isleft)
{
    float mag;
    float vx;
    float thead;
    float cgangl=gangl-m_sgangl;
    float sclr=0.2;
    float v;
    float fp;   
    

    
    if(m_x4!=x4 || m_y4!=y4)
    {
        
        m_sgangl=gangl;
        //wpi::errs() << "Initialized!! ";
        m_x4=x4;
        m_y4=y4;
        m_sp=cp;
        fp= sqrt(pow(x4-x1,2)+pow(y4-y1,2))+m_sp;
        
        
        m_fp=fp;
        m_dt=dt;
        m_t.Reset();
        m_t.Start();
        m_done=false;
        m_k=k;
    }
    wpi::errs() << "fp: " << fp;
    if(!m_done)
    {
        float lv=0;
        float t=m_t.Get()/dt;
        float t2=t*t;
        float t3=t*t2;


        float dot=y1*(-3*t2+6*t-3)+3*y2*(3*t2-4*t+1)+3*y3*(2*t-3*t2)+3*y4*t2;
        
        vx= x1*(-3*t2+6*t-3)+3*x2*(3*t2-4*t+1)+3*x3*(2*t-3*t2)+3*x4*t2;
    
        mag=sqrt(pow(vx,2)+pow(dot,2));
        
        thead=acosf(dot/mag);
        if(acosf(vx/mag)>90*4*atan(1)/180)
        thead=-thead;
        lv = Drive(cp,m_fp,dt,tol,k);
        //wpi::errs() << "Linear V " << lv;
        if (isleft)
        {
            v=lv+sclr*sin(thead);
        }
        else 
        {
            v=lv-sclr*sin(thead);
        }
        //wpi::errs() << "Uncorrected V " << v;
        if (v>1.0)
            v=1.0;
        if (v<-1.0)
            v=-1.0;

        return v;
    }
    
    else 
    {
        //wpi::errs() << "Curve Function Done";
        return 0;
    }
}
float Motion::ConvertInchTick(float wd, float tkperrev)
{
    //wd= wheel Diameter
    //tkperrev= ticks per revolution
    return wd*tkperrev*3.14159265;
}
void Motion::ResetTime()
{
    m_t.Reset();
}

float Motion::GetTime()
{
    return m_t.Get();
}
float Motion::GetK()
{
    return m_k;
}

bool Motion::GetDone()
{
    return m_done;
}