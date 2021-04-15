 //  created:    2011/05/01
//  filename:   PosApp.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo cercle avec optitrack
//
//
/*********************************************************************/

#include "PosApp.h"

#include "attQuatBkstp/attQuatBkstp.h"
#include "attQuatBkstp/attQuatBkstp_impl.h"

#include "xyBackstepping/xyBackstepping.h"
#include "xyBackstepping/xyBackstepping_impl.h"

#include "zBackstepping/zBackstepping.h"
#include "zBackstepping/zBackstepping_impl.h"

#include "xyPD/xyPD.h"
#include "xyPD/xyPD_impl.h"

#include "attQuatPD/attQuatPD.h"
#include "attQuatPD/attQuatPD_impl.h"

#include "attSMC/attSMC.h"
#include "attSMC/attSMC_impl.h"

#include "xySMC/xySMC.h"
#include "xySMC/xySMC_impl.h"

#include <TargetController.h>
#include <Imu.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <TrajectoryGenerator2DCircle.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <DoubleSpinBox.h>
#include <iostream>
#include <Tab.h>
#include <TabWidget.h> 
#include <Quaternion.h>
#include <GroupBox.h>
#include <CheckBox.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

// Constructor
PosApp::PosApp(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80);

    //In this the uav object is created
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);
	
    // this could be useful to add modes or actions to the control law. 
    // this is to be used in the Qt gui. 	
    // PushButton on the GUI ONLY for attitude control
    attitudePB = new PushButton(GetButtonsLayout()->NewRow(),"Attitude Ctrl");

    // PushButton on the GUI ONLY for altitude control
    zCtrlPB = new PushButton(GetButtonsLayout()->LastRowLastCol(),"z Ctrl");
    // PushButton on the GUI for Position and attitude control
    positionPB = new PushButton(GetButtonsLayout()->LastRowLastCol(),"Position Ctrl");	

    // wtih this we create an horizontal tab at the top of the window
    Tab *myTab = new Tab(getFrameworkManager()->GetTabWidget(),"myTab");



    // with this we create three vertical tabs 
    TabWidget *tab_widget = new TabWidget(myTab->NewRow(),"User Control");
    setup_myTab = new Tab(tab_widget,"Setup");
    graph_law_angles = new Tab(tab_widget,"Graphs 01 pf");
    graph_law_position = new Tab(tab_widget,"Graphs 02 pf");
    

    z_Ctrl = new zBackstepping(setup_myTab->At(0,2),"zCtrl");
    z_Ctrl->UseDefaultPlot(graph_law_angles->NewRow());

    xCtrl = new xyPD(setup_myTab->At(0,0),"xCtrl-PD");
    xCtrl->UseDefaultPlot(graph_law_angles->LastRowLastCol());

    yCtrl = new xyPD(setup_myTab->At(0,1),"yCtrl-PD");
    yCtrl->UseDefaultPlot(graph_law_angles->LastRowLastCol());

    attQuat = new attSMC(setup_myTab->At(1,1),"Attitude Quaternion - SMC");
    attQuat->UseDefaultPlot(graph_law_angles->LastRowLastCol());

    attPD = new attQuatPD(setup_myTab->At(1,2),"Attitude PD_PFFP");
    attPD->UseDefaultPlot(graph_law_angles->LastRowLastCol());

    xSMC = new xySMC(setup_myTab->At(2,0),"x Ctrl SMC");
    xSMC->UseDefaultPlot(graph_law_position->At(0,0));

    ySMC = new xySMC(setup_myTab->At(2,1),"y Ctrl SMC");
    ySMC->UseDefaultPlot(graph_law_position->At(0,1));




    GroupBox *PosRef = new GroupBox(setup_myTab->At(1,0),"Position References");
    x_des = new DoubleSpinBox(PosRef->NewRow(),"x_des:",-10,10,1,1);
    y_des = new DoubleSpinBox(PosRef->NewRow(),"y_des:",-5,5,1,1);
    z_des = new DoubleSpinBox(PosRef->NewRow(),"z_des:",-5,5,1,1,1);
    psi_des = new DoubleSpinBox(PosRef->NewRow(),"psi_des",0,360,1,0);
    amplitude = new DoubleSpinBox(PosRef->NewRow(),"ampl:",-5,5,1,2,2);
    fs = new DoubleSpinBox(PosRef->NewRow(),"fs:",-5,5,1,1,3);

    getFrameworkManager()->AddDeviceToLog(attPD);
    getFrameworkManager()->AddDeviceToLog(xCtrl);
    getFrameworkManager()->AddDeviceToLog(yCtrl);
    getFrameworkManager()->AddDeviceToLog(z_Ctrl);

    customReferenceOrientation= new AhrsData(this,"reference");

    //customReferenceOrientation= new AhrsData(this,"HolaTest");
    uav->GetAhrs()->AddPlot(customReferenceOrientation,DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);

    customOrientation=new AhrsData(this,"orientation");
    //AddDataToLog(z_Ctrl);
    
      // Con esto inicializamos el vector. El tamno del vector es DELAY en muestras (d=h/Ts   h es tu delay en sec)!!!
    // DELAY lo defines en Backstepping.h 

}

PosApp::~PosApp() {
}

/****************************************************************
        ATTTIUDE EXTRACTION FROM DRONE
*****************************************************************
    -> Extract the attitude from the drone the measurements are obtained from the IMU  and optitrack
    -> The output is given in quaternion for attitude
    -> The output is given in 3D vector for angular speed.
*****************************************************************/
const AhrsData *PosApp::GetOrientation(void) const {
    //get yaw from vrpn
    // Generates a quaternion object named vrpnQuaternion
	Quaternion vrpnQuaternion;
    //GetQuaternion works with refernces, hence the vrpnQuaternion is filled with the 
    //uav attitude in quaternions
    uavVrpn->GetQuaternion(vrpnQuaternion);

    // get roll, pitch and w from imu
    // Generates a quaternion object named ahrsQuaternion
    Quaternion ahrsQuaternion;
    // Generates a float Vector3D named ahrsAngularSpeed
    Vector3Df ahrsAngularSpeed;
    //GetQuaternionAngularRates works with references,
    //GetDefaultOrientation works with references
    //ahrsQuaternion is filled 
    //ahrsAngular Speed is filled
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    //an object ahrsEuler  of type Euler (3D vector with roll, pitch and yaw components) is created
    //The object arhrsEuler is filled with the Euler Angles associated to the Quadrotor Quaternion attitude
    Euler ahrsEuler=ahrsQuaternion.ToEuler();
    //Once the ahrsEuler is filled with the attitude quaternion (data obtained from IMU)
    // the yaw angle is changed by the measure from the vrpn() (Optitrack object)
    // this is done to avoid the derive of the angle, (Error usually presented on the IMU)
    ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
    // A Quaternion object named mixQuaternion is obtained using the Euler angles
    // roll->IMU            pitch->IMU          yaw->Optitrack
    Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

    //custom orientation is filled with the mixQuaternion and the AngularSpeed 
    // Observe that Angular Speed is from the IMU (roll, pitch and yaw)
    // Why didn't they use the angular speed also from the Optitrack????
    customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

    return customOrientation;
}


/******************************************************
        ALTITUDE VALUES   (FROM OPTITRACK)
*******************************************************
    ->Extract the altitude from the optitrack
    ->Changes into the Drone's frame
******************************************************/
void PosApp::AltitudeValues(float &z,float &dz) const{
    // a float 3DVector is created to store the uav_pos
    // a float 3DVector is created to store the uav_vel
    Vector3Df uav_pos,uav_vel;

    //GetPosition and GetSpeed work with references
    // uav_pos is filled with the uav position from Optitrack
    // uav_vel is filled with the uav velocity from Optitrack
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    //z and dz must be in uav's frame
    z=-uav_pos.z;
    dz=-uav_vel.z;
}




/*******************************************************************
            STATE MACHINE (Event Driven Programming)
********************************************************************
This a state machine or Event Driven Programming we've seen this in the
coursera course.
There are only three states
    TakingOff
        -> Changes the BehaviourMode_t to DEFAULT
        -> Disable the Optitrack

    EnteringControlLoop
        -> Verifies if the behaviourMode is in Cricle and if the circle is not running
            then commands hover flight

        -> If the first condition doesn't fits, hence it is in Circle Mode and the
           uav is executing the circle, so it keeps doing that action

    EnteringFailSafeMode
        -> Sets the BehaviourMode_t to default

Here we must play with:
behaviourMode 
BehaviourMode_t::Constants
********************************************************************/
void PosApp::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    //case Event_t::EnteringControlLoop:
    //    if ((behaviourMode==BehaviourMode_t::Circle) && (!circle->IsRunning())) {
    //        VrpnPositionHold();
    //    }
    //    break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}


/***************************************************************************************
    ExtraCheckPushButton  -> Checks whick button of the QtWidget has been clicked and changes the
                             behaviourMode into the three states of the state-machine of the SignalEvent
                             method
****************************************************************************************
// This Method, helps to verify the buttons of the GroundStation (I Guessssss)
// Signals and slots are used here if the QT implementation is done
// No input
// No outpu
****************************************************************************************/
void PosApp::ExtraCheckPushButton(void) {
    if(attitudePB->Clicked() && (behaviourMode==BehaviourMode_t::Default)){
        OnlyAttitudeJoystick_mode();
    }
    if(zCtrlPB->Clicked() && (behaviourMode==BehaviourMode_t::Default)){
        AltitudeMode();
    }
    if(positionPB->Clicked() && (behaviourMode==BehaviourMode_t::Default)){
        xyControl();
    }

}

/***************************************************************************************
    ExtraCheckJoystick  - > Checks which button of the Dualshock has ben pressed and changes the
                            behaviourMode into the the three states of the state-machine the SignalEvent
                            method
****************************************************************************************
// this function helps to verify which buttons from the Joystick has been pressed
// No input
// No output
                    Do not use cross, start nor select buttons!!
    0: "start"       1: "select"      2: "square"      3: "triangle"
    4: "circle"      5: "cross";      6: "left 1"      7: "left 2"
    8: "left 3"      9: "right 1"     10: "right 2"    11: "right 3"
    12: "up"         13: "down"       14: "left"       15: "right"
****************************************************************************************/
void PosApp::ExtraCheckJoystick(void) {
    if(GetTargetController()->IsButtonPressed(10) && (behaviourMode==BehaviourMode_t::Default)){
        OnlyAttitudeJoystick_mode();
    }
    if(GetTargetController()->IsButtonPressed(7) && (behaviourMode==BehaviourMode_t::Default)){
        AltitudeMode();
    }
    if(GetTargetController()->IsButtonPressed(6) && (behaviourMode==BehaviourMode_t::Default)){
        xyControl();
        flag=false;
    }
    
    if(GetTargetController()->IsButtonPressed(4) && (behaviourMode==BehaviourMode_t::Default)){ // Circulo
        flag=true;
        time2 = double(GetTime())/1000000000;
        xyControl();
    }

}


void PosApp::OnlyAttitudeJoystick_mode(void){
    std::cout << "We might be in Attitude Control and the reference might be given by the Dualshock3" << std::endl; 
    behaviourMode=BehaviourMode_t::AttitudeCtrl;

    auto _TorqueMode = GetTorqueMode();
    auto _OrientationMode = GetOrientationMode();

    if(_TorqueMode == TorqueMode_t::Custom){
        cout << " TORQUE CUSTOM"  << endl;
    }
    else{
        if(SetTorqueMode(TorqueMode_t::Custom)){
            cout << "SE HA CAMBIADO A TOQUE CUSTOM" << endl;
        }
        else{
            cout << "No se ha podidio cambiar de modo el torque" << endl;
        }
    }

    if(_OrientationMode == OrientationMode_t::Manual){
        cout << "OrientationMode MANUAL"  << endl;
    }
    else{
        if(SetOrientationMode(OrientationMode_t::Manual)){
            cout << "OrientationMode MANUAL" << endl;
        }
        else{
            cout << "No se ha podidio cambiar de modo de Orientacion" << endl;
        }
    }

    
}

void PosApp::AltitudeMode(void){
    std::cout << "We might be in Atltitude mode... Check the performance and good luck   " << endl;
    behaviourMode=BehaviourMode_t::AlitutudeMode;

    auto _AltitudeMode = GetAltitudeMode();
    auto _TorqueMode = GetTorqueMode();
    auto _OrientationMode = GetOrientationMode();

    if(_AltitudeMode==AltitudeMode_t::Custom){
        cout << "Altitude Mode CUSTOM" << endl;
    }
    else{
        if(SetAltitudeMode(AltitudeMode_t::Custom)){
            cout << "Altitude Mode CUSTOM" << endl;
        }
        else{
            cout << "NO se ha podido cambiar a Altitude CUSTOM" << endl; 
        }
    }

    if(_TorqueMode == TorqueMode_t::Default){
        cout << "TORQUE DEFAULT"  << endl;
    }
    else{
        if(SetTorqueMode(TorqueMode_t::Default)){
            cout << "SE HA CAMBIADO A TOQUE Default" << endl;
        }
        else{
            cout << "No se ha podidio cambiar de modo el torque" << endl;
        }
    }

    if(_OrientationMode == OrientationMode_t::Manual){
        cout << "OrientationMode Manual"  << endl;
    }
    else{
        if(SetOrientationMode(OrientationMode_t::Manual)){
            cout << "OrientationMode Manual" << endl;
        }
        else{
            cout << "No se ha podidio cambiar de modo de Orientacion" << endl;
        }
    }


}



void PosApp::xyControl(void){
    std::cout << "We might be in XY Position Control mode .... Check the performance and good luck" << endl; 
    behaviourMode=BehaviourMode_t::PositionCtrl;

    auto _TorqueMode = GetTorqueMode();
    auto _OrientationMode = GetOrientationMode();
    auto _AltitudeMode = GetAltitudeMode();

    if(_TorqueMode == TorqueMode_t::Custom){
        cout << " TORQUE CUSTOM"  << endl;
    }
    else{
        if(SetTorqueMode(TorqueMode_t::Custom)){
            cout << "SE HA CAMBIADO A TOQUE CUSTOM" << endl;
        }
        else{
            cout << "No se ha podidio cambiar de modo el torque" << endl;
        }
    }

    if(_OrientationMode == OrientationMode_t::Custom){
        cout << "OrientationMode CUSTOM"  << endl;
    }
    else{
        if(SetOrientationMode(OrientationMode_t::Custom)){
            cout << "OrientationMode CUSTOM" << endl;
        }
        else{
            cout << "No se ha podidio cambiar de modo de Orientacion" << endl;
        }
    }

    if(_AltitudeMode==AltitudeMode_t::Custom){
        cout << "Altitude Mode CUSTOM" << endl;
    }
    else{
        if(SetAltitudeMode(AltitudeMode_t::Custom)){
            cout << "Altitude Mode CUSTOM" << endl;
        }
        else{
            cout << "NO se ha podido cambiar a Altitude CUSTOM" << endl; 
        }
    }
}

void PosApp::MixOrientation(void) {
    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);
	
	if (first_up2==true){
		//////////////////////VRPN////////////////////////////
		Quaternion vrpnQuaternion;
		uavVrpn->GetQuaternion(vrpnQuaternion);
		Euler vrpnEuler;
		vrpnQuaternion.ToEuler(vrpnEuler);
		vrpnQuaternion.q0 = cos(vrpnEuler.yaw/2);
		vrpnQuaternion.q1 = 0;
		vrpnQuaternion.q2 = 0;
		vrpnQuaternion.q3 = sin(vrpnEuler.yaw/2);
		vrpnQuaternion.Normalize();
		//----------------------------------------------------
		
        // Rotate attitude to VRPN inertial frame
		qI = vrpnQuaternion*ahrsQuaternion.GetConjugate();
        first_up2 = false;
    }

	mixQuaternion = qI*ahrsQuaternion;
	mixQuaternion.Normalize();
	mixAngSpeed = ahrsAngularSpeed;
}

void PosApp::MixOrientation(flair::core::Quaternion &mixAttitude,flair::core::Vector3Df &mixSpeed) {
    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);
    
    if (first_up2==true){
        //////////////////////VRPN////////////////////////////
        Quaternion vrpnQuaternion;
        uavVrpn->GetQuaternion(vrpnQuaternion);
        Euler vrpnEuler;
        vrpnQuaternion.ToEuler(vrpnEuler);
        vrpnQuaternion.q0 = cos(vrpnEuler.yaw/2);
        vrpnQuaternion.q1 = 0;
        vrpnQuaternion.q2 = 0;
        vrpnQuaternion.q3 = sin(vrpnEuler.yaw/2);
        vrpnQuaternion.Normalize();
        //----------------------------------------------------
        
        // Rotate attitude to VRPN inertial frame
        qI = vrpnQuaternion*ahrsQuaternion.GetConjugate();
        first_up2 = false;
    }

    mixAttitude = qI*ahrsQuaternion;
    mixAttitude.Normalize();
    mixSpeed = ahrsAngularSpeed;
}


void PosApp::MixOrientation(flair::core::Quaternion &mixAttitude){
    //get roll, pitch and w from imu
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);
    
    if (first_up2==true){
        //////////////////////VRPN////////////////////////////
        Quaternion vrpnQuaternion;
        uavVrpn->GetQuaternion(vrpnQuaternion);
        Euler vrpnEuler;
        vrpnQuaternion.ToEuler(vrpnEuler);
        vrpnQuaternion.q0 = cos(vrpnEuler.yaw/2);
        vrpnQuaternion.q1 = 0;
        vrpnQuaternion.q2 = 0;
        vrpnQuaternion.q3 = sin(vrpnEuler.yaw/2);
        vrpnQuaternion.Normalize();
        //----------------------------------------------------
        
        // Rotate attitude to VRPN inertial frame
        qI = vrpnQuaternion*ahrsQuaternion.GetConjugate();
        first_up2 = false;
    }

    mixAttitude = qI*ahrsQuaternion;
    mixAttitude.Normalize();
}


/*********************************************************************************************************************
                        ComputeCustomTorques
**********************************************************************************************************************
This method has to be overloaded in order to compute the attitude control
The method woks with references, it receives a reference of  torque, hence, the the main objective of this method
is to fill the torque Euler object with the control for roll, pitch and yaw.
torques.roll = ctrl_roll
torques.pitch = ctrl_pitch
torques.yaw = ctrl_yaw
**********************************************************************************************************************/
void PosApp::ComputeCustomTorques(Euler &torques){
  //cout << "ComputingCustomTorques" << endl; 

  // REFERENCES in Quaternion
  Quaternion refQuaternion;
  Vector3Df refAngularRates;

  // Computes the reference quaternion, (from Joystick or from position control)
  GetReferenceQuaternion(refQuaternion, refAngularRates);

  
  // Computes the quadrotor attitude
  Quaternion currentQuaternion;
  Vector3Df currentAngularRates;

  // Computes the quadrotor attitude
  // Manual -> IMU
  // Custom -> IMU with Optitrak
  GetUAVOrientation(currentQuaternion, currentAngularRates);
    Quaternion q;
    Vector3Df w; 

	MixOrientation(q,w);
	//q =	mixQuaternion;
	//Vector3Df w = mixAngSpeed;
  
  // Obtain the Quadrotor Attitude
  //const AhrsData *currentOrientation = GetOrientation();
  
  // STATES IN QUATERNION
  //Quaternion currentQuaternion;
  //Vector3Df currentAngularRates;

  //AddDataToControlLawLog(currentQuaternion);
  // FILLING THE QUATERNION AND THE ANGULAR RATE
  //currentOrientation->GetQuaternionAndAngularRates(currentQuaternion,currentAngularRates);

      attQuat->SetValues(q.q0,q.q1,q.q2,q.q3,
                     refQuaternion.q0, refQuaternion.q1, refQuaternion.q2, refQuaternion.q3,
                     w.x, w.y, w.z); 

//   attQuat->SetValues(currentQuaternion.q0,currentQuaternion.q1,currentQuaternion.q2,currentQuaternion.q3,
//                      refQuaternion.q0, refQuaternion.q1, refQuaternion.q2, refQuaternion.q3,
//                      currentAngularRates.x, currentAngularRates.y, currentAngularRates.z); 
  
  attQuat->Update(GetTime());


/*
    attPD->SetValues(q.q0,q.q1,q.q2,q.q3,
                     refQuaternion.q0, refQuaternion.q1, refQuaternion.q2, refQuaternion.q3,
                     w.x, w.y, w.z); 
    attPD->Update(GetTime());

  torques.roll  = attPD->Output(0);
  torques.pitch = attPD->Output(1);
  torques.yaw   = attPD->Output(2);
*/


  torques.roll  = attQuat->Output(0);
  torques.pitch = attQuat->Output(1);
  torques.yaw   = attQuat->Output(2);


}

// This is to obtain the reference quaternion does not matter if is from Joystick or position control
void PosApp::GetReferenceQuaternion(flair::core::Quaternion &refQuat, flair::core::Vector3Df &refOmega){
    if(GetOrientationMode() == OrientationMode_t::Manual){
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    refOrientation->GetQuaternionAndAngularRates(refQuat, refOmega);
    //cout << "JoyMode" << endl;
    }
  else{
    const AhrsData *refOrientation = GetReferenceOrientation();
    refOrientation->GetQuaternionAndAngularRates(refQuat, refOmega);
    //cout << "PosCtrl   " << endl; 
    } 
} 

void PosApp::GetUAVOrientation(flair::core::Quaternion &uavQuaternion, flair::core::Vector3Df &uavAngSpeed){
    if(GetOrientationMode() == OrientationMode_t::Manual){
        GetDefaultOrientation()->GetQuaternionAndAngularRates(uavQuaternion, uavAngSpeed);
    }
  else{
      const AhrsData *currentOrientation = GetOrientation();
      currentOrientation->GetQuaternionAndAngularRates(uavQuaternion,uavAngSpeed);
    } 
}

/******************************************************************************************************
                    ComputeCustomThrust
********************************************************************************************************
This is the implmentation of the altitude control law
INPUT -> None
OUTPUT -> float - the thurst computed

*****************************************************************************************************/
float PosApp::ComputeCustomThrust(void){
    /*// two scalars are declared to store the position and velocity of the drone along the z-axis.
    float pos_z;
    float vel_z;

    // AltitudeValues works with references, hence, the variagles pos_z and vel_z are filled within this
    // functio
    AltitudeValues(pos_z, vel_z);

    // A random reference for altitude, this can be changed to use the dualshock measure, we don't know
    // how but it can be done, we should check the UavStateMachine classs, everythint is in there.
    float reference_altitude = 1.30;
    //cout << "pos_z: " << pos_z << "    vel_z: " << vel_z << endl; 
    float error = pos_z  - reference_altitude;

    float dot_error = vel_z;
    z_Ctrl->SetValues(pos_z, reference_altitude,vel_z,0.0);
    z_Ctrl->Update(GetTime());


    float commanded_thrust = -z_Ctrl->Output();

    cout << "commanded_thrust: " << commanded_thrust << endl;
    
    */

    //cout << "ComputeCustomThrust ----   "; 

    Vector3Df PositionCtrl;
    ComputePositionControllers(PositionCtrl);

    float Thrust = PositionCtrl.z; 
    //cout << "Trust: " << Thrust << endl;

    return Thrust;
}


/*******************************************************************************
        EXTRACT THE ATITTUDE REFERENCE (desired quaternion  q_d)

        HERE IS THE POSITION CONTROL
********************************************************************************
    -> Extract the references for position
    -> Computes the tracking error (e)
    -> Computes the velocity trackin errror (dot_e)
    -> Sends the e and dot_e to the pid clas for x and y
    -> Once the control law is computed, it is taken as reference to the attitude
    -> Here we can change the attitude selection, from the Dualshock or from the posision control.
            if(position_mode){
                refAngles.roll = uX->Output();
                refAngles.pitch = uY->Output();
                refAngles.yaw = 0. // this can be changed
            else{
                refAngles.roll = measure_from_Dualshock.x
                refAngles.pitch = measure_from_Dualshock.y
                refAngles.yaw = measure_from_Dualshock.z
                }

    -> NO INPUT
    -> OUTPUT AhrsData  
    
    THIS METHOD HAS TO BE OVERLOADED IN ORDER TO IMPLMENT YOUR CONTROL LAW
*******************************************************************************/
AhrsData *PosApp::GetReferenceOrientation(void) {

    Quaternion Qd;
    ComputeDesiredQuaternion(Qd);


    //Euler refAngles;
    //refAngles.roll = PosController.y;
    //refAngles.pitch = PosController.x;
    //refAngles.yaw = 0.0;
    // In this the attitude reference is computed
    // ONLY IF YOU ARE USING THE CONTROL POSITION
    customReferenceOrientation->SetQuaternionAndAngularRates(Qd,Vector3Df(0,0,0));

    //customReferenceOrientation is of type AhrsData // why??? wich kind of dat is it???

    // Returns the Attitude reference in quaternion and Omeg_vec = [ 0 0 0 ] ^T
    return customReferenceOrientation;
}

void PosApp::ComputePositionControllers(flair::core::Vector3Df &Controller){
    
    Vector3Df uav_pos,uav_vel;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    // two scalars are declared to store the position and velocity of the drone along the z-axis.
    float pos_z;
    float vel_z;

    // AltitudeValues works with references, hence, the variagles pos_z and vel_z are filled within this
    // function
    AltitudeValues(pos_z, vel_z);

    float dot_error = vel_z;
    
    z_Ctrl->SetValues(pos_z, z_des->Value(),vel_z,0.0);
    z_Ctrl->Update(GetTime());

    float ux;
    float uy;
    float uz;


    if(flag){
        Vector3Df uavPosTraj, uavVelTraj;
        circulo(uavPosTraj,uavVelTraj);
        xSMC->SetValues(uav_pos.x,uavPosTraj.x,uavVelTraj.x,0.0);
        xSMC->Update(GetTime());

        ySMC->SetValues(uav_pos.y,uavPosTraj.y,uavVelTraj.y,0.0);
        ySMC->Update(GetTime());

        ux = xSMC->Output();
        uy = ySMC->Output();
        uz = -z_Ctrl->Output();


    }
    else{
        xSMC->SetValues(uav_pos.x,x_des->Value(),uav_vel.x,0.0);
        xSMC->Update(GetTime());

        ySMC->SetValues(uav_pos.y,y_des->Value(),uav_vel.y,0.0);
        ySMC->Update(GetTime());

        ux = xSMC->Output();
        uy = ySMC->Output();
        uz = -z_Ctrl->Output();
    }




    Controller.x = ux;
    Controller.y = uy;
    Controller.z = uz;
}

void PosApp::circulo(){
 
    
    double time1 = double(GetTime())/1000000000;            
    float timeT = time1 - time2;
    
    x_circ = amplitude->Value()*cos(fs->Value()*timeT);
    y_circ = amplitude->Value()*sin(fs->Value()*timeT);

    
    dx_circ = -amplitude->Value()*sin(fs->Value()*timeT);
    dy_circ = amplitude->Value()*cos(fs->Value()*timeT);
    
 
}

void PosApp::circulo(flair::core::Vector3Df &posRef, flair::core::Vector3Df &velRef ){
    double time1 = double(GetTime())/1000000000;            
    float timeT = time1 - time2;
    
    posRef.x = amplitude->Value()*cos(fs->Value()*timeT);
    posRef.y = amplitude->Value()*sin(fs->Value()*timeT);
    posRef.z = z_des->Value();
    
    velRef.x = -amplitude->Value()*sin(fs->Value()*timeT);
    velRef.y = amplitude->Value()*cos(fs->Value()*timeT);
    velRef.z = 0.0;
}


void PosApp::ComputeDesiredQuaternion(flair::core::Quaternion &DesiredQuaternion){
    Vector3Df UPos;
    ComputePositionControllers(UPos);

    UPos.Normalize();
    Euler ahrs_des;
    Quaternion q;
	MixOrientation(q);
	
    
    //circulo();

    
    //ahrs_des.yaw = (180*3.1416)/180;
    //ahrs_des.yaw =
    float yaw_rad;

    if(flag){
        Vector3Df uavPosTraj, uavVelTraj;
        circulo(uavPosTraj,uavVelTraj);
        yaw_rad = atan2(uavPosTraj.x,uavPosTraj.y);

    }
    else{
        yaw_rad= ToRadians(psi_des->Value());
    }
    
    Quaternion refQuaternionQz = Quaternion(cos(yaw_rad/2),0,0,sin(yaw_rad/2));
    refQuaternionQz.Normalize();
        

    Quaternion qze = refQuaternionQz.GetConjugate()*q;
	Vector3Df thetaze = 2*qze.GetLogarithm();
	float zsign = 1;
	if (thetaze.GetNorm()>=3.1416){
		printf("Reference is too far!! %f \n",thetaze.GetNorm());
		zsign = -1;
	}
	refQuaternionQz = zsign*refQuaternionQz;

    // This vector can be changed... since the z-axis is pointing in a different direction
    // This vector is thrust in the body frame
    Vector3Df v(0.0,0.0,1.0);

    float qd_0 = sqrt((1 + DotProduct(UPos,v))/2);

    Vector3Df q_vec = VectorialProduct(UPos,v);

    DesiredQuaternion.q0 = qd_0;
    DesiredQuaternion.q1 = qd_0*q_vec.x;
    DesiredQuaternion.q2 = qd_0*q_vec.y;
    DesiredQuaternion.q3 = qd_0*q_vec.z;
    
/*
    if(flag==true){
    DesiredQuaternion = DesiredQuaternion*refQuaternionQz;
    
    }else{
         DesiredQuaternion = DesiredQuaternion;
    }
*/


    DesiredQuaternion = DesiredQuaternion*refQuaternionQz;


    DesiredQuaternion.Normalize();
    
}


float PosApp::DotProduct(flair::core::Vector3Df v1, flair::core::Vector3Df v2){
    return (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
}


flair::core::Vector3Df PosApp::VectorialProduct(flair::core::Vector3Df m, flair::core::Vector3Df v){
   return(Vector3Df(m.y*v.z - v.y*m.z,
                    v.x*m.z - m.x*v.z,
                    m.x*v.y - v.x*m.y));
}

float PosApp::ToRadians(float input){
    return input*3.1416/180;
}

