// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   attQuatPD_impl.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a PID
//
//
/*********************************************************************/
#include "attQuatPD_impl.h"
#include "attQuatPD.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <iostream>
#include <math.h>
#include <Quaternion.h>
#include "Eigen/Dense"
#include <Euler.h>



using namespace std;

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

using flair::core::Quaternion;

using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::VectorXf;
using Eigen::Vector3f;
using Eigen::ArrayXf;
using Eigen::Quaternion;
using Eigen::Quaternionf;

attQuatPD_impl::attQuatPD_impl(attQuatPD *self, const LayoutPosition *position, string name) {
  i = 0;
  first_update = true;
  this->self = self;

  // init matrix
  self->input = new Matrix(self, 11, 1, floatType, name);

  MatrixDescriptor *desc = new MatrixDescriptor(10, 1);
  desc->SetElementName(0, 0, "qe0");
  desc->SetElementName(1, 0, "qe1");
  desc->SetElementName(2, 0, "qe2");
  desc->SetElementName(3, 0, "qe3");
  desc->SetElementName(4, 0, "Quad_Euler.roll");
  desc->SetElementName(5, 0, "Quad_Euler.pitch");
  desc->SetElementName(6, 0, "Quad_Euler.yaw");
  desc->SetElementName(7, 0, "Quad_Euler_d.roll");
  desc->SetElementName(8, 0, "Quad_Euler_d.pitch");
  desc->SetElementName(9, 0, "Quad_Euler_d.yaw"); 

  state = new Matrix(self, desc, floatType, name);
  delete desc;

  GroupBox *reglages_groupbox = new GroupBox(position, name);
  //T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s", 0, 1, 0.01);

  //YawRef = new DoubleSpinBox(reglages_groupbox->NewRow(), "yaw_ref_deg:", -360, 360, 0.01,3);

  kp1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "kp1:", -10, 10, 0.01,3);
  kd1 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"kd1:",-10, 10, 0.01,3);
  scale1 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sc1:",-10, 10, 0.01,3);
  sat1 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sat1:",-10, 10, 0.01,3);

  kp2 = new DoubleSpinBox(reglages_groupbox->NewRow(), "kp2:", -10, 10, 0.01,3);
  kd2 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"kd2:",-10, 10, 0.01,3);
  scale2 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sc2:",-10, 10, 0.01,3);
  sat2 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sat2:",-10, 10, 0.01,3);

  kp3 = new DoubleSpinBox(reglages_groupbox->NewRow(), "kp3:", 0, 10, 0.01,3);
  kd3 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"kd3:",0, 10, 0.01,3);
  scale3 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sc3:",0, 10, 0.01,3);
  sat3 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"sat3:",0, 10, 0.01,3);




}

attQuatPD_impl::~attQuatPD_impl(void) {}

void attQuatPD_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(4), DataPlot::Black);
  plot->AddCurve(state->Element(5), DataPlot::Black);
  plot->AddCurve(state->Element(6), DataPlot::Black);
  plot->AddCurve(state->Element(7), DataPlot::Red);
  plot->AddCurve(state->Element(8), DataPlot::Red);
  plot->AddCurve(state->Element(9), DataPlot::Red);
}

void attQuatPD_impl::UpdateFrom(const io_data *data) {
  float p, d, total;
  float delta_t;
  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }
/*
  if (T->Value() == 0) {
    delta_t = (float)(data->DataDeltaTime() ) / 1000000000.;
  } else {
    delta_t = T->Value();
  }
  if (first_update == true) {
    delta_t = 0;
    first_update = false;
  }
*/
  input->GetMutex();

  // Generating the quaternions to obtain the quaternion error
  Quaternionf q(input->ValueNoMutex(0,0),input->ValueNoMutex(1,0),input->ValueNoMutex(2,0),input->ValueNoMutex(3,0));
  Quaternionf q_d(input->ValueNoMutex(4,0),input->ValueNoMutex(5,0),input->ValueNoMutex(6,0),input->ValueNoMutex(7,0));
  Quaternionf Omega(0.0,input->ValueNoMutex(8,0), input->ValueNoMutex(9,0), input->ValueNoMutex(10,0));


  input->ReleaseMutex();




  //Once the quaternions have been created they need to be normalied
  q.normalize();
  q_d.normalize();


  //Generating the Quaternion error
  Quaternionf q_e = q_d.conjugate()*q;

  q_e.normalize();


  q_e.w() = Sign(q_e.w())*q_e.w();


  flair::core::Quaternion Qquad(q.w(), q.x(), q.y(), q.z());
  flair::core::Quaternion Q_d(q_d.w(),q_d.x(),q_d.y(),q_d.z()); 


  
  flair::core::Euler Quad_Euler = Qquad.flair::core::Quaternion::ToEuler();
  flair::core::Euler Quad_Euler_d = Q_d.flair::core::Quaternion::ToEuler();


  //Eigen::Vector3f PropTerm = kpQ->Value()*q_e.vec();
  //Eigen::Vector3f DevTerm = kdQ->Value()*Omega.vec();

  float x = scale1->Value()*(kp1->Value()*q_e.x() + kd1->Value()*Omega.x());
  float y = scale2->Value()*(kp2->Value()*q_e.y() + kd2->Value()*Omega.y());
  float z = scale3->Value()*(kp3->Value()*q_e.z() + kd3->Value()*Omega.z());

  Saturation(x,sat1->Value());
  Saturation(y,sat2->Value());
  Saturation(z,sat3->Value());

  Eigen::Vector3f Ctrl(x,y,z);

  state->GetMutex();
  state->SetValueNoMutex(0, 0, q_e.w());
  state->SetValueNoMutex(1, 0, q_e.x());
  state->SetValueNoMutex(2, 0, q_e.y());
  state->SetValueNoMutex(3, 0, q_e.z());
  state->SetValueNoMutex(4, 0, Quad_Euler.roll);
  state->SetValueNoMutex(5, 0, Quad_Euler.pitch);
  state->SetValueNoMutex(6, 0, Quad_Euler.yaw);
  state->SetValueNoMutex(7, 0, Quad_Euler_d.roll);
  state->SetValueNoMutex(8, 0, Quad_Euler_d.pitch);
  state->SetValueNoMutex(9, 0, Quad_Euler_d.yaw);  
  state->ReleaseMutex();

  self->output->SetValue(0, 0, Ctrl[0]);
  self->output->SetValue(1, 0, Ctrl[1]);
  self->output->SetValue(2, 0, Ctrl[2]);
  self->output->SetDataTime(data->DataTime());
}

void attQuatPD_impl::Saturation(float &signal, float saturation){
  if (signal>=saturation)
    signal = saturation;
  else if (signal <= -saturation)
    signal = -saturation;
  else
    signal = signal;
}


 int attQuatPD_impl::Sign(const float &signal){
  if(signal>=0)
    return 1;
  else
    return 0;
}

float attQuatPD_impl::deg2rad(float input_deg){
  return input_deg*3.1416/180;
}

