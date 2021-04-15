// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   xySMC_impl.cpp
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
#include "xySMC_impl.h"
#include "xySMC.h"
#include <Matrix.h>
#include <LayoutPosition.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <iostream>
#include <AhrsData.h>
#include <DataPlot1D.h>
#include <math.h>
#include <IODevice.h>



using namespace std;

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

xySMC_impl::xySMC_impl(xySMC *self, const LayoutPosition *position, string name) {
  i = 0;
  first_update = true;
  this->self = self;

  // init matrix
  self->input = new Matrix(self, 4, 1, floatType, name);
  MatrixDescriptor *desc = new MatrixDescriptor(6, 1);
  desc->SetElementName(0, 0, "position");
  desc->SetElementName(1, 0, "ref_position");
  desc->SetElementName(2, 0, "e2");
  desc->SetElementName(3, 0, "CtrlTotal");
  desc->SetElementName(4, 0, "u");
  //desc->SetElementName(5, 0, "ref_pos");
  state = new Matrix(self, desc, floatType, name);
  
  self->AddDataToLog(state);

  delete desc;

  GroupBox *reglages_groupbox = new GroupBox(position, name);

  beta = new DoubleSpinBox(reglages_groupbox->NewRow(), "beta", -10,10,0.01,0);
  alpha = new DoubleSpinBox(reglages_groupbox->NewRow(), "alpha",-10,10,0.01,0);
  rho = new DoubleSpinBox(reglages_groupbox->NewRow(),"rho",-10,10,0.01,0);

  satCtrl = new DoubleSpinBox(reglages_groupbox->NewRow(),"SatCtrl",-10,10,0.01,0);
  sclCtrl = new DoubleSpinBox(reglages_groupbox->NewRow(),"ScaleCtrl",-10,10,0.01,0);

}

xySMC_impl::~xySMC_impl(void) {}

void xySMC_impl::UseDefaultPlot(const LayoutPosition *position) {
  DataPlot1D *plot = new DataPlot1D(position, self->ObjectName(), -1, 1);
  plot->AddCurve(state->Element(0));
  plot->AddCurve(state->Element(1), DataPlot::Green);
  plot->AddCurve(state->Element(2), DataPlot::Blue);
  plot->AddCurve(state->Element(3), DataPlot::Black);
}

void xySMC_impl::UpdateFrom(const io_data *data) {

  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }

  input->GetMutex();
  float position = input->ValueNoMutex(0,0);
  float ref_position = input->ValueNoMutex(1,0);
  float velocity = input->ValueNoMutex(2,0);
  float ref_velocity = input->ValueNoMutex(3,0);

  float e1 = position - ref_position;
  float e2 = velocity - ref_velocity;

  float sigma = beta->Value()*e1+alpha->Value()*e2;

  float smcCtrl = (1/alpha->Value())*(-rho->Value())*SignH(sigma);
  float FBLinCtrl = (1/alpha->Value())*(beta->Value()*e2);

  float CtrlTotal = FBLinCtrl + smcCtrl;

  //cout << "CtrlTotal:   " << CtrlTotal << "    ";

  float u = CtrlTotal*sclCtrl->Value();
  Sat(u,satCtrl->Value());


  state->GetMutex();
  state->SetValueNoMutex(0, 0, position);
  state->SetValueNoMutex(1, 0, ref_position);
  state->SetValueNoMutex(2, 0, e1);
  state->SetValueNoMutex(3, 0, CtrlTotal);
  state->SetValueNoMutex(4, 0, u);
  state->SetValueNoMutex(5, 0, 0.0);

  state->ReleaseMutex();

  self->output->SetValue(0, 0, u);
  self->output->SetDataTime(data->DataTime());
}


void xySMC_impl::Sat(float &signal, float satVal){
  if (signal > satVal){
    signal = satVal;
  }
  else if(signal < -satVal){
    signal = -satVal;
  }
  else
    signal =  signal;
}

float xySMC_impl::Sign(float value){
  if(value>0)
    return 1;
  else if(value < 0)
    return -1;
  else
    return 0;
}

float xySMC_impl::SignH(float value){
  return tanh(value);
}

