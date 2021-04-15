// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   Pid.cpp
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

#include "xySMC.h"
#include "xySMC_impl.h"
#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace filter {

xySMC::xySMC(const LayoutPosition *position, string name)
    : ControlLaw(position->getLayout(), name) {
  pimpl_ = new xySMC_impl(this, position, name);
  
  SetIsReady(true);
}

xySMC::~xySMC(void) { delete pimpl_; }

void xySMC::UseDefaultPlot(const gui::LayoutPosition *position) {
  pimpl_->UseDefaultPlot(position);
}

void xySMC::Reset(void) {
  pimpl_->i = 0;
  pimpl_->first_update = true;
}

float xySMC::GetIntegral(void) const { return pimpl_->i; }

void xySMC::UpdateFrom(const io_data *data) {
  pimpl_->UpdateFrom(data);
  ProcessUpdate(output);
}

void xySMC::SetValues(float pos, float ref_pos, float vel, float ref_vel) {
  input->SetValue(0, 0, pos);
  input->SetValue(1, 0, ref_pos);
  input->SetValue(2, 0, vel);
  input->SetValue(3, 0, ref_vel);
}

} // end namespace filter
} // end namespace flair
