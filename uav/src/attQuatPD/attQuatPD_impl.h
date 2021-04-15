// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file attQuatPD_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef ATTQUATPD_IMPL_H
#define ATTQUATPD_IMPL_H

#include <Object.h>

namespace flair {
namespace core {
class Matrix;
class io_data;
}
namespace gui {
class LayoutPosition;
class DoubleSpinBox;
}
namespace filter {
class attQuatPD;
}
}

/*! \class attQuatPD_impl
* \brief Class defining a PID
*/

class attQuatPD_impl {
public:
  attQuatPD_impl(flair::filter::attQuatPD *self, const flair::gui::LayoutPosition *position,
           std::string name);
  ~attQuatPD_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  void Saturation(float &signal,float saturation);
  int Sign(const float &signal);
  float deg2rad(float inpunt_deg);

  float i;
  bool first_update;

private:
  flair::filter::attQuatPD *self;

  // matrix
  flair::core::Matrix *state;

  flair::gui::DoubleSpinBox *T, *kp, *ki, *kd, *sat, *sati;

  flair::gui::DoubleSpinBox *kp1,*kp2,*kp3;
  flair::gui::DoubleSpinBox *kd1,*kd2,*kd3;
  flair::gui::DoubleSpinBox *sat1,*sat2,*sat3;
  flair::gui::DoubleSpinBox *scale1,*scale2,*scale3;

  flair::gui::DoubleSpinBox *YawRef;

  flair::gui::DoubleSpinBox *kpQ, *kdQ, *satQ;
};

#endif // attQuatPD_impl_H
