// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file xySMC_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef XYSMC_IMPL_H
#define XYSMC_IMPL_H

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
class xySMC;
}
}

/*! \class xySMC_impl
* \brief Class defining a PID
*/

class xySMC_impl {
public:
  xySMC_impl(flair::filter::xySMC *self, const flair::gui::LayoutPosition *position,
           std::string name);
  ~xySMC_impl();
  void UseDefaultPlot(const flair::gui::LayoutPosition *position);
  void UpdateFrom(const flair::core::io_data *data);
  float i;
  bool first_update;


private:
  float Sign(float value);

  float SignH(float value);

  void Sat(float &signal, float saturation_value);



  flair::filter::xySMC *self;

  // matrix
  flair::core::Matrix *state;

  flair::gui::DoubleSpinBox *beta, *alpha;
  flair::gui::DoubleSpinBox *rho;
  flair::gui::DoubleSpinBox *satCtrl,*sclCtrl;
};

#endif // xySMC_impl_H
