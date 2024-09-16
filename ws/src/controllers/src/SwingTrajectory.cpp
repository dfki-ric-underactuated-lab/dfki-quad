#include "SwingTrajectory.hpp"

/*!
 * Compute foot swing trajectory with a bezier curve
 * @param phase : How far along we are in the swing (0 to 1)
 * @param swingTime : How long the swing should take (seconds)
 */
void FootSwingTrajectory::computeSwingTrajectoryBezier(double phase, double swingTime) {
  if (phase < _world_blend) {
    _p = Interpolate::cubicBezier<Vec3>(_p0, _pf, phase / _world_blend);
    _v = Interpolate::cubicBezierFirstDerivative<Vec3>(_p0, _pf, phase / _world_blend) / swingTime * _world_blend;
    _a = Interpolate::cubicBezierSecondDerivative<Vec3>(_p0, _pf, phase / _world_blend)
         / (swingTime * _world_blend * swingTime * _world_blend);
  } else {
    _p = Interpolate::cubicBezier<Vec3>(_p0, _pf, 1.0);
    _v = Interpolate::cubicBezierFirstDerivative<Vec3>(_p0, _pf, 1.0) / swingTime;
    _a = Interpolate::cubicBezierSecondDerivative<Vec3>(_p0, _pf, 1.0) / (swingTime * swingTime);
  }

  double zp, zv, za;

  if (phase < 0.5) {
    zp = Interpolate::cubicBezier(_p0[2], _p0[2] + _height, phase * 2);
    zv = Interpolate::cubicBezierFirstDerivative(_p0[2], _p0[2] + _height, phase * 2) * 2 / swingTime;
    za = Interpolate::cubicBezierSecondDerivative(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
  } else {
    zp = Interpolate::cubicBezier(_p0[2] + _height, _pf[2], phase * 2 - 1);
    zv = Interpolate::cubicBezierFirstDerivative(_p0[2] + _height, _pf[2], phase * 2 - 1) * 2 / swingTime;
    za =
        Interpolate::cubicBezierSecondDerivative(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
  }
  //  if (phase > 0.8) {
  //    zv = 0.;
  //    zp = Interpolate::cubicBezier(_p0[2] + _height, _pf[2], 0.8 * 2 - 1);
  //  }

  _p[2] = zp;
  _v[2] = zv;
  _a[2] = za;
}
