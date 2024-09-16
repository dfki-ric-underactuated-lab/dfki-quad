#pragma once

class GaitInterface {
 protected:
  GaitInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  virtual double get_t_stance(unsigned int leg) const = 0;
  virtual ~GaitInterface() = default;
};
