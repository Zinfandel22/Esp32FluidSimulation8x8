// particle.h
#ifndef PARTICLE_H
#define PARTICLE_H
#include "config.h"

class Particle {
 private:
  float x_pos;  // position x in meters
  float y_pos;  // position y in meters
  float vx;     // velocity x in m/s
  float vy;     // velocity y in m/s

 public:
  Particle();  // constructor

  // apply gravity acceleration in specified direction
  void addGravity(float gravity_x, float gravity_y);

  // update position based on current velocity
  void updatePosition();

  // position getters
  float getX() const;
  float getY() const;

  // position setters
  void setX(float new_x);
  void setY(float new_y);

  // velocity getters
  float getVx() const;
  float getVy() const;

  // velocity setters
  void setVx(float new_vx);
  void setVy(float new_vy);
};

#endif