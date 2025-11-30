#include "particle.h"

// constructor
Particle::Particle() {
  x_pos = 0;
  y_pos = 0;
  vx = 0;
  vy = 0;
}

// apply gravity acceleration in the specified direction
// gravity_x and gravity_y are in m/s^2
void Particle::addGravity(float gravity_x, float gravity_y) {
  vx = vx + DELTA_T * gravity_x;
  vy = vy + DELTA_T * gravity_y;
}

// update position based on current velocity
void Particle::updatePosition() {
  x_pos = x_pos + DELTA_T * vx;
  y_pos = y_pos + DELTA_T * vy;
}

// position getters
float Particle::getX() const {
  return x_pos;
}
float Particle::getY() const {
  return y_pos;
}

// position setters
void Particle::setX(float new_x) {
  x_pos = new_x;
}
void Particle::setY(float new_y) {
  y_pos = new_y;
}

// velocity getters
float Particle::getVx() const {
  return vx;
}
float Particle::getVy() const {
  return vy;
}

// velocity setters
void Particle::setVx(float new_vx) {
  vx = new_vx;
}
void Particle::setVy(float new_vy) {
  vy = new_vy;
}