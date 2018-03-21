#ifndef TRANSFORMS_H
#define TRANSFORMS_H
/************************
*  transforms.hpp
*
*  This file contains transforms and other math helpers
*  for the Udacity CarND particle filter project
*
*
* (c) 2018 Sven Kratz
************************/

#include<math.h>
#include <vector>
#include "particle_filter.h"
#include "helper_functions.h"

// map vehicle to map coordinates, based on given particle
LandmarkObs VehicleToMapWithParticle(LandmarkObs obs, Particle p);
LandmarkObs VehicleToMapWithParticle(LandmarkObs obs, Particle p)
{
  const double x_obs = obs.x;
  const double y_obs = obs.y;
  const double x_part = p.x;
  const double y_part = p.y;
  const double theta = p.theta;

  LandmarkObs out;
  out.id = obs.id;
  out.x = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
  out.y = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

  return out;
}

// Euclidean distance between two LandmarkObs
inline double EuclideanDist(LandmarkObs & oa, LandmarkObs & ob);
inline double EuclideanDist(LandmarkObs & oa, LandmarkObs & ob)
{
  const double dx = oa.x - ob.x;
  const double dy = oa.y - ob.y;
  return sqrt(dx*dx + dy*dy);
}

inline double EuclideanDist(Particle & p, float bx, float by);
inline double EuclideanDist(Particle & p, float bx, float by)
{
  const double dx = p.x - bx;
  const double dy = p.y - by;
  return sqrt(dx*dx + dy*dy);
}

// Euclidean distance between particle and LandmarkObs
inline double EuclideanDist(Particle & p, LandmarkObs & ob);
inline double EuclideanDist(Particle & p, LandmarkObs & ob)
{
  const double dx = p.x - ob.x;
  const double dy = p.y - ob.y;
  return sqrt(dx*dx + dy*dy);
}


#endif
