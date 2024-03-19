#pragma once

namespace beacon
{
struct Para
{
  int minx_;
  int miny_;
  int minyaw_;
  int maxx_;
  int maxy_;
  int maxyaw_;
  int xw_;
  int yw_;
  int yaww_;
  double xyreso_{};
  double yawreso_;

  Para(int minx,
       int miny,
       int minyaw,
       int maxx,
       int maxy,
       int maxyaw,
       int xw,
       int yw,
       int yaww,
       double xyreso,
       double yawreso)
    : minx_(minx)
    , miny_(miny)
    , minyaw_(minyaw)
    , maxx_(maxx)
    , maxy_(maxy)
    , maxyaw_(maxyaw)
    , xw_(xw)
    , yw_(yw)
    , yaww_(yaww)
    , xyreso_(xyreso)
    , yawreso_(yawreso)
  {}

  Para(const Para&) = default;
  Para() = default;

  ~Para() = default;
};
} // namespace beacon