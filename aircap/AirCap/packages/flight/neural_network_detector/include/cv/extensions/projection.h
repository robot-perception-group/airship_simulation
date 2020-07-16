//
// Created by glawless on 05.05.17.
//

#ifndef CV_PROJECTION_H
#define CV_PROJECTION_H

#include <opencv2/core/core.hpp>

namespace cv{

  template<typename T>
  inline Point_<T> operator*(const Point_<T>& lp, const Point_<T>& rp){
    return Point_<T>(lp.x * rp.x, lp.y * rp.y);
  }
  template<typename T>
  inline Point_<T> operator/(const Point_<T>& lp, const Point_<T>& rp){
    return Point_<T>(lp.x / rp.x, lp.y / rp.y);
  }

  template<typename T, typename T1>
  inline Point_<T> operator/(const T1& lp,const Point_<T>& rp){
    return Point_<T>((T)lp / rp.x, (T)lp / rp.y);
  }

  template<typename T, typename TS=T>
  struct projection {
    TS scale;
    T offset;

    projection(const TS& _scale, const T& _offset) : scale(_scale), offset(_offset) {};

    inline T forward(const T &in) const { return (T)((TS)in * scale) - offset; };

    inline T backward(const T &in) const { return (T)((TS)(in + offset) * (1.0/scale)); };

    inline T operator<<(const T &in) const { return backward(in); };

    inline projection<T,TS> operator>>(const projection<T,TS> &pr) const {
      return projection<T,TS>(this->scale * pr.scale,(T)(pr.scale*(TS)this->offset) + pr.offset);
    };

    inline projection<T,TS> operator<<(const projection<T,TS> &pr) const {
      return *this >> pr;
    };

  };

    // point = point >> proj
  template<typename Tpt, typename Tpr, typename Tpr2>
  inline Point_<Tpt> operator >>(const Point_<Tpt> &pt, const projection<Tpr,Tpr2> &pr) { return pr.forward(pt); };

  // alias for point2 << (proj << point1)
  template<typename Tpt, typename Tpt2>
  void operator<<(Point_<Tpt> &pt,const Point_<Tpt2> &pt2) { pt=pt2; };
  template<typename Tpt, typename Tpt2>
  void operator>>(const Point_<Tpt> &pt,Point_<Tpt2> &pt2) { pt2=pt; };



  typedef projection<Point2i, Point2f> projection2i;
}

#endif //CV_PROJECTION_H
