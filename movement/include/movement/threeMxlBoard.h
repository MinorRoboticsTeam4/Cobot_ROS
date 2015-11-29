/*
 * threeMxlboard.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_THREEMXLBOARD_H__INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_THREEMXLBOARD_H__INCLUDED

//=======================================================
// predefined headers
#include <threemxl/C3mxl.h>
#include <boost/scoped_ptr.hpp>

//=======================================================
// new defined headers
#include "movement/board.h"

namespace movement
{

class ThreeMxlBoard : public Board
{

//Typedefs
//public:
private:
  typedef boost::scoped_ptr<CDxlGroup> MotorsPtr;

//Public Functions
public:
  ThreeMxlBoard();
  ~ThreeMxlBoard();
  bool init_motors();
  bool init_connection(std::string port_address, int baudrate);
  void drive_linearSpeed(double v_left, double v_right);
  void drive_angularSpeed(double v_left, double v_right);

  int get_status_left();
  int get_status_right();

//Private Functions
private:

  /** Clip the value between min<=x<=max
   *
   * @param n value to be clipped
   * @param lower lower bound
   * @param upper upper bound
   * @return clipped value
   */
  template<typename T>
    T clip(const T& n, const T& lower, const T& upper)
    {
      return std::max(lower, std::min(n, upper));
    }

//Public Variables
//public:

//Private Variables
private:
  MotorsPtr motors;
  LxSerial serial_port;

};

} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_THREEMXLBOARD_H__INCLUDED */
