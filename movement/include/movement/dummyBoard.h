/*
 * dummyBoard.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_DUMMYBOARD_H__INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_DUMMYBOARD_H__INCLUDED

//=======================================================
// predefined headers

//=======================================================
// new defined headers
#include "movement/board.h"

namespace movement
{

class DummyBoard : public Board
{

//Typedefs
//public:
//private:

//Public Functions
public:
  DummyBoard();
  ~DummyBoard();
  bool init_motors();
  bool init_connection(std::string port_address, int baudrate);
  void drive_linearSpeed(double v_left, double v_right);
  void drive_angularSpeed(double v_left, double v_right);

  int get_status_left();
  int get_status_right();

//Private Functions
//private:

//Public Variables
//public:

//Private Variables
//private:

};

} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_DUMMYBOARD_H__INCLUDED */
