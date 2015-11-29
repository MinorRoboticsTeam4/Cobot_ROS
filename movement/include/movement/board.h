/*
 * board.h
 * 
 */

//=======================================================
// include guard
#ifndef MOVEMENT_INCLUDE_MOVEMENT_BOARD_H__INCLUDED
#define MOVEMENT_INCLUDE_MOVEMENT_BOARD_H__INCLUDED

//=======================================================
// predefined headers
#include <boost/shared_ptr.hpp>
#include <iostream>

//=======================================================
// new defined headers

namespace movement
{

/**
 * Interface for boards
 */
class Board
{
//Typedefs
public:
  typedef boost::shared_ptr<Board> SharedPtr;
//private

//Public Functions
public:
  virtual ~Board() = 0;
  virtual bool init_motors() = 0;
  virtual bool init_connection(std::string port_address, int baudrate) = 0;
  virtual void drive_linearSpeed(double v_left, double v_right) = 0;
  virtual void drive_angularSpeed(double v_left, double v_right) = 0;

  virtual int get_status_left() = 0;
  virtual int get_status_right() = 0;

//Private Functions
//private:

//Public Variables
//public:

//Private Variables
//private:
};

} //namespace end

#endif /* MOVEMENT_INCLUDE_MOVEMENT_BOARD_H__INCLUDED */
