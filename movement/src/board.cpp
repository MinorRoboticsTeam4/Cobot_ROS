/*
 * board.cpp
 * 
 */

//=======================================================
// predefined headers
#include <iostream>

//=======================================================
// new defined headers
#include "movement/board.h"


namespace movement
{


Board::~Board()
{
  std::cout << "Destructor Board called" << std::endl;
}


}
