#ifndef DEBUG_MSG_H
#define DEBUG_MSG_H

#include <string>
#include <iostream>

struct DebugMsg
{
  std::string msg;
  inline DebugMsg(std::string msg) : msg{msg}
  {
    std::cout << "begin " << msg << std::endl;
  }
  inline ~DebugMsg()
  {
    std::cout << "end " << msg << std::endl;
  }
};

#endif // DEBUG_MSG_H
