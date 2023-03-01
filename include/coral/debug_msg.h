#ifndef DEBUG_MSG_H
#define DEBUG_MSG_H

#pragma once

#include <string>
#include <iostream>

struct DebugMsg
{
  std::string msg;
  static int indent;
  inline DebugMsg(std::string src) : msg{src}
  {
    for(int i = 0; i < indent; ++i)
      msg = " " + msg;
    std::cout << msg << ": begin" << std::endl;
    indent++;
  }
  inline ~DebugMsg()
  {
    std::cout << msg << ": end" << std::endl;
    indent--;
  }
};

int DebugMsg::indent{};

#endif // DEBUG_MSG_H
