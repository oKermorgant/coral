#ifndef DEBUG_MSG_H
#define DEBUG_MSG_H

#include <string>
#include <iostream>

struct DebugMsg
{
  std::string msg, my_indent;
  static int indent;
  inline DebugMsg(std::string src) : msg{src}
  {
    my_indent.resize(2*indent, ' ');
    std::cout << my_indent << "<-- " << msg << std::endl;
    indent++;
  }
  inline ~DebugMsg()
  {
    std::cout << my_indent << msg << " -->" << std::endl;
    indent--;
  }
};

#endif // DEBUG_MSG_H
