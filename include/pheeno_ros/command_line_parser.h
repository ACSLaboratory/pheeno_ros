#ifndef COMMAND_LINE_PARSER_H
#define COMMAND_LINE_PARSER_H

#include <string>

class CommandLineParser
{

private:
  int argc;
  char **argv;

public:
  // Constructor
  CommandLineParser(int _argc, char **_argv);

  // Modules
  bool operator[] (std::string param);
  std::string operator() (std::string param, std::string def_value="-1");

};

#endif // COMMAND_LINE_PARSER_H
