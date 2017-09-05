//
// Created by acslab on 8/10/17.
//

#include "command_line_parser.h"
#include <string>
#include <iostream>

CommandLineParser::CommandLineParser(int _argc, char **_argv): argc(_argc), argv(_argv){}

bool CommandLineParser::operator[] (std::string param)
{
  int idx = -1;
  for (int i = 0; i < argc && idx == -1; i++)
  {
    if (std::string(argv[i]) == param)
    {
      idx = i;
    }
  }

  return (idx != -1);
}

std::string CommandLineParser::operator() (std::string param, std::string def_value)
{
  int idx = -1;
  for (int i = 0; i < argc && idx == -1; i++)
  {
    if (std::string(argv[i]) == param)
    {
      idx = i;
    }
  }

  if (idx == -1)
  {
    return def_value;
  }
  else
  {
    return argv[idx + 1];
  }
}
