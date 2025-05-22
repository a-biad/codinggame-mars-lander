#pragma once

#include <boost/program_options.hpp>
#include <boost/program_options/detail/parsers.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/positional_options.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>

namespace po = boost::program_options;

class CmdArgs
{
  struct Options
  {
    bool help{ false };
    bool visualize{ false };
    bool test{ false };
  } opt;

  bool shouldExit_{ false };

public:
  CmdArgs(int argc, char** argv) { parseOptions(argc, argv); }

  void parseOptions(int argc, char** argv)
  {
    po::options_description desc("OPTIONS");
    desc.add_options()                                       //
      ("help,h", "Put help message in output stream and exit") //
      ("visualize,V", "Open the UI to view the details")
      ("test,t", "Run tests and exit");
    po::variables_map argmap;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), argmap);
    po::notify(argmap);

    if(argmap.count("help")) {
      opt.help = true;
      cout << desc << "\n";
      shouldExit_ = true;
    }
    if(argmap.count("visualize")) {
      opt.visualize = true;
    }

    if(argmap.count("test")) {
      opt.test = true;
    }
  }

  auto getVisualize() { return opt.visualize; }
  auto isHelpEnabled() { return opt.help; }
  bool shouldExit() { return shouldExit_; }
  auto getOptions() {return opt;}
};
