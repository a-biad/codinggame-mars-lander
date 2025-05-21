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
    bool help_{ false };
    bool visualize_{ false };
  } opt;

  bool shouldExit_{ false };

public:
  CmdArgs(int argc, char** argv) { parseOptions(argc, argv); }

  void parseOptions(int argc, char** argv)
  {
    po::options_description desc("OPTIONS");
    desc.add_options()("help", "Put help message in output stream and exit")(
      "visualize", "Open the UI to view the details");
    po::variables_map argmap;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), argmap);
    po::notify(argmap);

    if(argmap.count("help")) {
      opt.help_ = true;
      cout << desc << "\n";
      shouldExit_ = true;
    }
    if(argmap.count("visualize")) {
      opt.visualize_ = true;
    }
  }

  auto getVisualize() { return opt.visualize_; }
  auto isHelpEnabled() { return opt.help_; }
  bool shouldExit() { return shouldExit_; }
};
