#include "nuscenes2rosbag/NuScenes2Rosbag.hpp"
#include <boost/program_options.hpp>

using namespace boost::program_options;

int main(const int argc, const char *argv[]) {
  try {
    std::string sampleDir;
    options_description desc{"Options"};
    desc.add_options()("help,h", "show help");

    options_description inputDesc{"input"};
    inputDesc.add_options()(
        "sample_dir,s", value<std::string>(&sampleDir), "input directory");
    variables_map vm;

    desc.add(inputDesc);

    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      std::cout << desc << '\n';
    } else {
      NuScenes2Rosbag converter{};

      std::filesystem::path b("");

      converter.convertDirectory(sampleDir, b);
    }
  } catch (const error &ex) {
    std::cerr << ex.what() << '\n';
  }


  return 0;
}