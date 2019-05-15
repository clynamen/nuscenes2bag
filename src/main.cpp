#include "nuscenes2bag/NuScenes2Bag.hpp"
#include "nuscenes2bag/MetaDataReader.hpp"
#include <boost/program_options.hpp>

using namespace boost::program_options;

int main(const int argc, const char *argv[]) {
  try {
    std::string sampleDir;
    std::string outputBagName;


    options_description desc{"Options"};
    desc.add_options()("help,h", "show help");

    options_description inputDesc{"input"};
    inputDesc.add_options()("sample_dir,s", value<std::string>(&sampleDir),
                            "input directory")(
        "out,o", value<std::string>(&outputBagName), "output bag name");
    variables_map vm;

    desc.add(inputDesc);

    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      std::cout << desc << '\n';
    } else {
      NuScenes2Bag converter{};

      std::filesystem::path sampleDirPath(sampleDir); 
      converter.convertDirectory(sampleDir, outputBagName);
    }
  } catch (const error &ex) {
    std::cerr << ex.what() << '\n';
  }

  return 0;
}