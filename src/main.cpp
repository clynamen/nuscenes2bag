#include "nuscenes2bag/MetaDataReader.hpp"
#include "nuscenes2bag/NuScenes2Bag.hpp"
#include <boost/program_options.hpp>

using namespace boost::program_options;
using namespace nuscenes2bag;

int
main(const int argc, const char* argv[])
{
  try {
    std::string sampleDir;
    std::string outputBagName;
    int32_t threadNumber;
    int32_t sceneNumber = -1;

    options_description desc{ "Options" };
    desc.add_options()("help,h", "show help");

    options_description inputDesc{ "input" };
    inputDesc.add_options()(
      "scene_number,n", value<int32_t>(&sceneNumber), "only convert a given scene")(
      "sample_dir,s", value<std::string>(&sampleDir), "input directory")(
      "out,o", value<std::string>(&outputBagName), "output bag name")(
      "jobs,j",
      value<int32_t>(&threadNumber),
      "number of jobs (thread number)");
    variables_map vm;

    desc.add(inputDesc);

    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help")) {
      std::cout << desc << '\n';
    } else {
      NuScenes2Bag converter{};

      std::filesystem::path sampleDirPath(sampleDir);

      std::optional<int32_t> sceneNumberOpt;
      if(sceneNumber >= 0) {
        sceneNumberOpt = sceneNumber;
      }
      converter.convertDirectory(sampleDir, outputBagName, threadNumber, sceneNumberOpt);
    }
  } catch (const error& ex) {
    std::cerr << ex.what() << '\n';
  }

  return 0;
}