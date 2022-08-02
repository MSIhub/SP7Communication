#include "Cueing.h"

int main(int, char**)
{
    // initialize Logging
    auto console = spdlog::stdout_color_mt("console");

    // Verify that a file exist
    std::string sp7address;
    try
    {
        sp7address = "172.16.0.1";
        console->info("{}", sp7address);
    }
    catch (...)
    {
        console->info(" Missing or wrong config.yaml");
        return -1;
    }

    Manager m{ sp7address, console.get() };
    m.run();

    return 0;
}