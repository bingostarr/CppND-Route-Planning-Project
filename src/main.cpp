#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static const float COORD_DEF = 50.0;
static const float COORD_MIN = 0.0;
static const float COORD_MAX = 100.0;

void ReadValueFromUserInput(float &var, const std::string& var_name) {
  float value = COORD_DEF;

  std::cout << "Please enter value for <" + var_name + ">: ";
  std::cin >> value;

  while (std::cin.fail() || value < COORD_MIN || value > COORD_MAX) {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "Wrong input value (" << COORD_MIN << " <= val <= " << COORD_MAX << "). Please try again: ";
    std::cin >> value;
  }

  var = value;
}

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
    // user input for these values using std::cin. Pass the user input to the
    // RoutePlanner object below in place of 10, 10, 90, 90.
    float start_x = COORD_DEF, start_y = COORD_DEF, end_x = COORD_DEF, end_y = COORD_DEF;

    ReadValueFromUserInput(start_x, "start_x");
    ReadValueFromUserInput(start_y, "start_y");
    ReadValueFromUserInput(end_x, "end_x");
    ReadValueFromUserInput(end_y, "end_y");

    std::cout << "\nUsing starting coordinates: " << start_x << "," << start_y << std::endl;
    std::cout << "Using ending coordinates: " << end_x << "," << end_y << std::endl;
    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
