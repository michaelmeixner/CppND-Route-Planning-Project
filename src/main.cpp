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

float getInput(){
  float coord = 0.0;
  std::cout << "Enter value: ";
  std::cin >> coord;
  std::cout << std::endl;
  return coord;
}

bool checkRange(int user_input_x, int user_input_y){
  if (0.0 > user_input_x > 100.0 || 0.0 > user_input_y > 100.0) {
    std::cout << "Values must be in range 0.0 -> 100.0; Please try again." << std::endl;
    return true;
  } else {
    std::cout << "Values are valid." << std::endl;
  	return false;
  }
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
  
   	float start_x = 0.0, start_y = 0.0, end_x = 0.0, end_y = 0.0;
  	bool outOfRange = true;
  	while(outOfRange) {
      std::cout << "What is the x value for starting point?" << std::endl;
      start_x = getInput();
      std::cout<< "What is the y value for starting point?" << std::endl;
      start_y = getInput();
      outOfRange = checkRange(start_x, start_y);
    }
  	outOfRange = true;
  	while(outOfRange) {
      std::cout << "What is the x value for stopping point?" << std::endl;
      end_x = getInput();
      std::cout<< "What is the y value for stopping point?" << std::endl;
      end_y = getInput();
      outOfRange = checkRange(end_x, end_y);
    }
  
    RouteModel model{osm_data};
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

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
