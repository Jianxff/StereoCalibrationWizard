#include "basic.hpp"
#include "util/argparse.hpp"
#include "function/function.hpp"
using namespace std;
ArgumentParser _parser;

int main(int argc, char** argv){
    logging.config(logging.L_DEBUG);

    
    _parser.add_argument<string>("-f","--function")
          .choices<string>({"next","init","list","free","measure"})
          .help("calibrate function");
        
    _parser.add_argument<int>("-m","--match")
          .choices<int>({0,1,2})
          .help("0 - ELAS, 1 - SGBM, 2 - AD-Census");

    // _parser.add_argument<int>("-pi","--primary")
    //        .default_<int>(0)
    //        .help("primary camera index");
    
    // _parser.add_argument<int>("-si","--secondary")
    //        .default_<int>(-2)
    //        .help("secondary camera index");

    
    _parser.add_argument<bool>("-rt","--realtime")
          .help("real-time mode");
    
    _parser.parse_args(argc,argv);

    string func = _parser.get_value<string>("f");
    if(func == "list")
        listCamera();
    else if(func == "init")
        return initCalibrate();
    else if(func == "next")
        return nextCalibrate();
    else if(func == "free")
        return freeCalibrate();
    else if(func == "measure"){
        int match = _parser.get_value<int>("m");
        return _parser.get_value<bool>("rt") ? measureRT(match) : measure(match);
    }else{
        return 1;
    }
    return 0;
}
