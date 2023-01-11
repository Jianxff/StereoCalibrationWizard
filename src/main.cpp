#include "basic.hpp"
#include "function/function.hpp"
using namespace std;

#define MODE_LIST   0
#define MODE_INIT   1
#define MODE_NEXT   2
#define MODE_FREE   3
#define MODE_MEAS   4

vector<int> read_args(int argc, char** argv);

int main(int argc, char** argv){
    logging.config(logging.L_DEBUG);
    vector<int> args = read_args(argc,argv);
    switch(args[0]){
        case MODE_LIST: listCamera();   return 0;
        case MODE_INIT: return initCalibrate();
        case MODE_NEXT: return nextCalibrate();
        case MODE_FREE: return freeCalibrate();
        case MODE_MEAS: {
            if(args[2]) return measureRT(args[1]);
            else        return measure(args[1]);
        }
    }
    return 1;
}


vector<int> read_args(int argc, char** argv){
    vector<int> args = {MODE_LIST,Measure::ELAS,0};
    if(argc == 1)   return args;
    if(argc >= 2){
        string str = string(argv[1]);
        if(str == "list")       args[0] = MODE_LIST;
        else if(str == "init")  args[0] = MODE_INIT;
        else if(str == "next")  args[0] = MODE_NEXT;
        else if(str == "free")  args[0] = MODE_FREE;
        else if(str == "measure") args[0] = MODE_MEAS;
        else logging.critical(-1,"undefined mode %s\n",argv[1]);
    }

    if(argc >= 3){
        string str = string(argv[2]);
        if(str == "elas")       args[1] = Measure::ELAS;
        else if(str == "sgbm")  args[1] = Measure::SGBM;
        else logging.critical(-1,"undefined alg %s\n",argv[2]);
    }

    if(argc >= 4){
        string str = string(argv[3]);
        if(str == "rt" || str == "RT")
            args[2] = 1;
        else logging.critical(-1,"undefined arg %s\n",argv[3]);
    }
    return args;
}