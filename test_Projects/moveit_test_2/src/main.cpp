
#include "../include/json.hpp"
#include <iostream>
#include <fstream>
#include <string.h>

using json = nlohmann::json;

int main(){
    const std::string startsignalpath =  "../../../robot/assets/cppsignal.txt";
    const char* startsignalpathasconstcharpointer = "../../../robot/assets/cppsignal.txt";
    const std::string filepath = "../../../robot/assets/boundingboxes/dynamic.json";
    while (true)
    {
        std::fstream startsignalfile(startsignalpath);
        if (!startsignalfile){
            std::cout << "EYO THE FILE IS MISSING CUH" << std::endl;
        }
        else
        {
            std::string startsignal;
            std::getline(startsignalfile, startsignal);
            if (startsignal == "0"){
                std::cout << "waiting for signal" << std::endl;
                // do something
            }
            else if (startsignal == "1"){
                std::string readbuffer;
                std::string text;
                std::fstream dynamicboxes(filepath);
                while(!dynamicboxes or std::filesystem::is_empty(filepath)){
                    dynamicboxes.close();
                    std::fstream dynamicboxes(filepath);
                }
                while(getline(dynamicboxes,readbuffer)){
                    text.append(readbuffer);
                }
                dynamicboxes.close();

                json j = json::parse(text);
                for (auto& item : j.items()) {  
                    std::cout << "Key: " << item.key() << std::endl;
                    std::cout << "Value: " << item.value() << std::endl;
                }
                break;
            } 
            else
            {
                std::cout << "YOU MESSED UP THE FILE YOU BUFFOOON!!!" << std::endl; 
                std::cout << "  " << startsignal << std::endl;
            }
        }
        startsignalfile.close();
        std::remove(startsignalpathasconstcharpointer);
        std::ofstream startsignalfile2(startsignalpath);
        startsignalfile2.write("0",1);
        startsignalfile2.close();
    }

    

    return 0;
}