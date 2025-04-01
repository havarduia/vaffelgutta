#include <fstream>
#include <sstream>
#include <cstdlib>
// Function to load file content
std::string loadFileContent(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + file_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::string loadFileContent(const std::string &file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + file_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::string processXacroFile(const std::string &xacro_path) {
    std::string command = "xacro " + xacro_path;
    FILE *pipe = popen(command.c_str(), "r");
    if (!pipe) {
        throw std::runtime_error("Failed to run xacro command.");
    }

    char buffer[128];
    std::stringstream result;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result << buffer;
    }
    pclose(pipe);

    return result.str();
}