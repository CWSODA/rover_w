#include "lidar.hpp"

#include <string>
#include <iostream>

#include <fstream>

int main() {
    std::cout << "START\n";
    LidarParser lp;

    std::fstream input_file;
    // input_file.open("../fake_input", std::fstream::in);
    input_file.open("../input", std::fstream::in);

    int a;
    input_file >> std::hex;
    std::cout << std::hex << std::uppercase;

    int count = 0;
    int max = 1000000;
    while (input_file >> a) {
        lp.parse_byte(a);

        if (++count >= max) break;
    }

    return 0;
}