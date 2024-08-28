
#include <iostream>
#include <fstream>

// Function to read a file and return raw bytes as a vector of chars
std::vector<unsigned char> readFile(const std::string& filename) {
    // Open the file in binary mode
    std::ifstream file(filename, std::ios::binary);
    
    if (!file) {
        throw std::runtime_error("Unable to open file");
    }

    // Determine the file size
    file.seekg(0, std::ios::end);
    std::streamsize fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // Create a buffer to hold the file contents
    std::vector<unsigned char> buffer(fileSize);

    // Read the file contents into the buffer
    
    if (!file.read(reinterpret_cast<char*>(buffer.data()), fileSize)) {
        throw std::runtime_error("Error reading file");
    }

    return buffer;
};