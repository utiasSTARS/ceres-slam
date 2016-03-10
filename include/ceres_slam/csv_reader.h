#ifndef CSV_READER_HPP_
#define CSV_READER_HPP_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

//! Class to read and parse CSV files
class CSVReader {
   public:
    //! Open the file
    CSVReader(const std::string filename, const char delimeter = ',')
        : file_(filename), del_(delimeter) {
        if (isOpen()) {
            eof_ = !std::getline(file_, next_line_);
        } else {
            std::cerr << "Error: Could not open file " << filename << std::endl;
        }
    };
    //! Close the file when the reader gets destroyed
    ~CSVReader() { file_.close(); }

    //! Read and parse the next line
    std::vector<std::string> getLine() {
        std::vector<std::string> line_tokens;
        line_tokens = split(next_line_, del_);

        if (isOpen()) {
            eof_ = !std::getline(file_, next_line_);
        }

        return line_tokens;
    }

    //! Skip a line
    void skipLine() {
        std::string line;

        if (isOpen()) {
            eof_ = !std::getline(file_, next_line_);
        }
    }

    //! Check if the file was successfully opened
    bool isOpen() { return file_.is_open(); }

    //! Check if we have reached the end of the file
    bool atEOF() { return eof_; }

   private:
    //! Input file stream
    std::ifstream file_;
    //! Delimeter for parsing lines
    char del_;
    //! Storage for the next line in the file (so that we get eof_ == true
    //! without returning an empty vector)
    std::string next_line_;
    //! Have we reached the end of the file?
    bool eof_;

    //! Split a delimited string into a vector of tokens
    std::vector<std::string> split(const std::string str, const char del) {
        std::stringstream ss(str);  // Copy the string into a stream
        std::vector<std::string> tokens;
        std::string tok;

        while (std::getline(ss, tok, del)) {
            tokens.push_back(tok);
        }

        return tokens;
    }
};

#endif /* end of include guard: CSV_READER_HPP_ */
