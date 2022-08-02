#ifndef MEMBUF_H
#define MEMBUF_H

#include <array>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

static std::string generateName(const std::string &baseName)
{
  auto timeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now().time_since_epoch())
                       .count();
  std::stringstream ss;
  ss << baseName << timeStamp << ".log";
  return ss.str();
}

/*!
 * \brief The MemLogger class
 * Use a circular buffer do mantain a log in memory and then writes it into a
 * files
 */
class MemLogger {
public:
  MemLogger(const size_t _N, const std::string &filename_)
      : N{_N}, buf{new char[N]}, filename(filename_), index(0), full(false)
  {
  } ///< default constructor

  void addRow(const char *line)
  {
    const auto len = strlen(line); // String Length

    auto runningLen = len;
    auto readIdx = 0;

    while (runningLen > 0) {
      auto space = N - index;
      auto toCpy = std::min(space, runningLen);

      memcpy(&buf[index], &line[readIdx], toCpy);

      readIdx += toCpy;
      index += toCpy;
      runningLen -= toCpy;

      if (index == N) {
        index = 0;
        full = true;
      }
    }

    buf[index] = '\n';
    if (++index == N) {
      index = 0;
      full = true;
    }
  }

  /**
   * @brief write to disk the log
   * Writes the logs to disk
   */
  void commit()
  {
    std::ofstream myfile;
    myfile.open(generateName(filename).c_str());

    if (!full)
      myfile.write(buf.get(), index);
    else {
      auto ptr = buf.get();
      myfile.write(&ptr[index], N - index);
      myfile.write(ptr, index);
    }

    myfile.flush();
    myfile.close();

    index = 0;
    full = false;
  }

private:
  const size_t N;
  std::unique_ptr<char[]> buf;
  const std::string filename;
  size_t index;
  bool full;
};

#endif