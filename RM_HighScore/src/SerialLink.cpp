// 必须放在所有系统头文件之前：
// -std=c++17 是严格模式，glibc 默认不暴露 B115200 等扩展常量
#if (defined(__linux__) || defined(__APPLE__)) && !defined(_DEFAULT_SOURCE)
#define _DEFAULT_SOURCE 1
#endif

#include "SerialLink.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#if defined(__linux__) || defined(__APPLE__)

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace
{
    speed_t toSpeed(int baud, bool& ok)
    {
        ok = true;

        switch (baud)
        {
            case 9600:   return B9600;
            case 19200:  return B19200;
            case 38400:  return B38400;
#if defined(B57600)
            case 57600:  return B57600;
#endif
#if defined(B115200)
            case 115200: return B115200;
#endif
#if defined(B230400)
            case 230400: return B230400;
#endif
            default:     ok = false; return B9600;
        }
    }
}

SerialLink::~SerialLink()
{
    close();
}

bool SerialLink::open(const std::string& port, int baud)
{
    close();

    // O_NONBLOCK：串口没插/没配好时不会卡死在 open 上
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd_ < 0)
    {
        std::cerr << "打开串口失败：" << port << "（" << std::strerror(errno) << "）" << std::endl;
        std::cerr << "检查：ls /dev/ttyUSB* /dev/ttyACM*；"
                     "权限问题执行 sudo usermod -aG dialout $USER 后重新登录" << std::endl;
        return false;
    }

    termios tty {};
    bool speedOk = true;

    if (tcgetattr(fd_, &tty) != 0)
    {
        std::cerr << "tcgetattr 失败：" << std::strerror(errno) << std::endl;
        close();
        return false;
    }

    // 等价于 cfmakeraw，但手写（严格模式下 cfmakeraw 可能没有声明）
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // 8N1，无流控
    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
    tty.c_cflag |= (CLOCAL | CREAD | CS8);

#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;
#endif

    const speed_t speed = toSpeed(baud, speedOk);

    if (!speedOk)
    {
        std::cerr << "不支持的波特率：" << baud << "，改用 9600" << std::endl;
    }

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;   // 0.1 秒

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "tcsetattr 失败：" << std::strerror(errno) << std::endl;
        close();
        return false;
    }

    tcflush(fd_, TCIOFLUSH);

    return true;
}

bool SerialLink::sendLine(const std::string& line)
{
    if (fd_ < 0)
    {
        return false;
    }

    const std::string data = line + "\n";

    std::size_t sent = 0;

    while (sent < data.size())
    {
        const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);

        if (n > 0)
        {
            sent += static_cast<std::size_t>(n);
            continue;
        }

        if (n < 0 && errno == EINTR)
        {
            continue;
        }

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        std::cerr << "串口写入失败：" << std::strerror(errno) << std::endl;
        return false;
    }

    // 等数据真正发出去，避免程序退出时被截断
    tcdrain(fd_);

    return true;
}

void SerialLink::flush()
{
    buffer_.clear();

    if (fd_ >= 0)
    {
        tcflush(fd_, TCIFLUSH);
    }
}

std::string SerialLink::readLine(int timeoutMs)
{
    if (fd_ < 0)
    {
        return std::string();
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while (true)
    {
        const std::size_t newline = buffer_.find('\n');

        if (newline != std::string::npos)
        {
            std::string line = buffer_.substr(0, newline);
            buffer_.erase(0, newline + 1);

            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }

            return line;
        }

        char chunk[128];
        const ssize_t n = ::read(fd_, chunk, sizeof(chunk));

        if (n > 0)
        {
            buffer_.append(chunk, static_cast<std::size_t>(n));
            continue;
        }

        if (std::chrono::steady_clock::now() > deadline)
        {
            return std::string();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void SerialLink::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }

    buffer_.clear();
}

#else   // 非 POSIX 平台：编译通过但不工作

SerialLink::~SerialLink() {}

bool SerialLink::open(const std::string& port, int)
{
    std::cerr << "串口只支持 Linux / macOS（当前平台不可用）：" << port << std::endl;
    return false;
}

bool SerialLink::sendLine(const std::string&) { return false; }

std::string SerialLink::readLine(int) { return std::string(); }

void SerialLink::close() {}

void SerialLink::flush() {}

#endif
