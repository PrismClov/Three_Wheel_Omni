#include "CSerialPort/SerialPort.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#ifdef _WIN32
#include <cstdlib>
#endif

#pragma pack(push, 1)
struct Struct_USB_Rx_Data
{
    char info[5];
    uint16_t x_velocity;
    uint16_t y_velocity;
    uint16_t omega;
};
#pragma pack(pop)

static bool StartsWith(const std::string& value, const std::string& prefix)
{
    return value.compare(0, prefix.size(), prefix) == 0;
}

static uint16_t Math_Float_To_Int(float val, float min_val, float max_val)
{
    float normalized = (val - min_val) / (max_val - min_val);
    if (normalized < 0.0f) {
        normalized = 0.0f;
    }
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }

    return static_cast<uint16_t>(normalized * 65535.0f + 0.5f);
}

static std::string BuildPortName(const std::string& input)
{
#ifdef _WIN32
    if (input.empty()) {
        return "COM3";
    }
    if (StartsWith(input, "COM") || StartsWith(input, "\\\\.\\")) {
        return input;
    }
    return "COM" + input;
#else
    if (input.empty()) {
        return "/dev/ttyACM0";
    }
    if (StartsWith(input, "/dev/")) {
        return input;
    }
    if (StartsWith(input, "tty")) {
        return "/dev/" + input;
    }
    if (StartsWith(input, "ACM") || StartsWith(input, "USB")) {
        return "/dev/tty" + input;
    }
    return "/dev/ttyACM" + input;
#endif
}

static void WaitBeforeExit()
{
#ifdef _WIN32
    system("pause");
#endif
}

int main(int argc, char* argv[])
{
    std::string input;

    if (argc > 1 && argv[1] != nullptr) {
        input = argv[1];
    } else {
#ifdef _WIN32
        std::cout << "serial number or port [COM3]: ";
#else
        std::cout << "serial port [/dev/ttyACM0]: ";
#endif
        std::getline(std::cin, input);
    }

    const std::string portName = BuildPortName(input);
    std::cout << "Opening serial: " << portName << std::endl;

    itas109::CSerialPort cdc;
    cdc.init(portName.c_str(),
             itas109::BaudRate115200,
             itas109::ParityNone,
             itas109::DataBits8,
             itas109::StopOne,
             itas109::FlowNone);
    cdc.setOperateMode(itas109::SynchronousOperate);

    if (!cdc.open()) {
        std::cout << "Failed to open " << portName << std::endl;
        std::cout << "Error: " << cdc.getLastErrorMsg() << std::endl;
#ifndef _WIN32
        std::cout << "If this is a permission issue, add your user to dialout and log in again:" << std::endl;
        std::cout << "  sudo usermod -aG dialout $USER" << std::endl;
#endif
        WaitBeforeExit();
        return 1;
    }

    std::cout << "Port opened successfully!" << std::endl;
    std::cout << "Sending Struct_USB_Rx_Data binary frames..." << std::endl;

    Struct_USB_Rx_Data tx_data{};
    std::memcpy(tx_data.info, "CMD", 4);

    int frame_count = 0;
    int vx = -8;
    int vy = 0;
    int vo = 0;

    while (frame_count < 10000) {
        vx++;
        if (vx > 8) {
            vx = -8;
            vy++;
        }
        if (vy > 8) {
            vy = -8;
            vo++;
        }
        if (vo > 8) {
            break;
        }

        tx_data.x_velocity = Math_Float_To_Int(static_cast<float>(vx), -8.0f, 8.0f);
        tx_data.y_velocity = Math_Float_To_Int(static_cast<float>(vy), -8.0f, 8.0f);
        tx_data.omega = Math_Float_To_Int(static_cast<float>(vo), -8.0f, 8.0f);

        const int bytesWritten = cdc.writeData(reinterpret_cast<char*>(&tx_data), sizeof(tx_data));
        std::cout << "[" << frame_count << "] vx=" << vx
                  << " vy=" << vy << " vo=" << vo
                  << " | raw: x=" << tx_data.x_velocity
                  << " y=" << tx_data.y_velocity
                  << " o=" << tx_data.omega
                  << " | sent " << bytesWritten << " bytes" << std::endl;

        frame_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "Done. Sent " << frame_count << " frames." << std::endl;

    char readBuffer[256] = {0};
    const int bytesRead = cdc.readData(readBuffer, sizeof(readBuffer) - 1);
    if (bytesRead > 0) {
        readBuffer[bytesRead] = '\0';
        std::cout << "Received: " << readBuffer << std::endl;
    }

    cdc.close();
    std::cout << "Serial port closed" << std::endl;

    WaitBeforeExit();
    return 0;
}
