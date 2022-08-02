#pragma once


#ifdef _WIN32
#define _WIN32_WINNT 0x0A00
#endif

#define ASIO_STANDALONE
#include <asio.hpp>
#include <asio/ts/buffer.hpp>


using asio::ip::udp;

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <cstdlib>
#include <iostream>

#include "net.h"
#include "mcaFilter.h"
#include "resampler.h"

using namespace sp7;

enum class ManagerState
{
	IDLE,
	CONNECTED,
	WAITING_CONNECTED,
	STARTED,
	WAITING_STARTED,
	ERRORR,
};

enum class PlugInSession
{
	IDLE,
	CONNECTED,
	RESTARTING,
};



class Manager
{
private:
    constexpr static int max_length{ 256 };
    ManagerState         state{ ManagerState::IDLE };

    asio::io_context         io_context;
    udp::socket              sp7Socket;
    udp::endpoint            sender_endpoint_;
    udp::endpoint            receiver_endpoint;
    char                     datafromSp7[max_length]{};
    NetCmd::remote_command_t msgFromSp7;

    udp::socket   controlSocket;
    udp::endpoint control_sender_endpoint_;
    char          dataControl[max_length]{};

    char                     data_[max_length]{};
    uint32_t                 id{ 0 };
    NetCmd::remote_command_t msgToSp7;

    udp::socket        plugInSocket;
    asio::steady_timer t;           // Connection Management
    asio::steady_timer filterTimer; // Filter Management

    bool started{ false };
    bool connected{ false };

    McaFilter       mc;
    spdlog::logger* log;

    int                  motionDataCnt{};
    int                  msgReceived{};
    constexpr static int WAITVAL{ 4 };
    int                  waitCnt{ WAITVAL };

    PlugInSession        plugInSession{ PlugInSession::IDLE };
    constexpr static int WAITPLUGVAL{ 8 };
    int                  plugInCounter{ WAITPLUGVAL };

    constexpr static int NUM_IGNORED_SAMPLE{ 103 }; // Sample ignored for gravity
    int                  gracePeriod{ NUM_IGNORED_SAMPLE };

    constexpr static double beta{ 0.2 }; // Sample ignored for gravity
    Resampler<float>        rsample[12];

    std::chrono::time_point<std::chrono::steady_clock> time_zero; // Start Time

    
public:
    Manager(const std::string& sp7addr, spdlog::logger* log_) :
        sp7Socket(io_context, udp::endpoint(udp::v4(), 18003)),
        controlSocket(io_context, udp::endpoint(udp::v4(), 22609)),
        plugInSocket(io_context, udp::endpoint(udp::v4(), 22608)), t(io_context),
        filterTimer(io_context), log(log_)
    {
        udp::resolver resolver(io_context);
        receiver_endpoint = *resolver.resolve(udp::v4(), sp7addr, "18004").begin();

        do_receive_control_socket();
        do_receive_plugin_socket();
        do_receive_sp7_socket();
        do_wait();
        do_filter();
    }

    /**
     *
     */
    void run()
    {
        try
        {
            io_context.run();
        }
        catch (std::exception& e)
        {
            log->error("Exception: {}", e.what());
        }
    }

    /**
     *
     */
    void connect()
    {
        msgToSp7.magic = NetCmd::MAGIC_WORD;
        msgToSp7.packet_id = id++;
        msgToSp7.command = static_cast<uint16_t>(NetCmd::NetCommands::CONNECT);
        std::cout << "Connected" << msgToSp7.packet_id << std::endl;
        std::error_code ignored_error;
        sp7Socket.send_to(asio::buffer((void*)&msgToSp7, sizeof(msgToSp7)),
            receiver_endpoint, 0, ignored_error);
        this->state = ManagerState::WAITING_CONNECTED;
    }

    /**
     *
     */
    void resetSp7()
    {
        msgToSp7.magic = NetCmd::MAGIC_WORD;
        msgToSp7.packet_id = id++;
        msgToSp7.command = static_cast<uint16_t>(NetCmd::NetCommands::RESET);

        std::error_code ignored_error;
        sp7Socket.send_to(asio::buffer((void*)&msgToSp7, sizeof(msgToSp7)),
            receiver_endpoint, 0, ignored_error);
        this->state = ManagerState::IDLE;
    }

    /**
     *
     */
    void disconnect()
    {
        msgToSp7.magic = NetCmd::MAGIC_WORD;
        msgToSp7.packet_id = id++;
        msgToSp7.command = static_cast<uint16_t>(NetCmd::NetCommands::DISCONNECT);

        std::error_code ignored_error;
        sp7Socket.send_to(asio::buffer((void*)&msgToSp7, sizeof(msgToSp7)),
            receiver_endpoint, 0, ignored_error);
    }

    void alive()
    {
        msgToSp7.magic = NetCmd::MAGIC_WORD;
        msgToSp7.packet_id = id++;
        msgToSp7.command = static_cast<uint16_t>(NetCmd::NetCommands::ALIVE);

        std::error_code ignored_error;
        sp7Socket.send_to(asio::buffer((void*)&msgToSp7, sizeof(msgToSp7)),
            receiver_endpoint, 0, ignored_error);
    }

    void do_receive_control_socket()
    {
       // std::cout << "In do_receive_control_socket\n";
        controlSocket.async_receive_from(asio::buffer(dataControl, max_length),
            control_sender_endpoint_,
            [this](std::error_code, std::size_t bytes_recvd) {
                dataControl[bytes_recvd] = 0;
                std::cout << dataControl << std::endl;
                log->info(" # [{}] {}", bytes_recvd, dataControl);

                if (strncmp(dataControl, "QUIT", 4) == 0)
                {
                    //controlSocket.get_io_context().stop();
                    io_context.stop();
                    reply_to_control_socket("OK", 2);
                }

                if (strncmp(dataControl, "CONN", 4) == 0)
                {
                    std::cout << "In connect\n";
                    log->info("Request Connection");
                    waitCnt = WAITVAL;
                    this->connect();
                }

                if (strncmp(dataControl, "DISC", 4) == 0)
                {
                    this->disconnect();
                    this->plugInSession = PlugInSession::IDLE;
                    this->state = ManagerState::IDLE;
                }

                if (strncmp(dataControl, "STRT", 4) == 0)
                {
                    if (this->state == ManagerState::CONNECTED)
                    {
                        this->gracePeriod = NUM_IGNORED_SAMPLE;
                        this->started = true;
                        reply_to_control_socket("OK", 2);
                    }
                    else
                        reply_to_control_socket("NO", 2);
                }

                if (strncmp(dataControl, "STOP", 4) == 0)
                {
                    this->plugInSession = PlugInSession::IDLE;
                    if (this->state == ManagerState::CONNECTED)
                    {
                        this->started = false;
                        reply_to_control_socket("OK", 2);
                    }
                    else
                        reply_to_control_socket("NO", 2);
                }

                if (strncmp(dataControl, "STAT", 4) == 0)
                {
                    std::string str
                        = fmt::format("msgReceived {} motionDataReceived {}",
                            msgReceived, motionDataCnt);
                    reply_to_control_socket(str.c_str(), str.length());
                }

                do_receive_control_socket();
            });
    }

    /**
     *
     */
    void do_wait()
    {
        //std::cout << "In do_wait\n";
        t.expires_from_now(std::chrono::milliseconds(500));
        t.async_wait([this](std::error_code) {
            if (this->plugInSession == PlugInSession::CONNECTED)
            {
                if ((--plugInCounter) == 0)
                {
                    log->info("END SESSION");
                    this->resetSp7(); // Resets the platform
                    this->state = ManagerState::IDLE;
                    this->mc.reset(); // REsets the filter
                    this->started = false;
                    this->plugInSession = PlugInSession::IDLE;
                    std::cout << "PluginSession idled in do0wat";
                    reply_to_control_socket("ENDSESSION", 10);
                    this->mc.logCommit(); // Writes the log file
                }
            }

            if (this->state == ManagerState::CONNECTED)
            {   
                this->alive();
                std::cout << "State" << (int)(this->state) << std::endl;
            }

            if (this->state == ManagerState::WAITING_CONNECTED)
            {
                if ((waitCnt--) == 0)
                {
                    log->info("NO CONNECTION");
                    this->state = ManagerState::IDLE;
                }
            }

            do_wait();
            });
    }

    void do_filter()
    {
       // std::cout << "In do_filter\n";
        
        filterTimer.expires_from_now(std::chrono::microseconds(16667));
        filterTimer.async_wait([this](std::error_code) {
            if (plugInSession == PlugInSession::CONNECTED)
            {
                std::cout << "In filter connection" << std::endl;
                float data[12]{};
                float dt = (float)std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - time_zero)
                    .count();

                for (int i = 0; i < 12; ++i)
                    data[i] = rsample[i].filt(dt);

                double filteredData[12];

                mc.filtering(data);
                mc.getData(filteredData);


                std::cout << filteredData[1] << ", " << filteredData[2] << ", " << filteredData[3] << ',' << filteredData[4] << "," << filteredData[5] << ',' << filteredData[6] << "," << std::endl;
                log->info(" sr:  [{}] {} {}", dt, data[0], filteredData[0]);
                if (gracePeriod > 0)
                {
                    gracePeriod--;
                }
                else // Send the packet only after the grace period
                {
                    msgToSp7.magic = NetCmd::MAGIC_WORD;
                    msgToSp7.packet_id = id++;
                    msgToSp7.command
                        = static_cast<uint16_t>(NetCmd::NetCommands::EE_TARGETS);
                    for (auto i = 0; i < 6; ++i)
                    {
                        msgToSp7.ee_target_data.positions[i] = filteredData[i] * (1 << 16);
                        msgToSp7.ee_target_data.velocities[i]
                            = filteredData[i + 6] * (1 << 16);
                    }
                    std::cout << msgToSp7.packet_id <<'\n';
                    std::error_code ignored_error;
                    sp7Socket.send_to(asio::buffer((void*)&msgToSp7, sizeof(msgToSp7)),
                        receiver_endpoint, 0, ignored_error);
                }
            }

            do_filter();
            });
    }

    void reply_to_control_socket(const char* msg, int n)
    {
        //std::cout << "In reply_to_control_socket\n";
        std::error_code ignored_error;
        controlSocket.send_to(asio::buffer((void*)msg, n), control_sender_endpoint_,
            0, ignored_error);
    }

    /**
     *
     */
    void do_receive_sp7_socket()
    {
      //  std::cout << "In do_receive_sp7_socket\n";
        sp7Socket.async_receive_from(asio::buffer(datafromSp7, max_length),
            sender_endpoint_, [this](std::error_code, std::size_t) {
                memcpy((void*)&msgFromSp7, datafromSp7, sizeof(msgFromSp7));

                msgReceived++;

#ifdef DEBUG_MSGS
                log->info(" From SP7 {} {} {} {}", msgFromSp7.packet_id,
                    msgFromSp7.command, msgFromSp7.cmd_reply.orig_command,
                    msgFromSp7.cmd_reply.code);
#endif

                if (msgFromSp7.magic != NetCmd::MAGIC_WORD)
                {
                    log->error(" # BAD MAGIC  ");
                }
                else
                {
                    if (msgFromSp7.command
                        == static_cast<uint16_t>(NetCmd::NetCommands::CMD_REPLY))
                    {
                        if (state == ManagerState::WAITING_CONNECTED)
                        {
                            if (msgFromSp7.cmd_reply.orig_command
                                == static_cast<uint16_t>(NetCmd::NetCommands::CONNECT)
                                && (msgFromSp7.cmd_reply.code
                                    == static_cast<uint32_t>(
                                        NetCmd::NetReplyCode::REPLY_OK)))
                            {
                                log->info(" Connected OK ");
                                reply_to_control_socket("OK", 2);
                                this->state = ManagerState::CONNECTED;
                            }
                            else
                            {
                                log->error(" Already Connected ");
                                reply_to_control_socket("OK", 3);
                                this->state = ManagerState::CONNECTED;
                            }
                        }
                    }
                    if (msgFromSp7.command
                        == static_cast<uint16_t>(NetCmd::NetCommands::MOTION_DATA))
                    {
                        motionDataCnt++;
                    }
                }

                do_receive_sp7_socket();
            });
    }

    /**
     *
     */
    void do_receive_plugin_socket()
    {
       // std::cout << "In do_receive_plugin_socket\n";
        plugInSocket.async_receive_from(asio::buffer(data_, max_length),
            sender_endpoint_, [this](std::error_code, std::size_t) {
                if (this->started)
                {                    
                    plugInCounter = WAITPLUGVAL;
                    if (plugInSession == PlugInSession::IDLE)
                    {
                        plugInSession = PlugInSession::CONNECTED;
                        log->info("START SESSION");
                    }

                    float dt
                        = (float)std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now() - time_zero)
                        .count();

                    // PUNNING
                    float data[12];
                    memcpy(data, data_, sizeof(data));

                    for (int i = 0; i < 12; ++i)
                        rsample[i].addSample(data[i], dt);

                    log->info(" pin: [{}] {}", dt, data[0]);
                }

                do_receive_plugin_socket();
            });
    }

};
