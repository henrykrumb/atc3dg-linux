#include <iostream>
#include <math.h>
#include <cstdlib>
#include <csignal>

#include "atc3dg.hpp"

#include "igtlOSUtil.h"
#include "igtlPositionMessage.h"
#include "igtlServerSocket.h"
#include "igtlTrackingDataMessage.h"
#include "igtlTransformMessage.h"
#include "igtlPointMessage.h"

#include "CLI/App.hpp"
#include "CLI/Formatter.hpp"
#include "CLI/Config.hpp"

static bool running;

void signal_handler(int signum)
{
    std::cout << "Signal " << signum << " received." << std::endl;
    if (signum == SIGINT)
    {
        std::cout << "SIGINT received." << std::endl;
        running = false;
    }
}

int main(int argc, char *argv[])
{
    int port = 18944;
    int timeout = 1000;
    bool dry = false;

    // parse command line args
    CLI::App app{"trakSTAR IGTLink Server"};
    app.add_option("-p,--port", port, "Server port");
    app.add_option("-t,--timeout", timeout, "Connection timeout");
    app.add_flag("-d,--dry", dry, "Dry run (without tracker)");
    CLI11_PARSE(app, argc, argv);

    auto server_socket = igtl::ServerSocket::New();
    int status = server_socket->CreateServer(port);
    igtl::Socket::Pointer client_socket;

    if (status < 0)
    {
        std::cerr << "Cannot create a server socket." << std::endl;
        std::cerr << "    status: " << status << std::endl;
        exit(EXIT_FAILURE);
    }

    ATC3DGTracker tracker;
    int num_sensors;
    int interval;

    if (!dry)
    {
        tracker.connect();

        num_sensors = tracker.get_number_sensors();
        std::cout << num_sensors << " sensors connected." << std::endl;
        interval = (int)(1000.0 / tracker.get_rate());
    }
    else
    {
        num_sensors = 1;
        interval = (int)(1000.0 / 80.0);
    }

    signal(SIGINT, signal_handler);


    // trakSTAR return values
    double position[] = {0.0, 0.0, 0.0};
    double quaternion[] = {0.0, 0.0, 0.0, 0.0};
    double atcmatrix[3][3] = {
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}};
    double orientation[] = {0.0, 0.0, 0.0};
    double quality = 0;
    bool button = false;

    float igtmatrix[4][4];
    // igtl::TransformElement::Pointer transform_element_ptr;

    bool connected = false;
    running = true;

    std::cout << "IGTLink Server running on port " << port << "." << std::endl;

    while (running)
    {
        client_socket = server_socket->WaitForConnection(timeout);

        if (!dry && !tracker.good())
        {
            std::cout << "Tracker disconnected." << std::endl;
            running = false;
        }

        while (running && client_socket.IsNotNull() && client_socket->GetConnected())
        {
            if (!connected)
            {
                std::cout << "Client connected." << std::endl;
                connected = true;
            }

            for (int sensor = 0; sensor < num_sensors; sensor++)
            {
                //std::cout << "sensor " << sensor << std::endl;
                auto transform_message = igtl::TransformMessage::New();
                
                std::string name;
                switch (sensor)
                {
                case 0:
                    name = "Reference";
                    break;
                case 1:
                    name = "Tool";
                    break;
                
                default:
                    name = "Unknown";
                    break;
                }
                
                transform_message->SetDeviceName(name.c_str());

                if (!dry)
                {
                    tracker.update(sensor, position, orientation, atcmatrix, quaternion, &quality, &button);
                }

                for (int j = 0; j < 3; j++)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        igtmatrix[i][j] = static_cast<float>(atcmatrix[i][j]);
                    }
                }

                igtmatrix[3][0] = 0.0f;
                igtmatrix[3][1] = 0.0f;
                igtmatrix[3][2] = 0.0f;
                igtmatrix[0][3] = position[0];
                igtmatrix[1][3] = position[1];
                igtmatrix[2][3] = position[2];
                igtmatrix[3][3] = 1.0f;
                transform_message->SetMatrix(igtmatrix);
                transform_message->Pack();
                
                if (!client_socket->Send(transform_message->GetPackPointer(), transform_message->GetPackSize()))
                {
                    break;
                }
            }

            igtl::Sleep(interval);
        }

        if (connected)
        {
            std::cout << "Client disconnected." << std::endl;
        }
        connected = false;
    }

    if (client_socket.IsNotNull())
    {
        client_socket->CloseSocket();
    }

    if (!dry)
    {
        tracker.disconnect();
    }
}
