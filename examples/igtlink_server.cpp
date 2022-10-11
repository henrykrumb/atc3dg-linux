#include <iostream>
#include <math.h>
#include <cstdlib>
#include <csignal>

#include "atc3dg.hpp"

#include "igtlOSUtil.h"
#include "igtlPositionMessage.h"
#include "igtlServerSocket.h"
#include "igtlTrackingDataMessage.h"
#include "igtlPointMessage.h"

#include "CLI/App.hpp"
#include "CLI/Formatter.hpp"
#include "CLI/Config.hpp"


int main(int argc, char* argv[]) {
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

    if (status < 0) {
        std::cerr << "Cannot create a server socket." << std::endl;
        std::cerr << "    status: " << status << std::endl;
        exit(EXIT_FAILURE);
    }

    ATC3DGTracker tracker;
    int num_sensors;
    int interval;

    if (!dry) {
        tracker.connect();

        num_sensors = tracker.get_number_sensors();
        interval = (int) (1000.0 / tracker.get_rate());
    } else {
        num_sensors = 1;
        interval = (int) (1000.0 / 80.0);
    }

    // trakSTAR return values
    double position[] = { 0.0, 0.0, 0.0 };
    double quaternion[] = {0.0, 0.0, 0.0, 0.0};
    double atcmatrix[] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };
    double orientation[] = { 0.0, 0.0, 0.0 };
    double quality = 0;
    bool button = false;

    // construct the tracking message beforehand
    auto tracking_message = igtl::TrackingDataMessage::New();
    auto point_message = igtl::PointMessage::New();

    for (int i = 0; i < num_sensors; i++) {
        auto tracking_element = igtl::TrackingDataElement::New();
        std::string name = "CHANNEL_" + i;
        tracking_element->SetName(name.c_str());
        tracking_element->SetType(igtl::TrackingDataElement::TYPE_6D);
        tracking_message->AddTrackingDataElement(tracking_element);

        auto point_element = igtl::PointElement::New();
        point_element->SetName(name.c_str());
        point_message->AddPointElement(point_element);
    }
    tracking_message->SetDeviceName("Tracker");
    point_message->SetDeviceName("Tracker");

    float igtmatrix[4][4];
    igtl::TrackingDataElement::Pointer tracking_element_ptr;
    igtl::PointElement::Pointer point_element_ptr;

    bool running = true;
    bool connected = false;

    std::cout << "IGTLink Server running on port " << port << "." << std::endl;


    while (running)
    {
        client_socket = server_socket->WaitForConnection(timeout);

        while (client_socket.IsNotNull() && client_socket->GetConnected()) {
            if (!connected) {
                std::cout << "Client connected." << std::endl;
                connected = true;
            }
            for (int sensor = 0; sensor < num_sensors; sensor++) {
                if (!dry) {
                    tracker.update(sensor, position, orientation, atcmatrix, quaternion, &quality, &button);
                }

                for (int j = 0; j < 3; j++) {
                    for (int i = 0; i < 3; i++) {
                        igtmatrix[i][j] = static_cast<float>(atcmatrix[i + j * 3]);
                    }
                }
                igtmatrix[3][0] = 0.0f;
                igtmatrix[3][1] = 0.0f;
                igtmatrix[3][2] = 0.0f;
                igtmatrix[0][3] = position[0];
                igtmatrix[1][3] = position[1];
                igtmatrix[2][3] = position[2];
                igtmatrix[3][3] = 1.0f;

                tracking_message->GetTrackingDataElement(sensor, tracking_element_ptr);
                tracking_element_ptr->SetMatrix(igtmatrix);

                point_message->GetPointElement(sensor, point_element_ptr);
                point_element_ptr->SetPosition(position[0], position[1], position[2]);
            }

            tracking_message->Pack();
            client_socket->Send(tracking_message->GetPackPointer(), tracking_message->GetPackSize());
            point_message->Pack();
            int status = client_socket->Send(point_message->GetPackPointer(), point_message->GetPackSize());
            if (!status) {
                break;
            }

            igtl::Sleep(interval);
        }

        if (connected) {
            std::cout << "Client disconnected." << std::endl;
        }
        connected = false;
    }

    if (client_socket.IsNotNull()) {
        client_socket->CloseSocket();
    }

    if (!dry) {
        tracker.disconnect();
    }
}