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


#define DRY

/*
 * Make client socket global, so we can close it during cleanup.
 */
static igtl::Socket::Pointer client_socket;


int main(int argc, char* argv[]) {
    const int port = 18944;
    const int timeout = 1000;

    auto server_socket = igtl::ServerSocket::New();
    int status = server_socket->CreateServer(port);

    if (status < 0) {
        std::cerr << "Cannot create a server socket." << std::endl;
        std::cerr << "    status: " << status << std::endl;
        exit(EXIT_FAILURE);
    }

    #ifndef DRY
        ATC3DGTracker tracker;
        tracker.connect();

        const int num_sensors = tracker.get_number_sensors();
        const int interval = (int) (1000.0 / tracker.get_rate());
    #else
        const int num_sensors = 1;
        const int interval = (int) (1000.0 / 80.0);
    #endif

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
        
        while (client_socket.IsNotNull()) {
            if (!connected) {
                std::cout << "Client connected." << std::endl;
                connected = true;
            }
            for (int sensor = 0; sensor < num_sensors; sensor++) {
                #ifndef DRY
                tracker.update(sensor, position, orientation, atcmatrix, quaternion, &quality, &button);
                #endif

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
            client_socket->Send(point_message->GetPackPointer(), point_message->GetPackSize());

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

    #ifndef DRY
        tracker.disconnect();
    #endif
}