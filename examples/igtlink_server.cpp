#include <iostream>
#include <math.h>
#include <cstdlib>
#include <csignal>

#include "atc3dg.hpp"

#include "igtlOSUtil.h"
#include "igtlPositionMessage.h"
#include "igtlServerSocket.h"
#include "igtlTrackingDataMessage.h"


igtl::Socket::Pointer client_socket;


void cleanup() {
    if (client_socket.IsNotNull()) {
        client_socket->CloseSocket();
    }
}


void signal_handler(int signum) {
    cleanup();
}


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

    ATC3DGTracker tracker;
    const int num_sensors = tracker.get_number_sensors();
    const int interval = (int) (1000.0 / tracker.get_rate());
    auto trackingMessage = igtl::TrackingDataMessage::New();

    for (int i = 0; i < num_sensors; i++) {
        auto trackingElement = igtl::TrackingDataElement::New();
        std::string name = "Channel " + i;
        trackingElement->SetName(name.c_str());
        trackingElement->SetType(igtl::TrackingDataElement::TYPE_6D);
        trackingMessage->AddTrackingDataElement(trackingElement);
    }

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

    float igtmatrix[4][4];
    igtl::TrackingDataElement::Pointer ptr;

    bool running = true;
    signal(SIGINT, signal_handler);

    while (running)
    {
        client_socket = server_socket->WaitForConnection(timeout);
        
        if (client_socket.IsNull()) {
            continue;
        }

        for (int sensor = 0; sensor < num_sensors; sensor++) {
            tracker.update(sensor, position, orientation, atcmatrix, quaternion, &quality, &button);
            trackingMessage->GetTrackingDataElement(sensor, ptr);
            
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
            ptr->SetMatrix(igtmatrix);
        }

        trackingMessage->Pack();
        client_socket->Send(trackingMessage->GetPackPointer(), trackingMessage->GetPackSize());
        igtl::Sleep(interval);
    }

    cleanup();
}