#include <iostream>
#include <math.h>
#include <cstdlib>
#include <csignal>

#include "atc3dg.hpp"

#include "igtlOSUtil.h"
#include "igtlPositionMessage.h"
#include "igtlServerSocket.h"
#include "igtlTrackingDataMessage.h"


igtl::Socket::Pointer socket;


void signalHandler(int signum) {
    if (socket.IsNotNull()) {
        socket->CloseSocket();
    }
}


int main(int argc, char* argv[]) {
    const int port = 18944;
    const int timeout = 1000;

    igtl::ServerSocket::Pointer serverSocket;
    serverSocket = igtl::ServerSocket::New();
    int status = serverSocket->CreateServer(port);

    if (status < 0) {
        std::cerr << "Cannot create a server socket." << std::endl;
        std::cerr << "    status: " << status << std::endl;
        exit(EXIT_FAILURE);
    }

    auto tracker = ATC3DGTracker();
    const int num_sensors = tracker.get_number_sensors();
    const int interval = (int) (1000.0 / tracker.get_rate());
    igtl::TrackingDataMessage::Pointer trackingMessage;
    trackingMessage = igtl::TrackingDataMessage::New();

    for (int i = 0; i < num_sensors; i++) {
        igtl::TrackingDataElement::Pointer trackingElement;
        trackingElement = igtl::TrackingDataElement::New();
        std::string name = "Channel " + i;
        trackingElement->SetName(name.c_str());
        trackingElement->SetType(igtl::TrackingDataElement::TYPE_6D);
        trackingMessage->AddTrackingDataElement(trackingElement);
    }

    double position[3];
    double quaternion[4];
    double atcmatrix[9];
    double orientation[3];
    double quality = 0;
    bool button = false;
    igtl::TrackingDataElement::Pointer ptr;

    signal(SIGINT, signalHandler);

    while (true) {
        socket = serverSocket->WaitForConnection(timeout);
        
        if (socket.IsNull()) {
            continue;
        }

        for (int sensor = 0; sensor < num_sensors; sensor++) {
            tracker.update(sensor, position, orientation, atcmatrix, quaternion, &quality, &button);
            trackingMessage->GetTrackingDataElement(sensor, ptr);
            float matrix[4][4];
            for (int j = 0; j < 3; j++) {
                for (int i = 0; i < 3; i++) {
                    matrix[i][j] = static_cast<float>(atcmatrix[i + j * 3]);
                }
            }
            matrix[3][0] = 0;
            matrix[3][1] = 0;
            matrix[3][2] = 0;
            matrix[0][3] = position[0];
            matrix[1][3] = position[1];
            matrix[2][3] = position[2];
            matrix[3][3] = 1.0;
            ptr->SetMatrix(matrix);
        }

        trackingMessage->Pack();
        socket->Send(trackingMessage->GetPackPointer(), trackingMessage->GetPackSize());
        igtl::Sleep(interval);
    }

    socket->CloseSocket();
}