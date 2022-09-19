#include <iostream>
#include <math.h>
#include <cstdlib>

#include "atc3dg.hpp"

#include "igtlOSUtil.h"
#include "igtlPositionMessage.h"
#include "igtlServerSocket.h"


int main(int argc, char* argv[]) {
    const int port = 18944;
    const int interval = (int) (1000.0 / 80.0);
    const int timeout = 1000;

    igtl::ServerSocket::Pointer serverSocket;
    serverSocket = igtl::ServerSocket::New();
    int status = serverSocket->CreateServer(port);

    if (status < 0) {
        std::cerr << "Cannot create a server socket." << std::endl;
        std::cerr << "    status: " << status << std::endl;
        exit(EXIT_FAILURE);
    }

    igtl::Socket::Pointer socket;

    auto tracker = ATC3DGTracker();
    const int num_sensors = tracker.get_number_sensors();
    igtl::TrackingDataMessage::Pointer trackingMessage;
    trackingMessage = igtl::TrackingDataMessage::New();

    for (int i = 0; i < num_sensors; i++) {
        igtl::TrackingDataElement::Pointer trackingElement;
        trackingElement = igtl::TrackingDataElement::New();
        std::string name = "Channel " + i;
        trackingElement->SetName(name.c_str());
        trackingElement->SetType(igtl::TrackingDataElement::TYPE_6D);
        trackingMessage->AddTrackingElement(trackingElement);
    }

    double position[3];
    double quaternion[4];
    double matrix[9];
    double orientation[3];
    double quality = 0;
    bool button = false;
    igtl::TrackingDataElement::Pointer ptr;

    while (true) {
        socket = serverSocket->WaitForConnection(timeout);
        
        if (socket.IsNull()) {
            continue;
        }

        for (int sensor = 0; sensor < num_sensors; sensor++) {
            tracker.update(sensor, position, orientation, matrix, quaternion, &quality, &button);
            trackingMessage->GetTrackingElement(sensor, ptr);
            ptr->SetMatrix((float*) matrix);            
        }

        trackingMessage->Pack();
        trackingMessage->Send(trackingMessage->GetPackPointer(), trackingMessage->GetPackSize());
        igtl::Sleep(interval);
    }

    socket->CloseSocket();
}