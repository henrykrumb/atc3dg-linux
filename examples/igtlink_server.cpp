#include <iostream>
#include <math.h>
#include <cstdlib>

#include "atc3dg.hpp"

#include "igtlOSUtil.h"
#include "igtlPositionMessage.h"
#include "igtlServerSocket.h"


int main(int argc, char* argv[]) {
    int port = 18944;
    int interval = (int) (1000.0 / 80.0);

    igtl::ServerSocket::Pointer serverSocket;
    serverSocket = igtl::ServerSocket::New();
    int status = serverSocket->CreateServer(port);

    if (status < 0) {
        std::cerr << "Cannot create a server socket." << std::endl;
        exit(EXIT_FAILURE);
    }

    igtl::Socket::Pointer socket;

    auto tracker = ATC3DGTracker();
  
    while (true) {
        socket = serverSocket->WaitForConnection(1000);
        
        if (socket.IsNull()) {
            continue;
        }

        double position[3];
        double quaternion[4];
        double matrix[9];
        double orientation[3];
        double quality = 0;
        bool button = false;
        igtl::PositionMessage::Pointer positionMsg;
        positionMsg = igtl::PositionMessage::New();
        tracker.update(0, position, orientation, matrix, quaternion, &quality, &button);
        positionMsg->SetDeviceName("Tracker");
        positionMsg->SetPackType(igtl::PositionMessage::ALL); // default
        positionMsg->SetPosition((float*) position);
        positionMsg->SetQuaternion((float*) quaternion);
        positionMsg->Pack();
        socket->Send(positionMsg->GetPackPointer(), positionMsg->GetPackSize());
        igtl::Sleep(interval); // wait
    }

    socket->CloseSocket();
}