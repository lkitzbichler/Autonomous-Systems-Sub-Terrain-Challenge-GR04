#ifndef LCMNODE_TCPIMAGESERVER_H
#define LCMNODE_TCPIMAGESERVER_H

#include <stdint.h>

#include <iostream>
#include <memory>
#include <string>

#include "TCPStreamReader.h"

struct ImageData {
    uint32_t time_sec;
    uint32_t time_nsec;
    int width = 0;
    int height = 0;
    std::shared_ptr<uint8_t> data;
};

class TCPImageServer {
    TCPStreamReader* m_stream_reader;
    bool m_flip;
    ImageData ReadImage(const int width, const int height);
    ImageData FlipImage(ImageData const& img);

   public:
    TCPImageServer(TCPStreamReader* stream_reader, const bool flip_image = false);
    virtual ~TCPImageServer() {}
    void WaitConnect();
    bool Good();
    ImageData GetImage();
};

#endif  // LCMNODE_TCPIMAGESERVER_H
