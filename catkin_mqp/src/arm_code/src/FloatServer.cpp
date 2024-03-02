#include <iostream>
#include <vector>
#include "FloatPacketType.h"
#include <unordered_map>
#include <thread>

class FloatServer : public AbstractSimpleComsServer {
    private:
        FloatPacketType packet;
        std::vector<float> data;
    public:
        FloatServer(int id) : FloatServer(id, 64) {}

        FloatServer(int id, int size) : packet(id, size) {
            //this->data.resize((this->packet.size()));
            FloatPacketType n(id, size);
            this->packet = n;
            std::vector<float> ndata(packet.getDownstream().size());
            this-> data = ndata;

        }

        FloatPacketType getPacket() {
            return this->packet;
        }
        bool event(std::vector<float> packet) {
            for (int i = 0; i < this->data.size(); i++)
                this->data[i] = packet[i];
            bool ret = event(this->data);
            for (int j = 0; j < this->data.size(); j++)
                packet[j] = (float)(this->data[j]);
            return ret;
        }
        virtual bool event(std::vector<float> paramArrayOffloat) = 0;
};

class IOnPacketEvent {
    // implementation of IOnPacketEvent class
    bool event(std::vector<unsigned char> paramArrayOfNumber);
};

class IServerImplementation {
    FloatPacketType getPacket() {
        // implementation of getPacket method
    }
};

class AbstractSimpleComsServer {
private:
    std::unordered_map<int, IOnPacketEvent> servers;
    std::unordered_map<IOnPacketEvent, FloatPacketType> packets;
    bool connected;
    std::vector<unsigned char> data;

public:
    void addServer(FloatPacketType packet, IOnPacketEvent iOnBytePacketEvent) {
        this->servers[packet.idOfCommand] = iOnBytePacketEvent;
        this->packets[iOnBytePacketEvent] = packet;
    }

    void addServer(IServerImplementation imp) {
        FloatPacketType packet = imp.getPacket();
        IOnPacketEvent iOnBytePacketEvent = imp;
        addServer(packet, iOnBytePacketEvent);
    }

    bool connect() {
        this->connected = connectDeviceImp();
        std::thread t([this]() {
            while (this->connected) {
                try {
                    int readAmount = this->read(this->getData(), 2);
                    if (readAmount > 0) {
                        int ID = FloatPacketType.getId(this->getData());
                        IOnPacketEvent* event = &this->servers[ID];
                        if (event != nullptr) {
                            FloatPacketType* packet = &this->packets[event];
                            if (packet != nullptr) {
                                std::vector<float> dataValues = packet.parse(this->getData());
                                if (event.event(dataValues)) {
                                    std::vector<unsigned char> backData = packet.command(ID, dataValues);
                                    this->write(backData, this->getPacketSize(), 2);
                                }
                            }
                        }
                    }
                } catch (std::exception& e) {
                    std::cout << e.what() << std::endl;
                }
            }
            this->disconnectDeviceImp();
            std::cout << "Disconnect AbstractSimpleComsServer" << std::endl;
        });
        t.detach();
        return this->connected;
    }

    void disconnect() {
        this->connected = false;
    }

private:
    std::vector<unsigned char> getData() {
        if (this->data.size() == 0)
            std::vector<unsigned char> newdata(getPacketSize());
            this->data = newdata;
        return this->data;
    }

    bool connectDeviceImp() {
        // implementation of connectDeviceImp method
    }

    int read(std::vector<unsigned char> data, int size) {
        // implementation of read method
    }

    void write(std::vector<unsigned char> data, int size, int value) {
        // implementation of write method
    }

    void disconnectDeviceImp() {
        // implementation of disconnectDeviceImp method
    }

    int getPacketSize() {
        // implementation of getPacketSize method
    }
};

