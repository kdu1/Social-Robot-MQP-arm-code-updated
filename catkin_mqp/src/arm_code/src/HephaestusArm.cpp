#include <vector>
//#include "FloatPacketType.h"
//#include "Robot.h"
#include <unordered_map>
#include "SimpleComsDevice.h"

class HephaestusArm{
private:
    FloatPacketType pollingPacket = FloatPacketType(37, 64);
    FloatPacketType pidPacket = FloatPacketType(65, 64);
    FloatPacketType PDVelPacket = FloatPacketType(48, 64);
    FloatPacketType SetVelocity = FloatPacketType(42, 64);
    std::vector<FloatPacketType> pollingQueue;
    std::unordered_map<int, std::vector<Runnable>> events;

public:
  HephaestusArm(int vidIn, int pidIn) {
    pidPacket.setOneShotMode();
    pidPacket.sendOk();
    PDVelPacket.setOneShotMode();
    PDVelPacket.sendOk();
    SetVelocity.setOneShotMode();
    SetVelocity.sendOk();
    std::vector<FloatPacketType*> packets = {&pollingPacket, &pidPacket, &PDVelPacket, &SetVelocity};
    for (FloatPacketType* pt : packets) {
      addPollingPacket(*pt);
    }
  }

  void addPollingPacket(FloatPacketType packet) {
    printf("addPollingPacket\n");
    printf("packet idOfCommand: %d\n", packet.idOfCommand);
    if (!(getPacket((int)packet.idOfCommand) == nullptr)){
        printf("Only one packet of a given ID is allowed to poll. Add an event to receive data");
        throw("Only one packet of a given ID is allowed to poll. Add an event to receive data"); 
    }
    pollingQueue.push_back(packet);
}

 
  /**
 * given id
 * return FloatPacketType pointer corresponding to it
*/
 FloatPacketType* getPacket(int ID) {
    printf("getPacket\n");
    fflush( stdout );
    printf("ID: %d\n", ID);
    fflush( stdout );

    for (int i = 0; i < pollingQueue.size(); i++) {
        printf("idOfCommand: %d \n", pollingQueue[i].idOfCommand);
        fflush( stdout );
        if (pollingQueue[i].idOfCommand == ID){
            printf("id exists\n");
            fflush(stdout);
            return &pollingQueue[i]; 
        }
    } 
    return nullptr;
}

  void addPollingPacketEvent(Runnable event) {
    addEvent(pollingPacket.idOfCommand, event);
  }

  void addEvent(int id, Runnable event) {
    if (events.find(id) == events.end())
        std::vector<Runnable> nrun();
        events[id] = std::vector<Runnable> ();
    events.at(id).push_back(event);
  }

  void setValuesevent(int index, float position, float velocity, float force) {
    pollingPacket.getDownstream()[index * 3 + 0] = position;
    pollingPacket.getDownstream()[index * 3 + 1] = velocity;
    pollingPacket.getDownstream()[index * 3 + 2] = force;
  }

  void setPIDGains(int index, float kp, float ki, float kd) {
    pidPacket.getDownstream()[index * 3 + 0] = kp;
    pidPacket.getDownstream()[index * 3 + 1] = ki;
    pidPacket.getDownstream()[index * 3 + 2] = kd;
  }

  void pushPIDGains() {
    pidPacket.setOneShotMode();
  }

  void setPDVelGains(int index, float kp, float kd) {
    PDVelPacket.getDownstream()[index * 2 + 0] = kp;
    PDVelPacket.getDownstream()[index * 2 + 1] = kd;
  }

  void pushPDVelGains() {
    PDVelPacket.setOneShotMode();
  }

  void setVelocity(int index, float TPS) {
    SetVelocity.getDownstream()[index] = TPS;
  }

  void pushVelocity() {
    SetVelocity.setOneShotMode();
  }

  std::vector<double> getValues(int index) {
    std::vector<double> back;
    back.push_back(pollingPacket.getUpstream()[index * 3 + 0]);
    back.push_back(pollingPacket.getUpstream()[index * 3 + 1]);
    back.push_back(pollingPacket.getUpstream()[index * 3 + 2]);
    return back;
  }

  double getPosition(int index) {
    return pollingPacket.getUpstream()[index * 3 + 0];
  }

  std::vector<float> getRawValues() {
    return pollingPacket.getUpstream();
  }

  void setRawValues(std::vector<float> set) {
    for (int i = 0; i < set.size() && i < pollingPacket.getDownstream().size(); i++) {
      pollingPacket.getDownstream()[i] = set[i];
    }
  }
};

 // A Functor
/*class Runnable
{   private:
        SimpleComsDevice* scd;
    public:
        Runnable(SimpleComsDevice* s) : scd(s){  }

        // This operator overloading enables calling
        // operator function () on objects of increment
        //how do I get the a
        int operator () (SimpleComsDevice s) const {
            while (s.getConnected()) {
                try {
                    std::vector<FloatPacketType> pollingQueue = s.getPollingQueue();
                    for (int i = 0; static_cast<std::vector<float>::size_type>(i) < pollingQueue.size(); i++) {
                        FloatPacketType pollingPacket = pollingQueue[i];
                        if (pollingPacket.sendOk()){
                            s.process(pollingPacket); 
                        }
                    } 
                } catch (const std::exception& e) {
                    printf("connect thread exception: ");
                    std::cerr << e.what() << std::endl;
                    fflush(stdout);
                } 
                try {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                } catch (const std::exception& e1) {
                    printf("connect thread sleep exception: ");
                    std::cerr << e1.what() << std::endl;
                    fflush(stdout);
                    s.setConnected(false);
                } 
            } 
        
            s.disconnectDeviceImp();
            printf("SimplePacketComs disconnect");
            fflush(stdout);
            return 0;
        };

};*/