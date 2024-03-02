//#include <vector>
//#include <unordered_map>
//#include "PacketType.h"
//#include <ros/ros.h>
#include "FloatPacketType.h"

#include <iostream>
#include <cassert>
#include <hidapi/hidapi.h>
//#include <libusb.h>
#include "libusb-1.0/libusb.h"
//#include <hidapi/hidapi_libusb.h>
//#include "hidapi.h"

#include <memory>
#include <thread>
#include <complex>
#include <valarray>
#include <unistd.h>
#include <cstdlib>

#define _USE_MATH_DEFINES
# define M_PI           3.14159265358979323846  /* pi */


// Headers needed for sleeping.
#ifdef _WIN32
	#include <windows.h>
#else
	#include <unistd.h>
#endif

// Fallback/example
#ifndef HID_API_MAKE_VERSION
#define HID_API_MAKE_VERSION(mj, mn, p) (((mj) << 24) | ((mn) << 8) | (p))
#endif
#ifndef HID_API_VERSION
#define HID_API_VERSION HID_API_MAKE_VERSION(HID_API_VERSION_MAJOR, HID_API_VERSION_MINOR, HID_API_VERSION_PATCH)
#endif

//
// Sample using platform-specific headers

#if defined(USING_HIDAPI_LIBUSB) && HID_API_VERSION >= HID_API_MAKE_VERSION(0, 12, 0)
#include <hidapi_libusb.h>
#endif


typedef std::complex<float> Complex;
typedef std::valarray<Complex> CArray;




class SimpleComsDevice {
    
    private:
        std::vector<FloatPacketType> pollingQueue;
        //unsigned short vid = 0x16c0;
        //unsigned short pid = 0x0486;
        //wchar_t serial[1024];
        //std::wcsncpy(serial, L"6E5B9DF04652375320202051413808FF", 1024); //TODO: error with this
        //serial not necessarily needed to open device

        //Alternatively:open by path
        const char* path; //= "/dev/hidraw1";
        hid_device * handle; //= hid_open_path(path);

    public:
        bool connected;
        bool virtualv = false;
        int readTimeout = 1000;
        bool isTimedOut = false;


    /**
    * Constructor
    */
    SimpleComsDevice();

    

    //device:
    //from test.c
    // Open the device using the VID, PID,
    // and optionally the Serial number.
    ////handle = hid_open(0x4d8, 0x3f, L"12345");
    /*unsigned short vid = 0x16c0;
    unsigned short pid = 0x0486;
    hid_device * handle = hid_open(vid, pid, NULL); //from hid.c*/

    bool validHandle(hid_device * handle);
    //std::unordered_map<int, std::vector<Runnable>> events;
    
    
    
    /**
    * add pollingPacket to pollingQueue
    * @param packet FloatPacketType packet to be process()ed
    */
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
     * get pollingQueue
     * @return pollingQueue
    */
     std::vector<FloatPacketType> getPollingQueue(){
        //ROS_INFO("pollingqueue");
        return this->pollingQueue;
    }

    /**
     * get hid_device* handle
     * @return handle
    */
    hid_device * getHandle(){
        //ROS_INFO("gethandle");
        return handle;
    }

    /**
     * set hid_device* handle
     * @param handle hid_device * handle to be set to
    */
    void setHandle(hid_device * handle){
        //ROS_INFO("sethandle");
        printf("sethandle\n");
        this->handle = handle;
    }


    /**
     * setPollingQueue
     * @param pollingQueue
    */
     void setPollingQueue(std::vector<FloatPacketType> pollingQueuein){
        this->pollingQueue = pollingQueuein;
    }

    
    /**
     * given id
     * return FloatPacketType pointer corresponding to it
     * @param id of certain packet
     * @return FloatPacketType* retrieved at id
    */
     FloatPacketType* getPacket(int ID) ;
    
    /*
    public void removeEvent(int id, Runnable event) {
        if (this.events.get(id) == null)
        this.events.put(id, new ArrayList<>()); 
        ((ArrayList)this.events.get(id)).remove(event);
    }
    
    public void addEvent(int id, Runnable event) {
        if (this.events.get(id) == null)
        this.events.put(id, new ArrayList<>()); 
        ((ArrayList<Runnable>)this.events.get(id)).add(event);
    }
    
   */
    
   
     void writeFloats(int id, std::vector<Complex> values) ;
    
     void writeFloats(int id, std::vector<Complex> values, bool polling) ;
    
    
     std::vector<float> readFloats(int id) ;

    /**
     * Gets the id from byte vector
     * @param bytes byte vector
    */
     float getId(std::vector<unsigned char> bytes);
    
    /**
     * actually calls write
    */
     void process(FloatPacketType packet);
    
     int getReadTimeout();
    /**
     * calls process which calls write
    */
     bool connect();
    
     void disconnect();
    
     bool isVirtual();
    
     void setVirtual(bool virtualv);
    
     void setReadTimeout(int readTimeout);
    
    /*String getName() {
        return this->name;
    }
    
    void setName(String name) {
        this.name = name;
    }*/
    
     bool getIsTimedOut();
    
     bool disconnectDeviceImp();
    
     bool connectDeviceImp();

    /**
     * setConnected
     * @param connected value 
    */
     void setConnected(bool set);
    /**
     * getConnected
     * @return connected value 
    */
     bool getConnected();

    /**
     * write using hidapi
    */
     int write(std::vector<unsigned char> packet, int len, unsigned char reportID);
    /**
     * TODO: not sure how to get resulting packets returned
     * read using hidapi
     * reads position data from each motor
    */
     int read(std::vector<unsigned char> bytes, int milliseconds);


     /**
     * print hid_device_info
     * @param cur_dev_ hid_device_info* struct
    */
    void print_device(struct hid_device_info *cur_dev);

    /**
     * print hid_device_info
     * @param cur_dev_ hid_device_info* linkedlist
    */
    void print_devices_with_descriptor(struct hid_device_info *cur_dev); 

    /**
     * print hid_report_descriptor given path
     * @param path const char* path to open device
    */
    void print_hid_report_descriptor_from_path(const char *path);

    /**
     *  print hid_report_descriptor given hid_device
     * @param device hid_device* device
    */
    void print_hid_report_descriptor_from_device(hid_device *device);

};

/**
* implementation of Java Runnable
*/
 // A Functor
class Runnable
{   private:
        SimpleComsDevice* scd;
    public:
        Runnable(SimpleComsDevice* s) : scd(s){  }

        // This operator overloading enables calling
        // operator function () on objects of increment
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

};
