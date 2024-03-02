

#include <vector>
#include <unordered_map>
#include "SimpleComsDevice.h"

#include <iostream>
#include <memory>
#include <cassert>
#include <thread>
#include <complex>
#include <valarray>

#define _USE_MATH_DEFINES
# define M_PI           3.14159265358979323846  /* pi */
   

/**
 * Constructor
*/
SimpleComsDevice::SimpleComsDevice(){
    printf("SimpleComsDevice Constructor\n");
    fflush( stdout );
    //const wchar_t* serial = L"F09D5B6E5337524651202020FF083841";
    //hid_device* handle = hid_open_path(path); //open by path
    hid_device* handle = hid_open(0x16c0, 0x0486, NULL);//open by vid/pid
    //prints whether open was successful or not
    printf("hid_error: %ls\n", hid_error(handle));
    fflush( stdout );
    SimpleComsDevice::setHandle(handle);

    //print device info using enumerate for debugging
   /* hid_device_info* devinfo = hid_enumerate(0x16C0, 0x0486);
    print_devices_with_descriptor(devinfo);
    hid_free_enumeration(devinfo);*/
}



//HIDPacketComs extends this, which calls read + write, which calls hidDevice.read+write, 
//which calls HidApi.read + write
//which calls hid_api.read + write
/**
* Determines whether a given hid_device handle is valid
* @param handle hid_device handle
*/
bool SimpleComsDevice::validHandle(hid_device * handle){
    //printf("in validHandle");
    if (!handle) {
        printf("invalid handle: unable to open device\n");
        fflush( stdout );
        hid_exit();
        return 0;
    }
    return 1;
}

/**
 * print hid_device_info
 * @param cur_dev_ hid_device_info* struct
*/

void SimpleComsDevice::print_device(struct hid_device_info *cur_dev) {
	printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
    fflush( stdout );
	printf("\n");
    fflush( stdout );
	printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
    fflush( stdout );
	printf("  Product:      %ls\n", cur_dev->product_string);
    fflush( stdout );
	printf("  Release:      %hx\n", cur_dev->release_number);
    fflush( stdout );
	printf("  Interface:    %d\n",  cur_dev->interface_number);
    fflush( stdout );
	printf("  Usage (page): 0x%hx (0x%hx)\n", cur_dev->usage, cur_dev->usage_page);
    fflush( stdout );
	//printf("  Bus type: %u (%s)\n", (unsigned)cur_dev->bus_type, hid_bus_name(cur_dev->bus_type));
	printf("\n");
    fflush( stdout );
}

/**
 * print hid_device_info
 * @param cur_dev_ hid_device_info* linkedlist
*/
void SimpleComsDevice::print_devices_with_descriptor(struct hid_device_info *cur_dev) {
	for (; cur_dev; cur_dev = cur_dev->next) {
        //print_hid_report_descriptor_from_device(SimpleComsDevice::handle);
		print_device(cur_dev);
		//print_hid_report_descriptor_from_path(cur_dev->path);
	}
}

/**
 * print hid_report_descriptor given path
 * @param path const char* path to open device
*/
/*void SimpleComsDevice::print_hid_report_descriptor_from_path(const char *path) {
	hid_device *device = hid_open_path(path);
	if (device) {
		print_hid_report_descriptor_from_device(device);
		hid_close(device);
	}
	else {
		printf("  Report Descriptor: Unable to open device by path\n");
	}
}*/

/**
 *  print hid_report_descriptor given hid_device
 * @param device hid_device* device
*/
/*void SimpleComsDevice::print_hid_report_descriptor_from_device(hid_device *device) {
	unsigned char descriptor[4096];
	int res = 0;

	printf("  Report Descriptor: ");
	res = hid_get_report_descriptor(device, descriptor, sizeof(descriptor));
	if (res < 0) {
		printf("error getting: %ls", hid_error(device));
	}
	else {
		printf("(%d bytes)", res);
	}
	for (int i = 0; i < res; i++) {
		if (i % 10 == 0) {
			printf("\n");
		}
		printf("0x%02x, ", descriptor[i]);
	}
	printf("\n");
}*/

//std::unordered_map<int, std::vector<Runnable>> events;
    
/**
 * given id
 * return FloatPacketType pointer corresponding to it
*/
 FloatPacketType* SimpleComsDevice::getPacket(int ID) {
    printf("getPacket\n");
    fflush( stdout );
    printf("ID: %d\n", ID);
    fflush( stdout );

    //std::vector<FloatPacketType> pq = SimpleComsDevice::getPollingQueue();
    //FloatPacketType q(0, 1);

    //Need to return a new FloatPacketType pointer object
    for (int i = 0; i < SimpleComsDevice::getPollingQueue().size(); i++) {
        //FloatPacketType q = SimpleComsDevice::getPollingQueue()[i];
        printf("idOfCommand: %d \n", SimpleComsDevice::getPollingQueue()[i].idOfCommand);
        fflush( stdout );
        if (SimpleComsDevice::getPollingQueue()[i].idOfCommand == ID){
            printf("id exists\n");
            fflush(stdout);
            return &SimpleComsDevice::getPollingQueue()[i]; 
        }
    } 
    return nullptr;
}

/**
 * writes target endpoint info from servo_jp and adds it to the pollingPacket to be polled and read by Process()
 * Sets polling to true and has it be called by other writeFloats method
 * @param id, values reportID and values to be written
 */
void SimpleComsDevice::writeFloats(int id, std::vector<Complex> values) {
    //double checking stuff inside values for debugging
    printf("writeFloats\n");
    fflush( stdout );
    for(int i = 0; i < values.size(); i++){
        printf("values[%d]: %f\n", i, values[i].real());
    }
    writeFloats(id, values, true);
}
   
/**
 * writes target endpoint info from servo_jp and adds it to the pollingPacket to be polled and read by Process()
 * Actual code for writeFloats with polling
 * @param id, values reportID and values to be written
 */    
void SimpleComsDevice::writeFloats(int id, std::vector<Complex> values, bool polling) {
    //TODO: incorrect conversion
    //float* floatvalues = reinterpret_cast<float(&)[2]>(values);//cast from complex to float 

    //TODO: only taking the real component of the complex values because I don't need them for now. May cause issues with pickAndPlace
    float floatvalues[values.size()] = {};
    for(int i = 0; i < values.size(); i++){
        floatvalues[i] = values[i].real();
    }

    //another debugging check
    printf("floatvalues:\n");
    fflush( stdout );
    for(int i = 0; i < values.size(); i++){
        printf("floatvalues[%d]: %f\n", i,  floatvalues[i]);
    }
    if (getPacket(id) == nullptr) {
        FloatPacketType pt = FloatPacketType(id, 64);
        //if (!polling)//not going to be true
            //pt.oneShotMode(); 
        std::vector<float> newdownstream;
        for (int i = 0; static_cast<std::vector<float>::size_type>(i) < (pt.getDownstream()).size() && static_cast<std::vector<float>::size_type>(i) < values.size(); i++){
            newdownstream.push_back(floatvalues[i]); 
            
            //TODO: is this correct?
        }
        pt.setDownstream(newdownstream);
        SimpleComsDevice::addPollingPacket(pt); //size added here will be size of FloatPacketType called in process()
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        } catch (const std::exception& e) {
            printf("writeFloats thread sleep exception: ");
            fflush( stdout );
            printf(e.what());
            fflush( stdout );
        } 
    } else {
        std::vector<FloatPacketType, std::allocator<FloatPacketType>> pollingQueue = SimpleComsDevice::getPollingQueue();
        for (int j = 0; static_cast<std::vector<FloatPacketType>::size_type>(j) < pollingQueue.size(); j++) {
            FloatPacketType pt = pollingQueue[j];
            std::vector<float> newdownstream; //new vector for downstream
            if (pt.idOfCommand == id) {
                for (int i = 0; static_cast<std::vector<float>::size_type>(i) < pt.getDownstream().size() && static_cast<std::vector<float>::size_type>(i) < values.size(); i++){
                    //check if in floatvalues bounds
                    if(i < sizeof(floatvalues)/sizeof(float)){
                        newdownstream.push_back(floatvalues[i]); 
                    }
                    //otherwise just put 0
                    else{
                        newdownstream.push_back(0);
                    }
                }
                pt.setDownstream(newdownstream);
                /*if (!polling){
                    pt.oneShotMode(); 
                }*/
                return;
            } 
        }; 
    } 
}


/**
 * reads data given id
 * @param id reportid
 * @return vector of floats containing read information
 * gets the information given the id from the pollingPacket, then returns it
*/
 std::vector<float> SimpleComsDevice::readFloats(int id) {
    printf("readFloats\n");
    fflush( stdout );
    if (getPacket(id) == nullptr) {//packets are assigned an id related to the purpose, SimplePacketComs manages them
        printf("id null\n");
        fflush(stdout);
        FloatPacketType fl = FloatPacketType(id, 64);
        SimpleComsDevice::addPollingPacket(fl);
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        } catch (const std::exception& e) {
            printf("readFloats thread sleep exception: ");
            fflush( stdout );
            std::cout << e.what() << std::endl;
            fflush( stdout );
        } 
    } 

   
    printf("before getPacket id: %d\n", id);
    fflush( stdout );
    FloatPacketType* pt = SimpleComsDevice::getPacket(id);
    
    //should not reach this case
    if(pt == nullptr){
        printf("null");
        fflush( stdout );
        throw std::runtime_error("FloatPacketType null");
    }
    std::vector<float> values((pt->getUpstream()).size(), 0);
    for (int i = 0; i < (pt->getUpstream()).size() && i < values.size(); i++){
        values[i] = (float) pt->getUpstream()[i]; 
    }
    return values;
}


/**
 * gets first 4 bytes as reportID from message
 * @return int reportId
*/
 float SimpleComsDevice::getId(std::vector<unsigned char> bytes) {

    unsigned char data[4];


    //get 1st 4 bytes
    for(int i = 0; i < 4; i++){
        data[i] = bytes[i];
    }
    
    float id;

    //https://stackoverflow.com/questions/21005845/how-to-get-float-in-bytes

    //convert to float because original reportID was put into float array and converted to byte along with the rest of the array
    memcpy(&id, &data, sizeof(id));     
    printf("getId, %f\n", id);
    fflush(stdout);
    return id;
}

/**
 * Writes a given FloatPacketType and then reads from it accordingly
 * The function where actually calls write and does the process of using the data to move the arm
 * @param packet a packet that is being processed
*/
 void SimpleComsDevice::process(FloatPacketType packet){
    packet.started = true;
    try {
        if (!isVirtual()) {
            try {
                //convert downstream from float to unsigned char
                std::vector<unsigned char> message = packet.command(packet.idOfCommand, packet.getDownstream()); //this converts floats to unsigned char format for writing

                printf("message:\n");
                for(int the = 0; the < (int)message.size(); the++){
                    printf("message[%d]: %u\n", the, message[the]);
                    fflush(stdout);
                }
                int val = write(message, message.size(), 1); //call to function that will call hid_write
                sleep(1);
                printf("result from write: %d\n", val);
                fflush( stdout );
                if (val > 0) {
                    //gets result after write
                    int readint = read(message, SimpleComsDevice::getReadTimeout()); //calls hid_read
                    sleep(1);
                    printf("read result: %d \n", readint);
                    fflush( stdout );
                    //if read result greater than or equal to packet upstream size, do some timeout resolving packet managing
                    if (static_cast<std::vector<float>::size_type>(readint) >= (packet.getUpstream()).size()) {
                        printf("greater than upstream size\n");
                        fflush(stdout);
                        float ID = SimpleComsDevice::getId(message); // if current packet being processed matches current input id 
                        printf("ID : %f\n", ID);
                        fflush(stdout);
                        printf("%d\n", packet.idOfCommand);
                        fflush(stdout);
                        //TODO: why is this needed?
                        if ((int)ID == packet.idOfCommand) {
                            printf("ids equal\n");
                            fflush(stdout);
                            if (SimpleComsDevice::isTimedOut){
                                printf("Timeout resolved %d\n", ID); 
                                fflush( stdout );
                            }
                            SimpleComsDevice::isTimedOut = false;
                            std::vector<float> up = packet.parse(message);//change from float array back
                            for (int i = 0; static_cast<std::vector<float>::size_type>(i) < (packet.getUpstream()).size(); i++){
                                packet.getUpstream()[i] = up[i];
                            }
                        } else {
                            for (int i = 0; i < 3; i++){
                                read(message, SimpleComsDevice::getReadTimeout()); 
                                sleep(1);
                            }
                            printf(" ");
                            fflush( stdout );
                            SimpleComsDevice::isTimedOut = true;
                            return;
                        } 
                    } else {
                        SimpleComsDevice::isTimedOut = true;
                        return;
                    } 
                } else {
                    printf("write return value not 1\n");
                    fflush( stdout );
                    return;
                } 
            } catch (const std::exception& e) {
                //t.printStackTrace(System.out);
                printf("exception: in process under not isvirtual ");
                printf(e.what());
                fflush( stdout );
                disconnect();
            } 
    } 
    else {
        std::vector<float> newupstream;
        std::vector<float> currentDownstream = packet.getDownstream(); 
        for (int j = 0; static_cast<std::vector<float>::size_type>(j) < (packet.getDownstream()).size() && static_cast<std::vector<float>::size_type>(j) < (packet.getUpstream()).size(); j++){
            newupstream[j] = currentDownstream[j];
        }
        packet.setUpstream(newupstream);
    } 


    //currently unusued:
    //if there are packets that were added to events, run each one
    /*if (events.find(packet.idOfCommand) == events.end()){
        printf("going through events\n");
        fflush(stdout);
        //Runs runnable
        //if (events.find(packet.idOfCommand) != events.end()) {
        for (Runnable e : events[packet.idOfCommand]) {
            printf("runnable\n");
            fflush(stdout);
            if (&e != nullptr) {
                try {
                    e(*this);
                } catch (const std::exception& e) {
                    std::cout << e.what() << std::endl;
                    fflush( stdout );
                }
            }
        }
        //}

        /*
        for (SimpleComsDevice::Runnable e : this->events.at((int)packet.idOfCommand)) {
            if (&e != nullptr){
                try {
                    e(*this);
                } catch (const std::exception& e) {
                    printf("something went wrong inside process");
                    printf("exception: ");
                    printf(e.what());
                }  
            }
        }  */
    /*}*/

    } catch (const std::exception& e) {
        printf("exception: ");
        printf(e.what());
        fflush( stdout );
    }
    printf("end of process\n");
    fflush( stdout );
    packet.done = true;
}

 int SimpleComsDevice::getReadTimeout() {
    return SimpleComsDevice::readTimeout;
}

/**
 * Connects to device and begins to process packets and write to it
 * calls process which calls write
*/
 bool SimpleComsDevice::connect() {
    if (connectDeviceImp()) {
        setVirtual(false);
    } else {
        setVirtual(true);
    } 
 
    SimpleComsDevice::connected = true;
    std::thread([&]() {
        //while still connected to device, process everything in the pollingpacket which manages all the data being written and read
        while (SimpleComsDevice::connected) {
            try {
                std::vector<FloatPacketType, std::allocator<FloatPacketType>> pollingQueue = SimpleComsDevice::getPollingQueue();
                //process all packets in pollingQueue
                for (int i = 0; static_cast<std::vector<FloatPacketType>::size_type>(i) < pollingQueue.size(); i++) {
                    printf("polling i: %d\n", i);
                    fflush( stdout );
                    FloatPacketType pollingPacket = pollingQueue[i];
                    fflush( stdout );
                    if (pollingPacket.sendOk() && pollingPacket.done == false){
                        process(pollingPacket);
                    }
                    //mark polling packet done
                    pollingQueue[i].done = true;
                    SimpleComsDevice::setPollingQueue(pollingQueue);
                }
            } catch (std::exception& e) {
                std::cout << e.what() << std::endl;
                //printf(e.what());
                fflush( stdout );
            }
            try {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            } catch (std::exception& e1) {
                std::cout << e1.what() << std::endl;
                //printf(e1.what());
                fflush( stdout );
                SimpleComsDevice::connected = false;
            }
        }

        this->disconnectDeviceImp();
        printf("SimplePacketComs disconnect\n");
        fflush( stdout );
    }).detach();

    std::cout << "end of connect" << std::endl;
    fflush( stdout );
    return true;
}

/**
 * Disconnects from hid and indicates that device is no longer connected
*/
 void SimpleComsDevice::disconnect() {
    hid_close(SimpleComsDevice::getHandle());
    hid_exit();
    SimpleComsDevice::connected = false;
}

/**
* getter for virtualv
* @return bool whether virtualv is true or false
*/
 bool SimpleComsDevice::isVirtual() {
    return SimpleComsDevice::virtualv;
}

/**
* Setter for virtualv
*/
 void SimpleComsDevice::setVirtual(bool virtualv) {
    SimpleComsDevice::virtualv = virtualv;
}

/**
* Setter for readTimeout
*/
 void SimpleComsDevice::setReadTimeout(int readTimeout) {
    SimpleComsDevice::readTimeout = readTimeout;
}

/*String getName() {
    return this->name;
}

void setName(String name) {
    this.name = name;
}*/

 bool SimpleComsDevice::getIsTimedOut() {
    return SimpleComsDevice::isTimedOut;
}

//disconnection
 bool SimpleComsDevice::disconnectDeviceImp(){
    SimpleComsDevice::connected = false;
    printf("closing device...\n");
    fflush( stdout );
    hid_close(SimpleComsDevice::getHandle());
    printf("device closed\n");
    fflush( stdout );
    hid_exit();
    return true;
}

/**
 * TODO: why isn't this implemented
*/
bool SimpleComsDevice::connectDeviceImp(){
    return true;
}


/**
 * setConnected
 * @param connected value 
*/
void SimpleComsDevice::setConnected(bool set){
    SimpleComsDevice::connected = set;
}

/**
 * getConnected
 * @return connected value 
*/
bool SimpleComsDevice::getConnected(){
    return SimpleComsDevice::connected;
}


/**
 * write using hidapi
 * @param packet, len, reportID 
 * the packet to write, the length of the packet, and the reportID of the packet
*/
int SimpleComsDevice::write(std::vector<unsigned char> packet, int len, unsigned char reportID){
    
    try{
        printf("write\n");
        fflush( stdout );

        /*for(int i =0; i < packet.size(); i++){
            printf("message %d:, %u\n", i, packet[i]);

        }*/
           
       

        if(!validHandle(SimpleComsDevice::getHandle())){
            //printf("unable to open device\n");
            printf("invalid handle\n");
            fflush( stdout );
            hid_exit();
            throw std::runtime_error("invalid handle");
            return -1;
        }
        
        //first 4 bytes (int or float) is reportid being 0 cast to a byte

        // Precondition checks
        if (packet.size() < static_cast<std::vector<long unsigned int>::size_type>(len)) {
            len = packet.size();
        }

        unsigned char bytesbuffer[packet.size()+1];

        bool dropReportIdZero = false;
        if (dropReportIdZero == 0 && reportID == 0) { //TODO: not 100% on the data type conversion on reportID here, but this is for windows so it doesn't matter for this case
            // Use the alternative buffer representation that does
            // not include report ID 0x00
            // This overcomes "The parameter is incorrect" errors on
            // Windows 8 and 10
            // See the commentary on the dropReportIdZero flag for more info
           
            if(len >= 1){
            	for(int i = 0; i < len; i++){
                	bytesbuffer[i] = packet[i];
            	}
            }
        } else {
            // Put report ID into position 0 and fill out buffer
            if (len >= 1) {
                //copy packet into bytesbuffer at index 1
                for(int i = 0; i < packet.size(); i++){
                    bytesbuffer[i] = packet[i];
                }
                
            }
        }
        //TODO: can potentially replace this with a ROS node, and have it send information through the node.
        //flashing ROS code onto the arm board with micropython in order to have the ros node on the board, or can use arduino
        //or just wire it to the raspberry pi which is probably easier but will have to rewrite firmware

        printf("before hid_write\n");
        fflush( stdout );
        //another test here just in case
        if(!validHandle(SimpleComsDevice::getHandle())){
            throw std::runtime_error("device handle invalid");
            return -1;
        }
        //TODO: idk if it will change the data if writing to hid_write with a float array twice as long (to account for complex values)
        int res = hid_write(SimpleComsDevice::getHandle(), bytesbuffer, len); //hid_device *dev, const unsigned char *data, size_t length
        printf("%d\n", res);
        fflush( stdout );
        printf("after hid_write\n");
        fflush( stdout );
        if(res < 0){
            printf("Unable to write()\n: %ls\n", hid_error(SimpleComsDevice::getHandle()));
            fflush( stdout );
            return -1;
        }
        return res;
    } 
    catch (const std::exception& e) {
        printf("Command error, reading too fast\n");
        fflush( stdout );
        std::cout << e.what() << std::endl;
        fflush( stdout );
        return -1;
    }
}


/**
 * read using hidapi
 * reads position data from each motor
 * @param bytes vector of bytes to be read
 * @param milliseconds time allotted for hid_read_timeout
 * @return res int result of hid_read_timeout
*/
int SimpleComsDevice::read(std::vector<unsigned char> bytes, int milliseconds){
    printf("read\n");
    fflush( stdout );
    //reportID is what goes in the first position of the buffer
    if(!validHandle(SimpleComsDevice::getHandle())){
        hid_exit();
        throw std::runtime_error("unable to open device");
    }


    //it wants unsigned char * instead of vector
    unsigned char* bytesbuffer = &bytes[0];
    printf("before hid_read_timeout:\n");
    fflush( stdout );
    int res = hid_read_timeout(SimpleComsDevice::getHandle(), bytesbuffer, bytes.size(), milliseconds); //hid_device *dev, unsigned char *data, size_t length, int milliseconds

     if(res < 0){
        printf("Unable to read()/2: %ls\n", hid_error(SimpleComsDevice::getHandle()));
        throw std::runtime_error("unable to read");
    }
    printf("after hid_read_timeout\n");
    fflush( stdout );
    printf("hid_read_timeout: %d\n", res);
    fflush( stdout );

    return res;
}

