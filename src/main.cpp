#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
};  ConfigIP networkAddress;   //3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0}; //This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

uint8_t autoSteerUdpData[UDP_TX_PACKET_MAX_SIZE];  // Buffer For Receiving UDP Data
//Heart beat hello AgIO
uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
uint8_t helloFromAutoSteer[] = { 0x80, 0x81, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
int16_t helloSteerPosition = 0;

unsigned int portMy = 5120;             // port of this module
unsigned int AOGNtripPort = 2233;       // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;   // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;    // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];       // buffer for receiving ntrip data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpPAOGI;     //Out port 5544
EthernetUDP Eth_udpNtrip;     //In port 2233
EthernetUDP Eth_udpAutoSteer; //In & Out Port 8888

IPAddress Eth_ipDestination;
bool Autosteer_running = true; //Auto set off in autosteer setup
bool Ethernet_running = false; //Auto set on in ethernet setup
bool GGA_Available = false;    //Do we have GGA on correct port?
uint32_t PortSwapTime = 0;
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport);

void setup(){
  // start the Ethernet connection:
  Serial.println("Initializing ethernet with static IP address");

  // try to congifure using IP:
  Ethernet.begin(mac,0);          // Start Ethernet with IP 0.0.0.0

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet shield was not found. GPS via USB only.");

    return;
  }

  if (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.println("Ethernet cable is not connected - Who cares we will start ethernet anyway.");
  }

//grab the ip from EEPROM
  Eth_myip[0] = networkAddress.ipOne;
  Eth_myip[1] = networkAddress.ipTwo;
  Eth_myip[2] = networkAddress.ipThree;
    Eth_myip[3] = 120;  //120 is GPS only module

  Ethernet.setLocalIP(Eth_myip);  // Change IP address to IP set by user
  Serial.println("\r\nEthernet status OK");
  Serial.print("IP set Manually: ");
  Serial.println(Ethernet.localIP());

  Ethernet_running = true;

  Eth_ipDestination[0] = Eth_myip[0];
  Eth_ipDestination[1] = Eth_myip[1];
  Eth_ipDestination[2] = Eth_myip[2];
  Eth_ipDestination[3] = 255;

  Serial.print("\r\nEthernet IP of module: "); Serial.println(Ethernet.localIP());
  Serial.print("Ethernet sending to IP: "); Serial.println(Eth_ipDestination);
  Serial.print("All data sending to port: "); Serial.println(portDestination);

  // init UPD Port sending to AOG
  if (Eth_udpPAOGI.begin(portMy))
  {
    Serial.print("Ethernet GPS UDP sending from port: ");
    Serial.println(portMy);
  }
}
void loop(){

    if (!Ethernet_running)
    {
        return;
    }

    uint16_t len = Eth_udpAutoSteer.parsePacket();

    // if (len > 0)
    // {
    //  Serial.print("ReceiveUdp: ");
    //  Serial.println(len);
    // }

    // Check for len > 4, because we check byte 0, 1, 3 and 3
    if (len > 4)
    {
        Eth_udpAutoSteer.read(autoSteerUdpData, UDP_TX_PACKET_MAX_SIZE);

        if (autoSteerUdpData[0] == 0x80 && autoSteerUdpData[1] == 0x81 && autoSteerUdpData[2] == 0x7F) //Data
        {
            if (autoSteerUdpData[3] == 200) // Hello from AgIO
            {
                SendUdp(helloFromAutoSteer, sizeof(helloFromAutoSteer), Eth_ipDestination, portDestination);
                 SendUdp(helloFromIMU, sizeof(helloFromIMU), Eth_ipDestination, portDestination); 
			}

        } //end if 80 81 7F
    }
}

void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
  Eth_udpAutoSteer.beginPacket(dip, dport);
  Eth_udpAutoSteer.write(data, datalen);
  Eth_udpAutoSteer.endPacket();
}

