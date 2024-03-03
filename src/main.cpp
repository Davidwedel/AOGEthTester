#include <NativeEthernet.h>
#include <IPAddress.h>
#include <EEPROM.h>
#include <NativeEthernetUdp.h>

struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 0;
};  ConfigIP networkAddress;   //3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0}; //This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

uint8_t data[UDP_TX_PACKET_MAX_SIZE];   // Buffer For Receiving UDP Data
										   //
byte Eth_ipDest_ending = 255; // ending of IP address to send UDP data to
IPAddress Eth_ipDestination;
uint8_t helloFromMachine[] = {128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71};

unsigned int portMy = 5123;             // port of this module
unsigned int portDestination = 9999;    // Port of AOG that listens
unsigned int portFromAOG = 8888;		// port from aog

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP EthUDPToAOG;
EthernetUDP EthUDPFromAOG;

uint8_t udpData[UDP_TX_PACKET_MAX_SIZE]; // Buffer For Receiving UDP Data
bool EthUdpRunning = false;


const uint8_t LOOP_TIME = 200; // 5hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;
uint32_t fifthTime = 0;
uint16_t count = 0;
int watchdogFails = 0;

uint8_t failTimer = 0;
uint32_t timeofLastFail = 0;
uint32_t timeofFirstFail = 0;

// Comm checks
uint8_t watchdogTimer = 20;   // make sure we are talking to AOG
uint8_t serialResetTimer = 0; // if serial buffer is getting full, empty it
void SendUdp(uint8_t *pgnData, uint8_t datalen, IPAddress dip, uint16_t dport);

void setup(){
	if (Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected - Who cares we will start "
				"ethernet anyway.");
	}
	// start the Ethernet connection
	Serial.println("Initializing ethernet with static IP address");

	// try to congifure using IP:
	Ethernet.begin(mac, 0); // Start Ethernet with IP 0.0.0.0
													// set ips manually
	Eth_myip[0] = networkAddress.ipOne;
	Eth_myip[1] = networkAddress.ipTwo;
	Eth_myip[2] = networkAddress.ipThree;
	Eth_myip[3] = 123;

	Ethernet.setLocalIP(Eth_myip); // Change IP address to IP set by user
	Serial.println("\r\nEthernet status OK");
	Serial.print("IP set Manually: ");
	Serial.println(Ethernet.localIP());

	Eth_ipDestination[0] = Eth_myip[0];
	Eth_ipDestination[1] = Eth_myip[1];
	Eth_ipDestination[2] = Eth_myip[2];
	Eth_ipDestination[3] = 255;

	Serial.print("\r\nEthernet IP of module: ");
	Serial.println(Ethernet.localIP());
	Ethernet.begin(mac, 0);
	if (Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");
	}
	for (byte n = 0; n < 3; n++) {
		Eth_ipDestination[n] = Eth_myip[n];
	}
	Eth_ipDestination[3] = Eth_ipDest_ending;
	Ethernet.setLocalIP(Eth_myip);
	Serial.print("Ethernet IP of Section Control module: ");
	Serial.println(Ethernet.localIP());
	Serial.print("Ethernet sending to IP: ");
	Serial.println(Eth_ipDestination);
	// init UPD Port sending to AOG
	if (EthUDPToAOG.begin(portMy)) {
		Serial.print("Ethernet UDP sending from port: ");
		Serial.println(portMy);
	}
	if (EthUDPFromAOG.begin(portFromAOG)) {
		Serial.print("Ethernet UDP listening to port: ");
		Serial.println(portFromAOG);
		EthUdpRunning = true;
	}
}
void loop(){
	currentTime = millis();
		if (currentTime - lastTime >= LOOP_TIME){
			lastTime = currentTime;
			failTimer++;
			if (watchdogTimer++ > 250)
				watchdogTimer = 20;
			if (watchdogTimer == 19){
				watchdogFails++;
				timeofLastFail = (millis()/1000);
				if (timeofFirstFail == 0)
					timeofFirstFail = (millis()/1000);
			}
			if (failTimer >8){
				failTimer = 0;
				Serial.println();
				Serial.println("-----------------------------------------------------");
				Serial.print("Watchdog => "); Serial.println(watchdogTimer);
				Serial.print("Watchdog Fails => "); Serial.println(watchdogFails);
				Serial.print("First fail => "); Serial.print(timeofFirstFail); Serial.println(" Seconds");
				Serial.print("Last fail => "); Serial.print(timeofLastFail); Serial.println(" Seconds");
				if (!EthUdpRunning)
					Serial.println("Eth not running");
				// Check for Ethernet hardware present
				if (Ethernet.hardwareStatus() == EthernetNoHardware) 
				{
					Serial.println("Ethernet shield was not found. What's going on here???.");
				}
				
			}
		}

	uint16_t len = EthUDPFromAOG.parsePacket();

	//for (int16_t i = 0; i < len; i++) {
	// Serial.print(udpData[i], HEX);
	// Serial.print("\t");
	// }
	if (len > 4) {
		EthUDPFromAOG.read(udpData, UDP_TX_PACKET_MAX_SIZE);
		 //Serial.println(udpData[3]);

		if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) // Data
		{
			

			if (udpData[3] == 239) // machine data
			{
				// reset watchdog
				watchdogTimer = 0;
			}

			else if (udpData[3] == 200) // Hello from AgIO
			{
				if (udpData[7] == 1) {
					watchdogTimer = 0;
				}

				helloFromMachine[5] = 255;
				helloFromMachine[6] = 255;

				SendUdp(helloFromMachine, sizeof(helloFromMachine), Eth_ipDestination, portDestination);
				delay(50);
			}
		}
	}
}

void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{
	EthUDPToAOG.beginPacket(dip, dport);
	EthUDPToAOG.write(data, datalen);
	EthUDPToAOG.endPacket();
}

