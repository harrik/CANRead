#ifndef NMEA2000_DEFS_H
#define NMEA2000_DEFS_H


//----------------------------------------------------------------
// ISO Adress Claim Fields
//
//----------------------------------------------------------------

// 1 Bit
#define NMEA2000_ArbitraryAccessCapable 1

// 1: On-Highway Equipment 
// 2: Agricultural and Forestry Equipment 
// 3: Construction Equipment 
// 4: Marine Equipment 
// 5: Industrial, Process Control, Stationary Equipment

// 3Bit
#define NMEA2000_IndustryGroup	4

// 4Bit
#define NMEA2000_SystemInstance	0


// 00	Reserved for 2000 use
// 10	System tools								TBD		TBD
// 20	Safety systems							110		Alarm Enunciator
// 25	Internetwork Device						130		Gateway
//												140		Router
//												150		Bridge
//												160		Repeater
// 30	Power management and lighting systems		130		Switch
//												140		Load
// 40	Steering systems							130		Follow-up Controller
//												140		Mode Controller
//												150		Automatic Steering Controller
//												160		Heading Sensors
// 50	Propulsion systems							130		Engineroom monitoring
//												140		Engine Interface
//												150		Engine Controller	
//												160		Engine Gateway
//												170		Control Head
//												180		Actuator
//												190		Gauge Interface
//												200		Gauge Large
//												210		Gauge Small
// 60	Navigation systems							130		Sounder, depth
//												140
//												145		Global Navigation Satellite System (GNSS)
//												150		Loran C
//												155		Speed Sensors
//												160		Turn Rate Indicator
//												170		Integrated Navigation
//												200		Radar and/or Radar Plotting
//												205		Electronic Chart Display & Information System (ECDIS)
//												210		Electronic Chart System (ECS)
//												220		Direction Finder
// 70	Communications systems					130		Emergency Position Indicating Beacon (EPIRB)
//												140		Automatic Identification System
//												150		Digital Selective Calling (DSC)
//												160		Data Receiver
//												170		Satellite
//												180		Radio-Telephone (MF/HF)
//												190		Radio-Telephone (VHF)
// 80	Instrumentation/general systems				130		Time/Date systems
//												140		Voyage Data Recorder
//												150		Integrated Instrumentation
//												160		General Purpose Displays
//												170		General Sensor Box
//												180		Weather Instruments
//												190		Transducer/general
//												200		NMEA 0183 Converter
// 90	Environmental (HVAC) systems				TBD		TBD
// 100	Deck, cargo, and fishing equipment systems	TBD		TBD

// 7Bit
#define NMEA2000_DeviceClassName		80

// 8Bit
#define NMEA2000_FunctionName			200

// 5Bit
#define NMEA2000_DeviceInstanceUpper	0

// 3Bit
#define NMEA2000_DeviceInstanceLower	0

// 11Bit
// Examples:
// AB Volvo/Volvo Penta			174		Kvasar AB						1859
// Actia Corporation				199		Kohler Power Systems				85
// Actisense						273		LITTON							1858
// Aetna Engineering/Fireboy-Xintex	215		Lowrance Electronics				140
// Airmar						135		Maretron							137
// Beede Electrical					185		Mercury Marine					144
// BEP							295		MMP								1860
// Blue Water Data				148		Moritz Aerospace					176
// Bombardier					163		Mystic Valley Communications		198
// C*Pac Systems AB				165		Nautibus electronic GmbH			147
// Coelmo SRL Italy				286		Navico							275
// Disenos Y Technologia			201		Navionics							1852
// DNA Group, Inc.				211		Northstar Technologies				1854
// EMMI Network					224		Offshore Systems UK				161
// eRide							243		Raymarine, Inc.					1851
// Evinrude/BRP					163		Sea Recovery						285
// Faria Instruments				1863	Ltd.								235
// Floscan Instrument Co., Inc.		192		Sanshin Industries /Yamaha Marine	1862
// Furuno USA					1855	Simrad							1857
// FW Murphy						78		Teleflex							1850
// Garmin						229		Trimble							1856
// Groco							272		Vector Cantech					1861
// Hamilton Jet					283		Westerbeke Corp.					154
// Hemisphere GPS/Satloc Precision	88		Xantrex Technology				168
// Honda Motor					257		Yacht Monitoring Solutions			233

#define NMEA2000_ManufatureCode	285

// A  number uniquely defining this device. MARETRON uses the device serial numer, while GARMIN uses a different number
// 21Bit
#define NMEA2000_UNIQUENUMBER	 1024219




//----------------------------------------------------------------
// Product Information (PGN 126996) Fields
//
//----------------------------------------------------------------

// 16bit
// is set to 1210 at all devices I have seen
#define NMEA2000_DATABASEVERSION	1210

// 16bit
#define NMEA2000_PRODUCTCODE			61258		

// 32 byte (ASCII-String)
#define NMEA2000_MODELID				"NMEA0183-Bridge" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff"

// 32 byte (ASCII-String)
#define NMEA2000_SOFTWAREVERSIONCODE	"0.02b (06/15/2009)" \
											"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff"

// 32 byte (ASCII-String)
#define NMEA2000_MODELVERSION			"0.104 (06/2009)" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff"

// 32 byte (ASCII-String)
#define NMEA2000_MODELSERIALCODE		"00001" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff" \
										"\xff\xff\xff\xff\xff\xff\xff\xff"

// 0 = LevelA, 1=LevelB
#define NMEA2000_CERTIFICATIONLEVEL		0

// The Load Equivalence Number (LEN) lists the power a device draws from the backbone.
// The LEN value represents the integer multiples of 50mA the device draws. A device that draws
// 100mA has a LEN of 2 while a device drawing 101mA has a LEN of 3.

#define NMEA2000_LOADEQUIVALENCE		1




#endif
