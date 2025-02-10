/*
  Code use to pull all data from the SD Card/Flash in the teensy
*/
 
#include <SD.h>
#include <SPI.h>

File myFile;

//const int chipSelect = BUILTIN_SDCARD; //Pull from the SD Card
const int chipSelect = 2; //Pull from the Flash in the Volta PCB V1

void setup()
{
 //UNCOMMENT THESE TWO LINES FOR TEENSY AUDIO BOARD:

/*   SPI.setMOSI(11);  // Audio shield has MOSI on pin 7
  SPI.setMISO(12);  // Audio shield has MOSI on pin 7
  SPI.setSCK(13);  // Audio shield has SCK on pin 14 */
  
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect.
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  // open the file. 
 
  
  // re-open the file for reading:
  myFile = SD.open("Volta.txt" );
  if (myFile) {
    Serial.println("Volta.txt:");
    
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
    	Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
  	// if the file didn't open, print an error:
    Serial.println("error opening Volta.txt");
  }
}

void loop()
{
	// nothing happens after setup
}


