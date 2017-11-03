#define SELPIN 10     //Selection Pin 
#define DATAOUT 11    //MOSI 
#define DATAIN 12    //MISO 
#define SPICLOCK 13  //Clock
int read_adc(int channel);

// *************************
// READ ANALOG TO DIGITAL CONVERTER
// *************************
int read_adc(int channel){

  int adcvalue = 0;
  byte commandbits = B11000000; //command bits: start, mode, channel, "dont care".

  //Change commandbits for different channels
  commandbits|=((channel-1)<<3);

	// Turn on
  digitalWrite(SELPIN,LOW);

  // Setup bits to be written
  for (int i=7; i>=3; i--){
    digitalWrite(DATAOUT,commandbits&1<<i);
    //cycle clock
    digitalWrite(SPICLOCK,HIGH);
    digitalWrite(SPICLOCK,LOW);    
  }

	//Ignores 2 null bits
  digitalWrite(SPICLOCK,HIGH);
  digitalWrite(SPICLOCK,LOW);
  digitalWrite(SPICLOCK,HIGH);  
  digitalWrite(SPICLOCK,LOW);

  //Read bits from adc
  for (int i=11; i>=0; i--){
    adcvalue+=digitalRead(DATAIN)<<i;
    //Cycle clock
    digitalWrite(SPICLOCK,HIGH);
    digitalWrite(SPICLOCK,LOW);
  }

	//Turn off device
  digitalWrite(SELPIN, HIGH);
  return adcvalue;
}

