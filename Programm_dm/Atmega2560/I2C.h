/*
 * I2C.h
 *Created:2022
 */ 


#ifndef I2C_H_
#define I2C_H_

unsigned char Temperatur_LO [2]; //Low part of the temperatur
unsigned char Temperatur_HI [2]; //High part of the Temperatur
unsigned char PEC; //Checksum, not needed, but you have to read it, for ending the procedure

#define I2C_WRITE 0 //
#define I2C_READ 1 //

void i2c_Wait(void); //Wait until TWINT is set
void i2c_Start(void); //To start the conversation by TWI
void i2c_Restart(void); //To toggle from read to write or vice versa
void i2c_Stop(void); //Stop the conversation
void i2c_Write(unsigned char); //Write data via TWI
void i2c_Address(unsigned char, unsigned char); //Write address of the slave and writr(0), read(1)
void I2C_Temperature(void); //The whole application to read the temperatur
void I2C_transmit_to_LCD(unsigned char, unsigned char); //The whole application to write data to LCD
unsigned char i2c_Read(unsigned char);

void i2c_Wait(void) {
	while (!(TWCR & (1<<TWINT))); //Wait for Interrupt Flag
}

void i2c_Start(void) {
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //Start transmission, no interrupt
	i2c_Wait();
}

void i2c_Restart(void) {
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //Repeated Start Condition, no interrupt
	i2c_Wait();
}

void i2c_Stop(void) {
	TWCR= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); //Send stop condition
	while(!(TWCR & (1<<TWSTO)));  // Wait till stop condition is transmitted
}

void i2c_Write(unsigned char data) {
	//Send 8-bit-data to the slave
	TWDR = data; //Data to the buffer
	TWCR = (1<<TWINT) | (1<<TWEN); //Initialize the transmission, no interrupt
	i2c_Wait();
}

void i2c_Address(unsigned char address, unsigned char mode) {
	//Send address and Read/Write
	unsigned char Address_R_W;
	Address_R_W = (address << 1); //You might shorten this...
	Address_R_W += mode; //...two lines, if you like
	TWDR = Address_R_W; //Data to the buffer
	TWCR = (1<<TWINT) | (1<<TWEN); //Initialize the transmission, no interrupt
	i2c_Wait();
}

unsigned char i2c_Read(unsigned char ack) {
	TWCR = (1<<TWINT) | (1<<TWEN) | (ack<<TWEA); //Initialize the transmission, possibly not the last data to be received, no interrupt
	while (!(TWCR & (1<<TWINT))); //Wait for Interrupt Flag, address has been sent
	return (TWDR);
}

//Get the temperature from sensor
//Needs about 600us
void I2C_Temperature(void) {
	//TWBR = 72; //100 kHz
	//Start:
	i2c_Start(); //Incl. wait for flag
	//while((TWSR & 0xF8)!= 0x08); //Start condition has been transmitted
	//Send slave address and read/write (1/0)
	i2c_Address(0x5A,I2C_WRITE); //Incl. shift the address, init. the transm. and wait for flag
	//while((TWSR & 0xF8)!= 0x18);  //Slaveaddress + W has been send, ACK
	//Send 8-bit-data to the slave
	i2c_Write(0x07); //Incl. wait for Flag
	//while((TWSR & 0xF8) != 0x28);//Data has been transmitted, ACK
	//Send repeated start condition
	i2c_Restart(); //Incl. wait for flag
	//while((TWSR & 0xF8)!= 0x10); //Rep. start condition has been send, ACK
	//Send slave address and read/write (1/0)
	i2c_Address(0x5A,I2C_READ); //Incl. shift the address, init. the transm. and wait for flag
	//while ((TWSR & 0xF8)!= 0x40); //Slaveaddress + R has been send, ACK
	//Read 8-bit-data from the slave
	Temperatur_LO[0] = i2c_Read(1); //0 = last data, 1 = not last data to be read
	//while ((TWSR & 0xF8)!= 0x50); //Data has been received, ACK
	//Read again 8-bit-data from the slave
	Temperatur_HI[0] = i2c_Read(1); //0 = last data, 1 = not last data to be read
	//while ((TWSR & 0xF8)!= 0x50); //Data has been received, ACK
	//Read PEC from the slave
	PEC = i2c_Read(0); //0 = last data, 1 = not last data to be read
	//while ((TWSR & 0xF8)!= 0x58); //Data has been received, NACK terminates the transmission
	//Send stop condition
	i2c_Stop(); //Incl. wait for flag
}

//Transmission via I2C to LCD
//Needs about 300us
//"value" is the date to be displayed
//"tabulator" -> 0 to 3 first line, 16 to 19 second line
void I2C_transmit_to_LCD(unsigned char value, unsigned char tabulator) {
	//TWBR = 72; //100 kHz
	i2c_Start();
	i2c_Address(0x20, I2C_WRITE);
	i2c_Write(tabulator);
	i2c_Write(value);
	i2c_Stop();
}



#endif /* I2C_H_ */