
#include "ADG706.h"

// Constructor
ADG706::ADG706(int pinS0, int pinS1, int pinS2, int pinS3) {
  _S0 = pinS0;
  _S1 = pinS1;
  _S2 = pinS2;
  _S3 = pinS3;
}

// Initialize control pins as OUTPUT
void ADG706::begin() {
  pinMode(_S0, OUTPUT);
  pinMode(_S1, OUTPUT);
  pinMode(_S2, OUTPUT);
  pinMode(_S3, OUTPUT);

  // Set all control pins to LOW (default channel 0)
  digitalWrite(_S0, LOW);
  digitalWrite(_S1, LOW);
  digitalWrite(_S2, LOW);
  digitalWrite(_S3, LOW);
}

// Function to select the channel (0-15)
void ADG706::selectChannel(int channel) {
  // Ensure the channel is within the valid range (0-15)
  channel = channel % 16;

  // Set control pins based on the binary value of the channel
  digitalWrite(_S0, channel & 0x01);  // S0 = channel[0]
  digitalWrite(_S1, (channel >> 1) & 0x01);  // S1 = channel[1]
  digitalWrite(_S2, (channel >> 2) & 0x01);  // S2 = channel[2]
  digitalWrite(_S3, (channel >> 3) & 0x01);  // S3 = channel[3]
}
