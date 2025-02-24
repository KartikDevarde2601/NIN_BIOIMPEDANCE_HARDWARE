#ifndef ADG706_H
#define ADG706_H

#include <Arduino.h>

class ADG706 {
  public:
    // Constructor to initialize the multiplexer with control pins
    ADG706(int pinS0, int pinS1, int pinS2, int pinS3);

    // Function to initialize the multiplexer
    void begin();

    // Function to select a channel (0-15)
    void selectChannel(int channel);

  private:
    int _S0, _S1, _S2, _S3;  // Control pin numbers
};

#endif
