#pragma once

struct AccelerometerData {
    float x_g;          //in g (±200g range)
    float y_g;
    float z_g;
    float magnitude_g;  // sqrt(x²+y²+z²)
    bool valid;   // True if the readings are valid, false otherwise
};