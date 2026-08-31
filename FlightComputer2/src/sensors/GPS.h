#pragma once

struct GPSdata {
    double latitude; // Latitude in decimal degrees
    double longitude; // Longitude in decimal degrees
    float altitude; // Altitude in meters
    uint8_t sats;   // Number of satellites used in the fix
    bool fix; // True if a valid fix is available, false otherwise
    bool valid;   // True if the readings are valid, false otherwise
};