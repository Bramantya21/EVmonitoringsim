#define VOLTAGE_SENSOR_PIN A0  // Pin untuk sensor tegangan
#define ACS712_PIN A1          // Pin untuk sensor arus ACS712
#define NUM_SAMPLES 100  // Number of samples for averaging

#define ENA_m1 5        // pin 5
#define ENB_m1 6        // pin 6
#define ENA_m2 10       // pin 10 
#define ENB_m2 11       // pin 11
#define IN_11  2        // driver1 in 1 to pin 2
#define IN_12  3        // driver1 in 1 to pin 3
#define IN_13  4        // driver1 in 1 to pin 4
#define IN_14  7        // driver1 in 1 to pin 7 
#define IN_21  8        // driver2 in to pin 8
#define IN_22  9        // driver2 in to pin 9
#define IN_23  12       // driver2 in to pin 12
#define IN_24  13       // driver2 in to pin 13

int command;              // status perintah 
int speedCar = 255;      // 50 - 255.
int speed_Coeff = 4;

void setup() {  
    Serial.begin(9600); 
    calibrateACS712();

    pinMode(ENA_m1, OUTPUT);
    pinMode(ENB_m1, OUTPUT);
    pinMode(ENA_m2, OUTPUT);
    pinMode(ENB_m2, OUTPUT);
    pinMode(IN_11, OUTPUT);
    pinMode(IN_12, OUTPUT);
    pinMode(IN_13, OUTPUT);
    pinMode(IN_14, OUTPUT);
    pinMode(IN_21, OUTPUT);
    pinMode(IN_22, OUTPUT);
    pinMode(IN_23, OUTPUT);
    pinMode(IN_24, OUTPUT);
} 

void goAhead() { 
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    analogWrite(ENA_m1, speedCar);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    analogWrite(ENB_m1, speedCar);

    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    analogWrite(ENA_m2, speedCar);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    analogWrite(ENB_m2, speedCar);
}

void goBack() { 
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    analogWrite(ENA_m1, speedCar);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    analogWrite(ENB_m1, speedCar);

    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    analogWrite(ENA_m2, speedCar);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    analogWrite(ENB_m2, speedCar);
}

void goRight() { 
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    analogWrite(ENA_m1, speedCar);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    analogWrite(ENB_m1, speedCar);

    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    analogWrite(ENA_m2, speedCar);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    analogWrite(ENB_m2, speedCar); 
}

void goLeft() {
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    analogWrite(ENA_m1, speedCar);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    analogWrite(ENB_m1, speedCar);

    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    analogWrite(ENA_m2, speedCar);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    analogWrite(ENB_m2, speedCar);    
}

void goAheadRight() {
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    analogWrite(ENA_m1, speedCar);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    analogWrite(ENB_m1, speedCar);

    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    analogWrite(ENA_m2, speedCar/speed_Coeff);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    analogWrite(ENB_m2, speedCar/speed_Coeff);
}

void goAheadLeft() {
    digitalWrite(IN_11, HIGH);
    digitalWrite(IN_12, LOW);
    analogWrite(ENA_m1, speedCar/speed_Coeff);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, HIGH);
    analogWrite(ENB_m1, speedCar/speed_Coeff);

    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, HIGH);
    analogWrite(ENA_m2, speedCar);
    digitalWrite(IN_23, HIGH);
    digitalWrite(IN_24, LOW);
    analogWrite(ENB_m2, speedCar);
}

void goBackRight() { 
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    analogWrite(ENA_m1, speedCar/speed_Coeff);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    analogWrite(ENB_m1, speedCar/speed_Coeff);

    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    analogWrite(ENA_m2, speedCar);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    analogWrite(ENB_m2, speedCar);
}

void goBackLeft() { 
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, HIGH);
    analogWrite(ENA_m1, speedCar);
    digitalWrite(IN_13, HIGH);
    digitalWrite(IN_14, LOW);
    analogWrite(ENB_m1, speedCar);

    digitalWrite(IN_21, HIGH);
    digitalWrite(IN_22, LOW);
    analogWrite(ENA_m2, speedCar/speed_Coeff);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, HIGH);
    analogWrite(ENB_m2, speedCar/speed_Coeff);
}

void stopRobot() {  
    digitalWrite(IN_11, LOW);
    digitalWrite(IN_12, LOW);
    analogWrite(ENA_m1, speedCar);
    digitalWrite(IN_13, LOW);
    digitalWrite(IN_14, LOW);
    analogWrite(ENB_m1, speedCar);

    digitalWrite(IN_21, LOW);
    digitalWrite(IN_22, LOW);
    analogWrite(ENA_m2, speedCar);
    digitalWrite(IN_23, LOW);
    digitalWrite(IN_24, LOW);
    analogWrite(ENB_m2, speedCar);
}

// Read voltage from the external voltage sensor
float readVoltageSensor() {
    int sensorValue = analogRead(VOLTAGE_SENSOR_PIN);
    float voltage = sensorValue * (5.0 / 1023.0) * 5.0;  // Adjusted for voltage divider 1:11
    return voltage;
}

// Define a global variable to store the offset (calibrated 0 current voltage)
float zeroCurrentOffset = 2.5;  // Default is 2.5V but will calibrate dynamically

void calibrateACS712() {
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum += analogRead(ACS712_PIN);
        delay(5);
    }
    zeroCurrentOffset = (sum / NUM_SAMPLES) * (5.0 / 1023.0);  // Average baseline voltage
}

// Read current from ACS712 after calibration
float readCurrentACS712() {
    int sensorValue = analogRead(ACS712_PIN);
    float voltage = sensorValue * (5.0 / 1023.0);         // Sensor output voltage (0-5V)
    float current = (voltage - zeroCurrentOffset) / 0.185;  // ACS712-05B sensitivity = 185mV/A

    // Allow small negative currents if necessary, but clamp very small values to 0
    if (current < 0 && current > -0.05) {
        current = 0.0;  // Ignore small noise below 50mA
    }
    
    return current;
}

// Read battery values: voltage, current, and SoC
void readBatteryValues(float &voltage, float &current, float &soc) {
    voltage = readVoltageSensor();       // Read voltage from external sensor
    current = readCurrentACS712();       // Read current from ACS712 sensor
    soc = (voltage / 12.6) * 100;        // Calculate SoC based on 3S 12.6V max voltage
    soc = constrain(soc, 0, 100);        // Ensure SoC is within valid range (0-100%)
}


void loop() {
    if (Serial.available() > 0) {
        command = Serial.read();
        stopRobot(); // Initialize with motors stopped.

        float battery_voltage, battery_current, soc; // Variabel untuk menyimpan nilai

        // Membaca nilai baterai
        readBatteryValues(battery_voltage, battery_current, soc);

        switch (command) {
            case 'F': goAhead(); break;
            case 'B': goBack(); break;
            case 'L': goLeft(); break;
            case 'R': goRight(); break;
            case 'I': goAheadRight(); break;
            case 'G': goAheadLeft(); break;
            case 'J': goBackRight(); break;
            case 'H': goBackLeft(); break;
            case 'V': { // Kirim hanya tegangan ke aplikasi
                char buffer[10]; // Declare buffer for formatted output
                dtostrf(battery_voltage, 4, 1, buffer);   // Format voltase 1 angka desimal
                Serial.println(buffer);
                break;
            }
            case 'C': { // Kirim hanya arus ke aplikasi
                char buffer[10]; // Declare buffer for formatted output
                dtostrf(battery_current, 4, 1, buffer);   // Format arus 1 angka desimal
                Serial.println(buffer);
                break;
            }
            case 'S': { // Kirim hanya SOC ke aplikasi
                char buffer[10]; // Declare buffer for formatted output
                dtostrf(soc, 4, 1, buffer);               // Format SOC 1 angka desimal
                Serial.println(buffer);
                break;
        }

        // Clear the serial buffer after sending the data
        while (Serial.available() > 0) {
            Serial.read();
        }
    }
}
}