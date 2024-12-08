const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10;
const int encoderPinA = 2;
const int encoderPinB = 3;

volatile int direction = 0;
volatile int pulseCount = 0;
unsigned long lastTime = 0;
float rpmValue = 0;

const int PPR = 11;
const float GearRatio = 9.6;

float kp = 0.18;
float ki = 0.1;
float kd = 0.01;

float setpoint = 0;
float e = 0, e_prev = 0, inte = 0, inte_prev = 0;
int pidOutput = 0;
bool isRunning = false;

void setup() {
    Serial.begin(9600);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), countEncoder, RISING);
}

void loop() {
    unsigned long currentTime = millis();
    int currentCount = pulseCount;

    if (currentTime - lastTime >= 100) {
        readSensor();
        lastTime = currentTime;

        if (isRunning) {
            pidOutput = PIDControl();
            analogWrite(ENA, pidOutput);
        }

        Serial.print("RPM:");
        Serial.println(int(rpmValue));
        Serial.print("Dir:");
        Serial.println(direction);
    }

    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        handleCommand(command);
    }
}

void countEncoder() {
    pulseCount++;
    int stateB = digitalRead(encoderPinB);
    direction = (stateB == HIGH) ? 1 : -1;
}

void readSensor() {
    rpmValue = (pulseCount / (PPR * GearRatio)) * 600;
    pulseCount = 0;
}

int PIDControl() {
    e = setpoint - rpmValue;
    inte = inte_prev + e;
    float derivative = e - e_prev;
    pidOutput = constrain(kp * e + ki * inte + kd * derivative, 0, 255);
    e_prev = e;
    inte_prev = inte;
    return pidOutput;
}

float storedKp = 0.18;
float storedKi = 0.1;
float storedKd = 0.01;
int storedSetpoint = 0;
bool storedDirection = true;

void handleCommand(String command) {
    int delimiterIndex = command.indexOf('=');
    if (delimiterIndex > 0) {
        String param = command.substring(0, delimiterIndex);
        String value = command.substring(delimiterIndex + 1);

        if (param == "Kp") {
            kp = value.toFloat();
            storedKp = kp;
            Serial.println("Kp updated: " + String(kp));
        } else if (param == "Ki") {
            ki = value.toFloat();
            storedKi = ki;
            Serial.println("Ki updated: " + String(ki));
        } else if (param == "Kd") {
            kd = value.toFloat();
            storedKd = kd;
            Serial.println("Kd updated: " + String(kd));
        } else if (param == "R") {
            setpoint = value.toInt();
            storedSetpoint = setpoint;
            Serial.println("Target RPM updated: " + String(setpoint));
        } else if (param == "D") {
            if (value == "CW") {
                storedDirection = true;
                setMotorDirection(true);
                Serial.println("Direction set to CW");
            } else if (value == "CCW") {
                storedDirection = false;
                setMotorDirection(false);
                Serial.println("Direction set to CCW");
            }
        } else if (param == "C") {
            if (value == "GO") {
                isRunning = true;
                CommandRun(true);
                Serial.print("GO Command Received. Parameters: ");
                Serial.print("Kp: "); Serial.print(storedKp);
                Serial.print(", Ki: "); Serial.print(storedKi);
                Serial.print(", Kd: "); Serial.print(storedKd);
                Serial.print(", RPM: "); Serial.print(storedSetpoint);
                Serial.print(", Direction: "); Serial.println(storedDirection ? "CW" : "CCW");
            } else if (value == "STOP") {
                isRunning = false;
                CommandRun(false);
                analogWrite(ENA, 0);
                Serial.println("Stopping System");
            }
        }
    }
}

void CommandRun(bool go) {
    if (go) {
        setMotorDirection(storedDirection);
        pidOutput = 0;
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        setpoint = 0;
        pidOutput = 0;
    }
}

void setMotorDirection(bool clockwise) {
    if (clockwise) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
}