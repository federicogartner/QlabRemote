#include <MIDIUSB.h>
#include <Bounce2.h>
#include <TM1637Display.h>
#include <math.h> // Para funciones matemáticas

// Pines del botón y LED
#define button1 5
#define led1 6 // pin PWM
#define button2 7
#define led2 9 // pin PWM
#define button3 8
#define led3 10 // pin PWM

// Pines del encoder
#define PIN_A 2
#define PIN_B 3
#define PIN_P 4

// Notas MIDI para los botones y el encoder
#define NOTE_C3 48
#define NOTE_CSharp3 49 // Nota para el botón 2
#define NOTE_D3 50 // Nota para el botón 3
#define NOTE_E3 52 // Nota para el movimiento a la izquierda cuando el botón del encoder está presionado
#define NOTE_F3 53 // Nota para el movimiento a la derecha cuando el botón del encoder está presionado
#define NOTE_DS3 51 // Nota para el botón del encoder

// Control Change para el encoder
#define CC_BRIGHTNESS 16

// Brillo máximo y mínimo para el LED y el display
#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0
#define DISPLAY_MAX_BRIGHTNESS 7 // Brillo máximo del display (0-7)
#define DISPLAY_MIN_BRIGHTNESS 0 // Brillo mínimo del display (0-7)
#define IDLE_BRIGHTNESS 0.5 // 50% de brillo cuando el botón no está presionado

// Estados del botón
Bounce debouncedButton1 = Bounce();
Bounce debouncedButtonP = Bounce();
Bounce debouncedButton2 = Bounce();
Bounce debouncedButton3 = Bounce();

// Variables del encoder
uint8_t lrmem;
int num = 64; // Posición inicial en el centro del rango (50%)
int lastCCValue = -1; // Último valor MIDI CC enviado
bool brightnessAdjustmentMode = false;
unsigned long buttonPressTime = 0;
unsigned long lastBlinkTime = 0;
bool displayState = true;

// Pines para el TM1637
#define CLK 14
#define DIO 15

// Inicializar TM1637
TM1637Display display(CLK, DIO);

// Variables para almacenar el tiempo actual
volatile int minutes = 0;
volatile int seconds = 0;
volatile int lastSeconds = -1; // Para verificar cambios en los segundos

// Variables globales para el timecode
byte timecode[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Función para enviar Note On
void sendNoteOn(byte note, byte velocity, byte channel) {
    midiEventPacket_t noteOn = {0x09, 0x90 | channel, note, velocity};
    MidiUSB.sendMIDI(noteOn);
    MidiUSB.flush();
}

// Función para enviar Note Off
void sendNoteOff(byte note, byte velocity, byte channel) {
    midiEventPacket_t noteOff = {0x08, 0x80 | channel, note, velocity};
    MidiUSB.sendMIDI(noteOff);
    MidiUSB.flush();
}

// Función para enviar Control Change
void sendControlChange(byte controller, byte value, byte channel) {
    midiEventPacket_t controlChange = {0x0B, 0xB0 | channel, controller, value};
    MidiUSB.sendMIDI(controlChange);
    MidiUSB.flush();
}

// Función para calcular el brillo logarítmico
int calculateLogarithmicBrightness(int value) {
    // Normalizar el valor entre 0 y 127
    float normalizedValue = value / 127.0;

    // Aplicar la función exponencial para obtener un valor logarítmico
    float logBrightness = pow(normalizedValue, 2.2); // Puedes ajustar el exponente para cambiar la curva

    // Mapear el brillo logarítmico al rango de brillo del LED
    return map(logBrightness * 127, 0, 127, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
}

// Función para calcular el brillo del display
int calculateDisplayBrightness(int value) {
    // Normalizar el valor entre 0 y 127
    float normalizedValue = value / 127.0;

    // Mapear el valor normalizado al rango de brillo del display (0-7)
    return map(normalizedValue * 127, 0, 127, DISPLAY_MIN_BRIGHTNESS, DISPLAY_MAX_BRIGHTNESS);
}

// Configuración inicial del Arduino
void setup() {
    // Configurar botones como entrada con resistencia pull-up
    debouncedButton1.attach(button1, INPUT_PULLUP);
    debouncedButtonP.attach(PIN_P, INPUT_PULLUP);
    debouncedButton2.attach(button2, INPUT_PULLUP);
    debouncedButton3.attach(button3, INPUT_PULLUP);

    // Configurar intervalo de debounce
    debouncedButton1.interval(20);
    debouncedButtonP.interval(20);
    debouncedButton2.interval(20);
    debouncedButton3.interval(20);

    // Configurar LEDs como salida
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);

    // Apagar o atenuar los LEDs incorporados del Arduino
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // Apagar el LED incorporado

    // Apagar los LEDs RX y TX
    pinMode(0, OUTPUT);
    digitalWrite(0, LOW);
    pinMode(1, OUTPUT);
    digitalWrite(1, LOW);

    // Inicializar LEDs apagados
    int initialBrightness = calculateLogarithmicBrightness(num);
    analogWrite(led1, initialBrightness);
    analogWrite(led2, initialBrightness);
    analogWrite(led3, initialBrightness);

    // Inicializar el display TM1637
    display.setBrightness(calculateDisplayBrightness(num)); // Ajustar el brillo inicial del display

    Serial.begin(115200);
    Serial.println("Sistema configurado con 3 botones, 3 LEDs, encoder, display TM1637 y MIDI USB.");

    // Configurar interrupciones para el encoder
    attachInterrupt(digitalPinToInterrupt(PIN_A), rotary, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_B), rotary, CHANGE);
}

// Función para actualizar el brillo de los LEDs según el estado de los botones
void updateLedBrightness(int ledPin, Bounce &button, int brightness) {
    if (button.read() == LOW) {
        analogWrite(ledPin, brightness);
    } else {
        analogWrite(ledPin, brightness * IDLE_BRIGHTNESS);
    }
}

// Bucle principal del Arduino
void loop() {
    // Leer y estabilizar el valor del encoder
    num = constrain(num, 0, 127); // Asegurar que num se mantenga dentro del rango 0-127
    int brightnessStep = num; // Mapear a pasos de 1
    int brightness = calculateLogarithmicBrightness(brightnessStep);
    int displayBrightness = calculateDisplayBrightness(brightnessStep);

    // Actualizar estado de los botones
    debouncedButton1.update();
    debouncedButtonP.update();
    debouncedButton2.update();
    debouncedButton3.update();

    // Verificar si el botón del encoder está presionado y mantener la nota D#3 encendida
    if (debouncedButtonP.fell()) {
        buttonPressTime = millis();
        sendNoteOn(NOTE_DS3, 127, 0);
        Serial.println("Nota D#3 ON enviada");
    }
    if (debouncedButtonP.rose()) {
        if (!brightnessAdjustmentMode) {
            sendNoteOff(NOTE_DS3, 127, 0);
            Serial.println("Nota D#3 OFF enviada");
        }
    }

    // Verificar si el botón del encoder está presionado por más de 3 segundos para entrar en modo de ajuste de brillo
    if (debouncedButtonP.read() == LOW && (millis() - buttonPressTime) > 3000) {
        brightnessAdjustmentMode = !brightnessAdjustmentMode; // Cambiar el modo
        buttonPressTime = millis(); // Reiniciar el tiempo de presionado
        if (brightnessAdjustmentMode) {
            Serial.println("Entrando en modo de ajuste de brillo");
        } else {
            // Confirmar el ajuste de brillo
            sendNoteOff(NOTE_DS3, 127, 0);
            display.setBrightness(displayBrightness); // Establecer el brillo ajustado del display
            Serial.println("Saliendo del modo de ajuste de brillo");
        }
    }

    // Modo de ajuste de brillo
    if (brightnessAdjustmentMode) {
        // Parpadeo del display
        if ((millis() - lastBlinkTime) > 500) {
            displayState = !displayState;
            display.setBrightness(displayState ? DISPLAY_MAX_BRIGHTNESS : DISPLAY_MIN_BRIGHTNESS);
            lastBlinkTime = millis();
        }

        // Ajuste de brillo sin necesidad de presionar el botón
        if (brightnessStep != lastCCValue) {
            analogWrite(led1, brightness);
            analogWrite(led2, brightness);
            analogWrite(led3, brightness);

            sendControlChange(CC_BRIGHTNESS, brightnessStep, 0);
            lastCCValue = brightnessStep;
            Serial.print("MIDI CC Brightness Step: ");
            Serial.println(brightnessStep);
        }
    } else {
        // Modo normal
        if (brightnessStep != lastCCValue) {
            if (debouncedButtonP.read() != LOW) {
                // Enviar notas E y F
                if (brightnessStep > lastCCValue) {
                    sendNoteOn(NOTE_F3, 127, 0);
                    sendNoteOff(NOTE_F3, 127, 0);
                    Serial.println("Nota F3 enviada");
                } else if (brightnessStep < lastCCValue) {
                    sendNoteOn(NOTE_E3, 127, 0);
                    sendNoteOff(NOTE_E3, 127, 0);
                    Serial.println("Nota E3 enviada");
                }
                lastCCValue = brightnessStep;
            }
        }
    }

    // Manejar botón 1
    if (debouncedButton1.fell()) {
        sendNoteOn(NOTE_C3, 127, 0);
        Serial.println("Nota C3 ON enviada");
    }
    if (debouncedButton1.rose()) {
        sendNoteOff(NOTE_C3, 127, 0);
        Serial.println("Nota C3 OFF enviada");
    }

    // Manejar botón 2
    if (debouncedButton2.fell()) {
        sendNoteOn(NOTE_CSharp3, 127, 0);
        Serial.println("Nota C#3 ON enviada");
    }
    if (debouncedButton2.rose()) {
        sendNoteOff(NOTE_CSharp3, 127, 0);
        Serial.println("Nota C#3 OFF enviada");
    }

    // Manejar botón 3
    if (debouncedButton3.fell()) {
        sendNoteOn(NOTE_D3, 127, 0);
        Serial.println("Nota D3 ON enviada");
    }
    if (debouncedButton3.rose()) {
        sendNoteOff(NOTE_D3, 127, 0);
        Serial.println("Nota D3 OFF enviada");
    }

    // Actualizar el brillo de los LEDs en función del estado de los botones
    updateLedBrightness(led1, debouncedButton1, brightness);
    updateLedBrightness(led2, debouncedButton2, brightness);
    updateLedBrightness(led3, debouncedButton3, brightness);

    // Leer los mensajes MIDI entrantes
    midiEventPacket_t rx;
    do {
        rx = MidiUSB.read();
        if (rx.header != 0) {
            handleMIDIMessage(rx);
        }
    } while (rx.header != 0);

    // Refrescar el display
    multiplexDisplay(minutes, seconds);

    delay(50); // Reducir el retraso para mejorar la respuesta
}

// Manejar los mensajes MIDI entrantes
void handleMIDIMessage(midiEventPacket_t rx) {
    if (rx.byte1 == 0xB0 && rx.byte2 == CC_BRIGHTNESS) { // Control Change Message for brightness
        int brightnessValue = rx.byte3;
        num = brightnessValue;

        // Actualizar el brillo de los LEDs y del display
        int brightness = calculateLogarithmicBrightness(num);
        int displayBrightness = calculateDisplayBrightness(num);
        
        analogWrite(led1, brightness);
        analogWrite(led2, brightness);
        analogWrite(led3, brightness);
        
        display.setBrightness(displayBrightness);

        Serial.print("Control Change for Brightness: ");
        Serial.println(brightnessValue);
    }

    if (rx.byte1 == 0xF1) { // Quarter Frame Message
        handleMTCQuarterFrame(rx.byte2);
    }
}

// Manejar los mensajes de Quarter Frame del código de tiempo MIDI
void handleMTCQuarterFrame(byte data1) {
    byte messageType = (data1 & 0x70) >> 4;
    byte value = data1 & 0x0F;

    // Actualizar la parte correspondiente del timecode
    timecode[messageType] = value;

    // Solo actualizar los valores de minutos y segundos si todos los nibbles necesarios están disponibles
    if (messageType == 7) {
        // Decodificar los segundos
        seconds = (timecode[2] & 0x0F) | ((timecode[3] & 0x07) << 4);
        // Decodificar los minutos
        minutes = (timecode[4] & 0x0F) | ((timecode[5] & 0x07) << 4);

        // Mostrar el tiempo decodificado solo si los segundos han cambiado
        if (seconds != lastSeconds) {
            Serial.print("Tiempo Decodificado -> Minutos: ");
            Serial.print(minutes);
            Serial.print(", Segundos: ");
            Serial.println(seconds);
            lastSeconds = seconds;
        }
    }
}

// Multiplexar el display para mostrar los valores de minutos y segundos
void multiplexDisplay(int minutes, int seconds) {
    // Combinar minutos y segundos en un array de 4 dígitos
    uint8_t data[] = {
        (minutes / 10) % 10,
        minutes % 10,
        (seconds / 10) % 10,
        seconds % 10
    };

    // Mostrar los valores en el display TM1637
    display.showNumberDecEx((minutes * 100) + seconds, 0b01000000, true);
}

// Función de interrupción para manejar la rotación del encoder
void rotary() {
    static int lrsum = 0;
    static int8_t TRANS[] = {0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0};
    int8_t l, r;

    l = digitalRead(PIN_A);
    r = digitalRead(PIN_B);

    lrmem = ((lrmem & 0x03) << 2) + 2 * l + r;
    lrsum = lrsum + TRANS[lrmem];
    /* encoder no está en el estado neutral (detent) */
    if (lrsum % 4 != 0) return;
    /* encoder en el estado neutral - rotación en sentido horario */
    if (lrsum == 4) {
        lrsum = 0;
        if (brightnessAdjustmentMode) {
            num = num + 1; // Ajustar en pasos de 1
        } else {
            sendNoteOn(NOTE_F3, 127, 0);
            sendNoteOff(NOTE_F3, 127, 0);
            Serial.println("Nota F3 enviada");
        }
        return;
    }
    /* encoder en el estado neutral - rotación en sentido antihorario */
    if (lrsum == -4) {
        lrsum = 0;
        if (brightnessAdjustmentMode) {
            num = num - 1; // Ajustar en pasos de 1
        } else {
            sendNoteOn(NOTE_E3, 127, 0);
            sendNoteOff(NOTE_E3, 127, 0);
            Serial.println("Nota E3 enviada");
        }
        return;
    }
    /* lrsum > 0 si la transición es imposible */
    lrsum = 0;
    return;
}
