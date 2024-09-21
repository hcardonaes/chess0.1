/*
 Name:		chess333.ino
 Created:	21/09/2024 12:54:43
 Author:	Ofel
*/
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <TMCStepper.h>

struct Coordenadas {
    double x;
    double y;
};

struct Angles {
    double theta1;
    double theta2;
};
float last_theta1 = 0.0;
float last_theta2 = 0.0;
bool homingRealizado = false;


// Longitudes de los enlaces del brazo SCARA
const float L1 = 100.0;  // Longitud del primer enlace (en mm)
const float L2 = 100.0;  // Longitud del segundo enlace (en mm)

long pasosMotores[2]; // Array para almacenar las pasosMotores objetivo
double posicionActualX = 0.0;
double posicionActualY = 0.0;

// Configuración del motor del hombro (NEMA17 con TMC2209)
constexpr auto STEP_PIN = 6;
constexpr auto DIR_PIN = 3;
constexpr auto ENABLE_PIN = 51;
constexpr auto R_SENSE = 0.11f;      // R_SENSE para cálculo de corriente
constexpr auto DRIVER_ADDRESS = 0b00;       // Dirección del driver TMC2209 según MS1 y MS2
#define SERIAL_PORT Serial3

// Configuración del motor del codo (24BYJ48 con ULN2003)
constexpr auto HALFSTEP = 8;
constexpr auto motorPin1 = 8;     // IN1 on ULN2003 ==> Blue   on 28BYJ-48
constexpr auto motorPin2 = 9;     // IN2 on ULN2004 ==> Pink   on 28BYJ-48
constexpr auto motorPin3 = 10;    // IN3 on ULN2003 ==> Yellow on 28BYJ-48
constexpr auto motorPin4 = 11;    // IN4 on ULN2003 ==> Orange on 28BYJ-48

constexpr auto LEVA_HOMBRO_PIN = 5;
constexpr auto LEVA_CODO_PIN = 4;


TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper hombro(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
AccelStepper codo(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
MultiStepper motores;

// Variables para controlar el ángulo anterior de θ1
float prevTheta1 = 0;

// Definición de la función de cinemática inversa 333
void inverseKinematics(float x, float y, float& theta1, float& theta2) {
    float D = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    theta2 = acos(D);  // Ángulo del segundo enlace en radianes
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)); // Ángulo del primer enlace

    // Convertir radianes a grados
    theta1 = theta1 * 180.0 / PI;
    theta2 = theta2 * 180.0 / PI;

    // Controlar la continuidad de theta1 (evitar cambios bruscos de signo)
    if (abs(theta1 - prevTheta1) > 180) {
        if (theta1 > prevTheta1) {
            theta1 -= 360;  // Ajuste para evitar que θ1 cambie bruscamente de dirección
        }
        else {
            theta1 += 360;
        }
    }

    // Actualizar el valor previo de θ1
    prevTheta1 = theta1;
}

void moverEnGrados(double anguloHombro, double anguloCodo) {
    // Convertir ángulos a pasos de motor (en pasos)
    long pasosHombro = calcularPasosHombro(anguloHombro);
    long pasosCodo = calcularPasosCodo(anguloCodo);

    // Imprimir los ángulos y pasos calculados
    Serial.print("Moviendo a ángulos: Hombro = "); Serial.print(anguloHombro);
    Serial.print(" grados, Codo = "); Serial.print(anguloCodo); Serial.println(" grados");
    Serial.print("Pasos: Hombro = "); Serial.print(pasosHombro);
    Serial.print(", Codo = "); Serial.println(pasosCodo);

    // Mover los motores a las posiciones calculadas
    pasosMotores[0] = pasosHombro;
    pasosMotores[1] = pasosCodo;
    motores.moveTo(pasosMotores);
    motores.runSpeedToPosition();

    // Actualizar los ángulos actuales
    last_theta1 = anguloHombro * PI / 180.0;
    last_theta2 = anguloCodo * PI / 180.0;
}

long calcularPasosHombro(double theta1) {
    double pasosPorGradoHombro = 16150 / 360; // 15450 pasosMotores por vuelta
    long pasos = theta1 * pasosPorGradoHombro;
    return pasos;
}

long calcularPasosCodo(double theta2) {
    double pasosPorGradoCodo = 4140.0 / 360;
    long pasos = (theta2 + 180) * pasosPorGradoCodo;
    //Serial.print("theta2: "); Serial.println(theta2);
    return pasos;
}

void realizarHoming() {
    Serial.println("Realizando homing para los motores...");
    // Homing para el codo
    bool estadoLeva = digitalRead(LEVA_CODO_PIN);
    if (estadoLeva == LOW) {
        // Mover el motor hasta que se active el fin de carrera
        codo.setSpeed(-800);
        while (digitalRead(LEVA_CODO_PIN) == LOW) {
            codo.runSpeed();
        }
    }
    else {
        codo.setSpeed(800);
        while (digitalRead(LEVA_CODO_PIN) == HIGH) {
            codo.runSpeed();
        }
    }
    codo.setCurrentPosition(0); // Establece la posición actual transitoria como cero

    // Homing para el hombro
    estadoLeva = digitalRead(LEVA_HOMBRO_PIN);
    //Serial.print("Estado leva hombro: "); //Serial.println(estadoLeva);
    if (estadoLeva == LOW) {
        // Mover el motor hasta que se active el fin de carrera
        hombro.setSpeed(-800);
        while (digitalRead(LEVA_HOMBRO_PIN) == LOW) {
            hombro.runSpeed();
        }
    }
    else {
        hombro.setSpeed(800);
        while (digitalRead(LEVA_HOMBRO_PIN) == HIGH) {
            hombro.runSpeed();
        }
    }
    hombro.setCurrentPosition(0); // Establece la posición actual como cero

    pasosMotores[0] = 4060;
    pasosMotores[1] = -1625;
    // Mover los motores a la posición deseada
    motores.moveTo(pasosMotores);
    motores.runSpeedToPosition();
    // Establece la posición actual como cero
    codo.setCurrentPosition(0);
    hombro.setCurrentPosition(0);

}

Coordenadas calcularCoordenadasDesdeCentro(String comando)
{
    int columna = comando[0] - 'a'; // Columna [a-h]
    int fila = comando[1] - '1';    // Fila [1-8]
    double x = (columna - 3.5) * 40; // Ajuste para centrar en el tablero
    double y = (fila - 3.5) * 40;    // Ajuste para centrar en el tablero
    return { x, y };
}

void moverADestino(String comando) {
    // Calcular las coordenadas del destino
    Coordenadas destino = calcularCoordenadasDesdeCentro(comando);
    float xf = destino.x;
    float yf = destino.y;

    // Número de pasos de interpolación
    int steps = 20;

    // Variables para almacenar los ángulos de las articulaciones
    float theta1, theta2;

    // Interpolar entre los puntos
    for (int i = 0; i <= steps; i++) {
        // Interpolación lineal de la posición
        float t = (float)i / steps;
        float x = posicionActualX + t * (xf - posicionActualX);
        float y = posicionActualY + t * (yf - posicionActualY);

        // Calcular la cinemática inversa para obtener los ángulos
        inverseKinematics(x, y, theta1, theta2);
        moveToPosition(theta1, theta2);
        posicionActualX = x;
        posicionActualY = y;


    }
}

void moveToPosition(float theta1, float theta2) { 

   	Serial.print("Theta1: ");   
	Serial.print(theta1);
	Serial.print("  Theta2: ");
	Serial.println(theta2);
    // Convertir los ángulos a pasos para los motores
    //long steps1 = theta1 / degreesPerStep;
    //long steps2 = theta2 / degreesPerStep;
	long steps1 = calcularPasosHombro(theta1);
	long steps2 = calcularPasosCodo(theta2);

	pasosMotores[0] = steps1;
	pasosMotores[1] = steps2;
	// Mover los motores a la posición deseada
	motores.moveTo(pasosMotores);
	motores.runSpeedToPosition();



    //stepper1.moveTo(steps1);
    //stepper2.moveTo(steps2);

    //// Ejecutar el movimiento simultáneo de ambos motores
    //while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    //    stepper1.run();
    //    stepper2.run();
    //}
}

void setup() {
    Serial.begin(115200);

    SERIAL_PORT.begin(115200); // Configura la comunicación con el TMC2209

    // Configuración del driver TMC2209
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Habilitar el driver

    driver.begin();
    driver.toff(5);
    driver.rms_current(1000); // Corriente en mA para el NEMA17
    driver.microsteps(16);   // Configurar microstepping
    driver.en_spreadCycle(false); // Deshabilitar spreadCycle, usa StealthChop
    driver.pwm_autoscale(true); // Activar auto scale PWM
    driver.enn(); // Habilitar el driver

    // Configuración de los pines
    pinMode(LEVA_HOMBRO_PIN, INPUT_PULLUP);
    pinMode(LEVA_CODO_PIN, INPUT_PULLUP);

    // Configuración inicial de los motores
    hombro.setMaxSpeed(800);
    hombro.setAcceleration(800);
    codo.setMaxSpeed(400);
    codo.setAcceleration(200);

    // Add the motores to the MultiStepper object
    motores.addStepper(hombro); // position '0'
    motores.addStepper(codo);    // position '1'

    // Homing: Mueve los motores hasta posicionarse en el origen
    Serial.println("Realizando homing...");
    realizarHoming();
    delay(3000);

    //moverEnGrados(0, 360);
    //while (true)
    //{}
    Serial.println("Homing completado. Listo para recibir comandos.");
    homingRealizado = true;
    Serial.flush();
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        Serial.print("Comando recibido: "); Serial.println(input);

        // Validar que el comando tenga la estructura de una casilla de ajedrez
        if (input.length() == 2 && input[0] >= 'a' && input[0] <= 'h' && input[1] >= '1' && input[1] <= '8') {
            //moverAPosicion(input);
            moverADestino(input);
        }
        else {
            Serial.println("Comando inválido. Debe ser una casilla de ajedrez (a-h, 1-8).");
        }
    }

	//-----------------------------------------
 //   // Definir los puntos inicial y final de la trayectoria
 //   //float x0 = -140.0, y0 = -140.0; // Coordenadas iniciales
 //   //float xf = -140.0, yf = 140.0; // Coordenadas finales
	//float x0 = 0.0, y0 = 0.0; // Coordenadas iniciales
	//float xf = -140.0, yf = -140.0; // Coordenadas finales

 //   // Número de pasos de interpolación
 //   int steps = 20;

 //   // Variables para almacenar los ángulos de las articulaciones
 //   float theta1, theta2;

 //   // Interpolar entre los puntos
 //   for (int i = 0; i <= steps; i++) {
 //       // Interpolación lineal de la posición
 //       float t = (float)i / steps;
 //       float x = x0 + t * (xf - x0);
 //       float y = y0 + t * (yf - y0);

 //       // Calcular la cinemática inversa para obtener los ángulos
 //       inverseKinematics(x, y, theta1, theta2);

 //       // Mover los motores a los nuevos ángulos calculados
 //       moveToPosition(theta1, theta2);

 //       delay(500);  // Esperar 50 ms entre cada paso (ajusta si es necesario)
 //   }

}


//void moveToPosition(float theta1, float theta2) {
//	Serial.print("Theta1: ");   
//	Serial.print(theta1);
//	Serial.print("  Theta2: ");
//	Serial.println(theta2);
//
//    // Convertir los ángulos a pasos para los motores
//    //long steps1 = theta1 / degreesPerStep;
//    //long steps2 = theta2 / degreesPerStep;
//
//    //// Mover los motores a la posición deseada
//    //stepper1.moveTo(steps1);
//    //stepper2.moveTo(steps2);
//
//    //// Ejecutar el movimiento simultáneo de ambos motores
//    //while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
//    //    stepper1.run();
//    //    stepper2.run();
//    //}
//}
//
//void loop() {
//    // Definir los puntos inicial y final de la trayectoria
//    float x0 = -140.0, y0 = -140.0; // Coordenadas iniciales
//    float xf = -140.0, yf = 140.0; // Coordenadas finales
//
//    // Número de pasos de interpolación
//    int steps = 20;
//
//    // Variables para almacenar los ángulos de las articulaciones
//    float theta1, theta2;
//
//    // Interpolar entre los puntos
//    for (int i = 0; i <= steps; i++) {
//        // Interpolación lineal de la posición
//        float t = (float)i / steps;
//        float x = x0 + t * (xf - x0);
//        float y = y0 + t * (yf - y0);
//
//        // Calcular la cinemática inversa para obtener los ángulos
//        inverseKinematics(x, y, theta1, theta2);
//
//        // Mover los motores a los nuevos ángulos calculados
//        moveToPosition(theta1, theta2);
//
//        delay(50); // Esperar 50 ms entre cada paso (ajusta si es necesario)
//    }
//    while (true)
//    {
//
//    }
//   //elay(2000); // Pausa de 2 segundos antes de repetir el movimiento
//}
