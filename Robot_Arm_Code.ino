// Include InverseK and Servo libraries
#include <InverseK.h>
#include <Servo.h>

Servo base;
Servo upperarm;
Servo forearm;
Servo hand;
Servo pinza;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars]; // Temporary array for parsing

int ESTADO {};

// Line coordinates
float xA, yA, zA;
float xB, yB, zB;

// Circle coordinates
float r, xr, yr, zr;
float phi, theta, psi;

// Number of trajectory divisions
float n;

boolean newData = false;

void setup() {
  Serial.begin(9600);

  // Servo pin assignments
  base.attach(3);
  upperarm.attach(5);
  forearm.attach(6);
  hand.attach(9);
  pinza.attach(10);
  
  // Define link lengths (mm) and angular limits
  Link baseLink, upperarmLink, forearmLink, handLink;

  baseLink.init(0, b2a(0.0), b2a(180.0));
  upperarmLink.init(146.71, b2a(0.0), b2a(165.0));
  forearmLink.init(146.71, b2a(0.0), b2a(180.0));
  handLink.init(201.1, b2a(0.0), b2a(180.0));

  // Attach links to Inverse Kinematics model
  InverseK.attach(baseLink, upperarmLink, forearmLink, handLink);

  ESTADO = 1;
}

void loop() {
  switch (ESTADO) {
    case 1:
      ESTADO = def();
      break;
    case 2:
      ESTADO = Ingresocoordenadaslinea();
      break;
    case 3: 
      ESTADO = anguloslinea();
      break;
    case 4:
      ESTADO = Ingresocoordenadascirculo();
      break;
    case 5:
      ESTADO = anguloscirculo();
      break;
  }
}

// Default home position and menu selection
int def() {
  base.write(90);
  upperarm.write(83);
  forearm.write(71);
  hand.write(20);
  pinza.write(82);

  newData = false;
  
  Serial.println("Ingrese: Linea o Circulo");
  delay(2000);

  if (Serial.available()) {
    String serialData = Serial.readString();
    serialData.trim();
    if (serialData == "Linea") {
      return 2;  
    }
    if (serialData == "Circulo") {
      return 4;
    }
  }
  
  return 1;
}

// Parse line input string: <n, xA, yA, zA, xB, yB, zB>
int Ingresocoordenadaslinea() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
        strcpy(tempChars, receivedChars);
        char * strtokIndx;

        strtokIndx = strtok(tempChars, ",");
        n = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        xA = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        yA = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        zA = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        xB = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        yB = atof(strtokIndx);  
    
        strtokIndx = strtok(NULL, ",");
        zB = atof(strtokIndx);

        return 3;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  Serial.println("Ingrese coordenadas como <n,xA,yA,zA,xB,yB,zB>");
  delay(2000);

  return 2;
}

// Compute and execute 3D line trajectory
int anguloslinea() {
  Serial.print("xA "); Serial.println(xA);
  Serial.print("yA "); Serial.println(yA);
  Serial.print("zA "); Serial.println(zA);
  Serial.print("xB "); Serial.println(xB);
  Serial.print("yB "); Serial.println(yB);
  Serial.print("zB "); Serial.println(zB);
  Serial.println("====================================================");

  for (float i = 0; i <= n; i++) {
    float a0, a1, a2, a3;

    if (InverseK.solve(xA + (i/n)*(xB - xA), yA + (i/n)*(yB - yA), zA + (i/n)*(zB - zA), a0, a1, a2, a3)) {
      base.write(int(a2b(a0)));
      upperarm.write(int(a2b(a1)));
      forearm.write(int(a2b(a2)));
      hand.write(int(a2b(a3)));
      pinza.write(82);

      // Pause at start point until "start" command is received
      while (i == 0) {
        base.write(int(a2b(a0)));
        upperarm.write(int(a2b(a1)));
        forearm.write(int(a2b(a2)));
        hand.write(int(a2b(a3)));
        pinza.write(82);
        if (Serial.available()) {
          String serialData = Serial.readString();
          serialData.trim();
          if (serialData == "start") {
            break;  
          } 
        }
      }

      Serial.print(int(a2b(a0))); Serial.print(',');
      Serial.print(int(a2b(a1))); Serial.print(',');
      Serial.print(int(a2b(a2))); Serial.print(',');
      Serial.println(int(a2b(a3)));
    } else {
      Serial.println("No solution found!");
      return 1;
    } 

    // Hold position at target end point until "break" command is received
    while (i == n) {
      base.write(int(a2b(a0)));
      upperarm.write(int(a2b(a1)));
      forearm.write(int(a2b(a2)));
      hand.write(int(a2b(a3)));
      pinza.write(82);
      if (Serial.available()) {
        String serialData = Serial.readString();
        serialData.trim();
        if (serialData == "break") {
          return 1;  
        } 
      }
    }
  }

  return 1;
}

// Parse circle input string: <n, r, xr, yr, zr, phi, theta, psi>
int Ingresocoordenadascirculo() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
        strcpy(tempChars, receivedChars);
        char * strtokIndx;

        strtokIndx = strtok(tempChars, ",");
        n = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        r = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        xr = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        yr = atof(strtokIndx);
    
        strtokIndx = strtok(NULL, ",");
        zr = atof(strtokIndx);

        strtokIndx = strtok(NULL, ",");
        phi = atof(strtokIndx);

        strtokIndx = strtok(NULL, ",");
        theta = atof(strtokIndx);

        strtokIndx = strtok(NULL, ",");
        psi = atof(strtokIndx);

        return 5;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }

  Serial.println("Ingrese coordenadas como <n,r,xr,yr,zr,phi,theta,psi>");
  delay(2000);

  return 4;
}

// Compute and execute 3D circle trajectory using Euler rotation matrix
int anguloscirculo() {
  Serial.print("r "); Serial.println(r);
  Serial.print("xr "); Serial.println(xr);
  Serial.print("yr "); Serial.println(yr);
  Serial.print("zr "); Serial.println(zr);
  Serial.print("phi "); Serial.println(phi);
  Serial.print("theta "); Serial.println(theta);
  Serial.print("psi "); Serial.println(psi);
  Serial.println("====================================================");

  // Euler Z-X-Z Rotation Matrix
  float a11 = cos(psi * M_PI / 180) * cos(phi * M_PI / 180) - cos(theta * M_PI / 180) * sin(phi * M_PI / 180) * sin(psi * M_PI / 180);
  float a12 = cos(psi * M_PI / 180) * sin(phi * M_PI / 180) + cos(theta * M_PI / 180) * cos(phi * M_PI / 180) * sin(psi * M_PI / 180);
  float a13 = sin(psi * M_PI / 180) * sin(theta * M_PI / 180);

  float a21 = -sin(psi * M_PI / 180) * cos(phi * M_PI / 180) - cos(theta * M_PI / 180) * sin(phi * M_PI / 180) * cos(psi * M_PI / 180);
  float a22 = -sin(psi * M_PI / 180) * sin(phi * M_PI / 180) + cos(theta * M_PI / 180) * cos(phi * M_PI / 180) * cos(psi * M_PI / 180);
  float a23 = cos(psi * M_PI / 180) * sin(theta * M_PI / 180);

  float a31 = sin(theta * M_PI / 180) * sin(phi * M_PI / 180);
  float a32 = -sin(theta * M_PI / 180) * cos(phi * M_PI / 180);
  float a33 = cos(theta * M_PI / 180);

  for (float i = 0; i <= n; i++) {
    float a0, a1, a2, a3;

    float px = xr + cos((i / n) * 2 * M_PI) * r;
    float py = yr + sin((i / n) * 2 * M_PI) * r;
    float pz = zr;

    float x_rot = a11 * px + a12 * py + a13 * pz;
    float y_rot = a21 * px + a22 * py + a23 * pz;
    float z_rot = a31 * px + a32 * py + a33 * pz;

    if (InverseK.solve(x_rot, y_rot, z_rot, a0, a1, a2, a3)) {
      base.write(int(a2b(a0)));
      upperarm.write(int(a2b(a1)));
      forearm.write(int(a2b(a2)));
      hand.write(int(a2b(a3)));
      pinza.write(82);

      // Pause at start point until "start" command is received
      while (i == 0) {
        base.write(int(a2b(a0)));
        upperarm.write(int(a2b(a1)));
        forearm.write(int(a2b(a2)));
        hand.write(int(a2b(a3)));
        pinza.write(82);
        if (Serial.available()) {
          String serialData = Serial.readString();
          serialData.trim();
          if (serialData == "start") {
            break;  
          } 
        }
      }

      Serial.print(int(a2b(a0))); Serial.print(',');
      Serial.print(int(a2b(a1))); Serial.print(',');
      Serial.print(int(a2b(a2))); Serial.print(',');
      Serial.println(int(a2b(a3)));
    } else {
      Serial.println("No solution found!");
      return 1;
    } 

    // Hold position at completion until "break" command is received
    while (i == n) {
      base.write(int(a2b(a0)));
      upperarm.write(int(a2b(a1)));
      forearm.write(int(a2b(a2)));
      hand.write(int(a2b(a3)));
      pinza.write(82);
      if (Serial.available()) {
        String serialData = Serial.readString();
        serialData.trim();
        if (serialData == "break") {
          return 1;  
        } 
      }
    }
  }

  return 1;
}

// Angle unit conversion helpers
float b2a(float b) {
  return b / 180.0 * PI - HALF_PI;
}

float a2b(float a) {
  return (a + HALF_PI) * 180 / PI;
}
