### Kursinhalt embdded software 2026

### Hardware  
**Basic HW**  
   Esp32 board mit MPU6050  
**Ext HW**  
    Tumbler Robot
    
### User interface  
Die uC-Programme werden über das One-Button-Menü oder SvVis-Commands ( *SvProtocol* ) parametriert und gesteuert.  
Online-Werte werden über SvVis-Kanäle visualisiert.  
Ein SvVis konformes printf steht zur verfügung.  
SvVis3 wird als Standart Oberfläche zum Betrieb der uC-Programme verwendet.  
Für manche Übungen werden eigene GUI's in Python oder C# erstellt.  
Die eigenen GUI's kommunizieren über *SvProtocol* mit dem uC

### Basics
- uC Template-App zur Verwendung von *SvProtocol* und SvVis.  
  das HelloWorld für alle weiteren uC-Apps
- MPU-Daten ( Acc/Gyro ) vom Treiber lesen und im SvVis darstellen und analysieren.  
  TP-Filterung besprechen und demonstrieren.
- Eigene GUI mit *SvProtocol*. Verwendung des Templates SvVisMinni.cs oder SvVisMinni.py
- Projekt 2-Dim Wasserwaage.  
  Cursor in der GUI mit Acc-Sensor steuern  
  
### Led-Strip
Auf dem Tumbler-Board befindet sich ein LED-Strip ( 8-RGB Leds ),  
welche einzeln mit Farbe, Helligkeit, und Kontrast angesteuert werden können.  
Damit sind der Beispiele Fantasie keine Grenzen mehr gesetzt.  
**Matura Pool:**  
Anzeige der Acc-Wasserwage auf dem Led-Strip

### FuncGen und Filters
- Effiziente Rect, Triangle, Sine Funktionsgenerator Objekte.
- Analyse der Funktionen mit SvVis ( Fs=100Hz )
- Ausgabe von Echtzeit Audiosignalen ( Fs=20kHz )
- Digitale Filterung der FG-Signale
- Fourier-Synthese

### Motor und Encoder
- Funktion `void DriveDistance(pow, distTicks) void Turn(pow, distTicks)`  
  damit Rechteck fahren
- Mit US-Sensor bis zur Wand umdrehen . . .
- Mit US-Sensor Abstand halten
- Search and Follow mit US-Sensor

### Regelung
- RateLimiter Klasse
- Position Control Example
- Gyro Integrator Example ( um die Fahrtrichtung zu bestimmen )
- Angle Controller ( Fahrtrichtung )

### Regelung Anwendung
- Mithilfe des Angle Controllers eine Abfolge von Geraden und Kurven fahren,
  mit varibler Geschwindigkeit.
- Trapetzförmiges Geschwindigkeits-Profil mit RPM und Angle Controller

### FSST Matura Pool
- Acc-Sensor Cursor-Steuerung uC und C# oder Python
- Alle Regelungs Beispiele
- Alle Regelung Anwendungs Beispiele
- FuncGen und Filters
- Gyro-Integrator am uC Richtungsanzeige mit GUI
  
  






















