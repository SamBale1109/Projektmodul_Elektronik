# 📷 Intelligente Kochfeldüberwachung mittels KI

## 🧠 Projektmodul Elektronik – Motivation und Zielsetzung

Ein häufiges Problem im Alltag älterer Menschen ist das Vergessen von eingeschalteten Herdplatten. Oft wird ein Topf auf dem Herd abgestellt und das Kochfeld eingeschaltet, während parallel andere Tätigkeiten ausgeführt werden. In solchen Fällen kann es vorkommen, dass das Kochgut überkocht oder anbrennt, was ein erhebliches Sicherheitsrisiko darstellt.

**Ziel des Projekts** ist die Entwicklung einer kamerabasierten, KI-gestützten Lösung, die den Kochvorgang in Echtzeit überwacht. Die Anwendung soll durch Auswerten der Bilder einer über der Kochplatte montierten Kamera erkennen, ob sich ein Topf auf dem Herd befindet und ob dessen Inhalt bereits kocht. Bei Erreichen eines definierten Kochzustands soll ein digitales Signal erzeugt werden, das perspektivisch zur Ansteuerung eines Alarms oder zur automatischen Abschaltung genutzt werden kann.

---

## 🔧 Technische Umsetzung

### 1. 📸 Datenerhebung und Annotation
- Bilddaten typischer Kochsituationen aufgenommen (Topf auf Herd aus Vogelperspektive).
- Manuelle Annotation mit **Roboflow** in zwei Klassen:
  - `boiling` (kochend)
  - `not_boiling` (nicht kochend)

### 2. 🧪 Training des Erkennungsmodells
- Training eines **YOLOv8**-Objekterkennungsmodells.
- Ziel: Lokalisierung und Klassifikation der Zustände von Töpfen.
- Verwendung eigener Trainingsdaten.
- Trainingsskript und Modellgewichte werden beigefügt.

### 3. 🐍 Entwicklung des Detektionsskripts
Ein Python-Skript zur Laufzeit-Auswertung:
- Lädt das trainierte YOLOv8-Modell.
- Verarbeitet wahlweise Live-Videodaten (USB-Kamera) oder Testbilder.
- Visualisiert erkannte Objekte inkl. Herdplattengitter.
- Speichert für jede Detektion:
  - erkannte Klasse (kochend/nicht kochend)
  - Position (Herdplatte)
  - Zeitstempel & Dauer des Zustands

### 4. 🍓 Embedded-Einsatz (Raspberry Pi)
- Installation von **Ubuntu 22.04** auf **Raspberry Pi 4**.
- Anschluss einer **USB-Kamera (OVD 3601)** zur Bildaufnahme.
- Echtzeit-Auswertung direkt auf dem Gerät möglich.

### 5. 🔁 ROS2-Integration
- Installation von **ROS2 Humble** auf dem Raspberry Pi.
- Umwandlung des Python-Skripts in eine **ROS2 Node**.
- Zustände bzw. Bilder werden als ROS-Nachrichten im Netzwerk veröffentlicht.
- [Anleitung](https://github.com/SamBale1109/Projektmodul_Elektronik/blob/main/Kamera/README.md) zu Setup und Verwendung der Node
- Ermöglicht Kommunikation mit:
  - Benutzeroberflächen
  - Aktuatoren (z. B. Alarmgeber)
  - Logging-Systemen

---

## 🚀 Weiterführende Arbeiten und Ausblick

- Einführung zusätzlicher Zustände wie:
  - `verkocht`, `angebrannt`, `übergekocht`
- Erkennung von kritischen Veränderungen (z. B. Rauch) in Pfannen.
- Erweiterung um Kochzeitschätzung basierend auf erkannter Nahrung.
- Ausgabe von akustischen Warnsignalen.
- Integration mit **smarten Haushaltsgeräten** zur automatischen Abschaltung.

---

## 🖼️ Visualisierung

Beispielbilder mit erfolgreicher Objekterkennung werden im Anhang dargestellt und demonstrieren die praktische Funktionalität:

- 🟩 Gitterdarstellung zur Herdplattenerkennung  
- 🔵 Markierung der erkannten Töpfe  
- 🔴 Zustandsanzeige (kochend/nicht kochend)

---

## 📦 Ressourcen & Verweise

- **Roboflow Dataset:**  
  👉 [https://app.roboflow.com/projektmodulelektronik/boiling_detection/2]

- **Quellcodes (Beiliegend):**
  - [YOLOv8-Trainingsskript](https://github.com/SamBale1109/Projektmodul_Elektronik/blob/main/Kamera/zustandserkennung/zustandserkennung/yolov8_obb_train.py)
  - [Detektionsskript (Python)](https://github.com/SamBale1109/Projektmodul_Elektronik/blob/main/Kamera/zustandserkennung/zustandserkennung/webcam_detection.py)
  - [ROS2 Node zu Objekterkennung](https://github.com/SamBale1109/Projektmodul_Elektronik/blob/main/Kamera/zustandserkennung/zustandserkennung/boiling_detection_node.py)
  - [ROS2 Node zum USB-Kamera auslesen](https://github.com/SamBale1109/Projektmodul_Elektronik/blob/main/Kamera/zustandserkennung/zustandserkennung/camera_publisher.py)
  - [ROS2 launch file zum Start auf Raspberry Pi](https://github.com/SamBale1109/Projektmodul_Elektronik/blob/main/Kamera/zustandserkennung/zustandserkennung/boiling_detection_node.py)

---

© Projektmodul MEM – [Lukas Sambale / Projektgruppe Elektronik], 2025  

