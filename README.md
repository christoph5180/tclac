# 🌀 TCL Air Conditioner Integration for Home Assistant


> 🇩🇪 Übersetzt mit ChatGPT – erfolgreich getestet mit TCL TAC-09CHSD/TPG11l  
> Vor den Anpassungen hatte ich erhebliche Probleme: Die Einstellungen wurden nicht sofort übernommen und mussten mehrfach wiederholt werden. Außerdem lief die unveränderte Version nicht mit der aktuellen ESPHome-Version 2025.12.0.  
  
> 🇩🇪 Translated with ChatGPT – successfully tested with TCL TAC-09CHSD/TPG11l  
> Before making the adjustments, I had significant issues: the settings were not applied immediately and had to be executed several times. Also, the unmodified version did not work with the current ESPHome version 2025.12.0.


> Einfaches DIY-Projekt mit ESP32 + USB-Kabel. Keine Cloud nötig.

---

## 🛠️ Was du brauchst

- **ESP32** (z. B. ESP32-C3, WROOM32, NodeMCU)
- **USB-A-Stecker oder -Kabel**  
  👉 Ich habe dieses verwendet: [AliExpress-Link](https://www.aliexpress.com/item/1005005776162012.html)
- **Home Assistant mit ESPHome (ab Version 2023.3.0)**

---

## 🔌 Verkabelung

| USB-A Pin | Kabelfarbe | → ESP32 Pin |
|-----------|------------|--------------|
| GND       | Schwarz    | VIN/VCC      |
| D+        | Grün       | GND          |
| D-        | Grau       | RXD          |
| VBUS      | Rot        | TXD          |

### 🔍 Beispielbilder
(Beachte, dass ich hier nicht auf die Farben der Kabel geachtet habe. Die Farben in der Tabelle entsprechen jedoch in der Regel gängigen USB-A-Kabeln, die man einfach abschneiden kann.)

<img src="https://github.com/user-attachments/assets/9b674e06-41ca-4bcf-b09b-691a5fbd8545" width="400"/>
<br/>

![Wiring Example 2](https://github.com/user-attachments/assets/e30fadd9-19cd-47ec-baab-86f8a80410f6)

![7480a856c7839044d7a04292d352b709a2155c07_2_296x500](https://github.com/user-attachments/assets/5b3ccbb8-eb62-4743-8d05-f88a9b986743)

---

## 🧠 Einrichtung in Home Assistant

> Die Lösung basiert auf **ESPHome** und funktioniert nur mit Home Assistant.

### 1. ESPHome installieren

- In Home Assistant unter **Einstellungen → Add-ons → ESPHome** installieren

### 2. Neues Gerät erstellen

- Im ESPHome-Dashboard → "New Device"
- Deinen ESP32-Typ auswählen, z. B. `esp32-c3-devkitm-1` oder `nodemcu-32s`

### 3. Konfiguration einfügen

#### Option A: Einfache Konfiguration
[📄 Sample_conf.yaml](https://github.com/sorz2122/tclac/blob/master/Sample_conf.yaml)

#### Option B: Erweiterte Konfiguration
[📄 TCL-Conditioner.yaml](https://github.com/sorz2122/tclac/blob/master/TCL-Conditioner.yaml)

📝 **Wichtig:**  
- WLAN-Daten, Gerätename etc. anpassen  
- Kommentare im YAML helfen beim Einrichten

### 4. Auf ESP32 flashen

- USB-Kabel anschließen oder OTA (Over-the-Air) verwenden

---

## ✅ Kompatible Klimaanlagen

Diese Modelle wurden erfolgreich getestet:

- **TCL:** TAC-07CHSA / TAC-09CHSA / TAC-12CHSA / TAC-12CHDA
- **Daichi:** AIR20AVQ1, AIR25AVQS1R-1, DA35EVQ1-1
- **Axioma:** ASX09H1 / ASB09H1
- **Dantex:** RK-12SATI / RK-12SATIE  
- ...und ähnliche Modelle

⚠️ **Hinweis:**  
Auch wenn die Modellbezeichnung passt, kann es Unterschiede geben (kein USB-Anschluss, kein UART auf der Platine etc.).

---

## ☕ Unterstützung

https://buymeacoffee.com/sorz2122

<img src="https://github.com/user-attachments/assets/87d5d62f-ba5c-4a7e-a4b8-4cf1fd3018af" width="400"/>
<br/>

---

## 🔧 Erweiterte Konfiguration per Remote Package

Du kannst die Konfiguration modular laden:

```yaml
packages:
  remote_package:
    url: https://github.com/sorz2122/tclac.git
    ref: master
    files:
      - packages/core.yaml   # Hauptmodul
      # - packages/leds.yaml # Optional
    refresh: 30s
