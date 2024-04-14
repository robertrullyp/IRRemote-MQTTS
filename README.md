ESP Arduino Smart IR Remote via MQTT
Sketch Arduinonya cek di IRRemote-MQTTS/src/IRRemote-MQTTS.ino


Supported MCU :
1. ESP8266
2. ESP32

Komponen yang dibutuhkan :
1. LED IR Transmitter
2. LED IR Receiver
3. Resistor
4. Mosfet atau Transistor
5. Sensor Suhu & Kelembaban (AHTxx)

Dapat menggunakan modul IR Emitter:
![photo_2024-04-11_19-18-00](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/f3449755-5e8f-4832-9187-043411cd3fff)

atau ganti modul IR Emitter dengan rangkaian mosfet:
![screen-shot-2021-02-24-at-11 25 10-am](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/53a4a614-9ecc-4f50-a1db-024f87e330f5)


Fitur :
1. WebConfig
Memungkinkan untuk melakukan konfigurasi wifi dan parameter mqtt melalui web portal
2. DRD
Mendeteksi reset beberapakali untuk memasuki webconfig mode
3. Remote IR
Pengendalian untuk berbagai peralatan melalui Infra Merah dengan jangkauan kendali IoT 
4. Decode IR
Melakukan pembacaan kode IR dengan berbagai protokol untuk berbagai peralatan berbasis IR
Protokol yang didukung, dicek di :
https://github.com/crankyoldgit/IRremoteESP8266/blob/master/SupportedProtocols.md



Penggunaan :
1. Disesuaikan dulu atau dikembangkan dulu arduino codenya sesuai kebutuhan dan peralatan yang dipakai. Flash/Upload Firmware (pake kabel dulu buat upload pertama kali, selanjutnya untuk update program yang udah dikembangin bisa pake otp uploadnya)
2. Sambungkan wifi ke perangkat dengan menggunakan password yang sama dengan nama ssid (First time config)
3. Lakukan konfigurasi parameter wifi & mqtt
4. Save & Reboot/Reset
5. Kontrol menggunakan mqtt client atau integrasi dengan dashboard lainnya (ex: Home Assistant, dsb)
untuk bisa melihat struktur topic mqtt cek :
https://mqtt-explorer.com/

untuk merubah configurasi wifi & mqtt selanjutnya dapat menggunakan fitur DRD, caranya tekan reset 2 kali dalam 2-3 detik (disesuaikan), akan masuk mode webconfig

Kontrol menggunakan Android MQTT Dashboard Client:

![photo_2024-04-11_19-15-40](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/c1a319ea-93e0-438a-b82d-86c7dd958124)

Integrasi dengan Home Assistant:

![photo_2023-05-05_16-09-44](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/ba63651d-536f-444e-b8f3-36488e225426)


https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/75182d38-0b2f-40bb-9a3a-3af6e9f59962


https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/f4d919bb-fa94-40f4-b695-e09f36397f24  

