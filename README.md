# ArduinoESP Smart IR Remote MQTT

Memungkinkan untuk melakukan pengendalian dan otomatisasi untuk perangkat yang berbasis Infra Merah bahkan dapat diintegrasi dengan berbagai dashboard yang mendukung integrasi dengan MQTT.

Sketch Arduino cek di sini [ArduinoESP Smart IR Remote MQTT](https://github.com/robertrullyp/IRRemote-MQTTS/blob/main/src/IRRemote-MQTTS.ino)

## Supported MCU
1.Board berbasis ESP8266

2.Board Berbasis ESP32
## Program & Fitur
Pertama kali program berjalan akan melakukan pengecekan konfigurasi (wifi & mqtt) yang tersimpan, kalau belum ada konfigurasi yang valid maka masuk program webconfig, esp akan jadi access point dan nyediain captive portal buat ngisi parameter wifi & mqtt. Buat masuk ke mode ini setelah ada konfigurasi valid yang kesimpan juga bisa dengan cara tekan reset 2x dalam waktu 2 - 3 detik, cek aja keterangan di serial monitor atau liat indikator led nanti, waktu di mode webconfig lednya terus nyala.

note :

    Password untuk terhubung ke WiFi ESP saat dimode webconfig sama dengan nama SSID


Fungsi utama pastinya buat ngendaliin perangkat yang berbasis IR, untuk pengendalian perangak AC baru dibikin buat Sharp AH-A5UCYN dan AC pake protokol atau remot yang serupa.
Memungkinkan buat nambahin kapabilitas buat merek atau tipe remot lain, nambah coding & penyesuaian nanti coba dikembangin lagi, ini masih dalam proses...
Buat tau kapabilitasnya bisa sampai mana ikutin aja update dari library IRremote yang dipake atau cek disini [List Protokol](https://github.com/crankyoldgit/IRremoteESP8266/blob/master/SupportedProtocols.md) yang didukung sementara ini.

Alternatifnya sementara kalau mau pake buat AC bisa pakai mapping per code, artinya di decode dulu kodenya lalu salin terus nanti di dashboard assign aja ke masing-masing button untuk tiap fungsi dari code yang sudah di decode. Fitur ngirim codenya gw sebut IR Universal, akan ngirim berdasarkan code yg kita tulis. Caranya gini :

Coba Pake [MQTT Explorer](https://mqtt-explorer.com/) biar lebih gampang liat struktur topiknya
- Decode dulu biar dapet code, 
    
    kirim atau publish dulu data payload "1" di topic mqtt [usermqtt]/[UID][ChipID]/set/ir-receiver/state misalnya topic gw ada di robert/NodeMCU-v3-fd8575/set/ir-receiver/state ini buat set atau ngaktifin fungsi IR Receiver buat decode dan hasil decode dikirm balik ke mqtt. Lalu di remote original arahin ke led ir receiver dan tinggal pencet fungsi tombol yang akan diassign ke button di dashboard.
- Cek & Salin Code, 
    
    Berikutnya hasil decode fungsi remot tadi akan dikirim ke mqtt berupa hex code di topic mqtt [usermqtt]/[UID][ChipID]/state/ir-receiver/code misal di gw ada di topic robert/NodeMCU-v3-fd8575/state/ir-receiver/code dengan payload keterangan protocol, hex code, dan jumlah bit. 3 Parameter ini biasanya tiap remot beda-beda bahkan AC Sharp AH-A5UCYN pakenya protocol GREE. Jadi catat 3 Parameter ini buat tiap fungsi tombol remot.
- Assign Code untuk Button di Dashboard,
    
    Karna sebelumnya udah decode jadi punya informasi yang dibutuhkan, lalu tahap terakhir cuman perlu assign code, protocol dan jumlab bit tadi ke button di mqtt dashboard. Caranya buat button dan atur untuk payload berisi hex code misalnya "0x38096050022000D0" di topic mqtt [usermqtt]/[UID][ChipID]/set/universal/[protocol]/[jumlahbit] contohnya code tadi hasil decode AC protocol gree dengan jumlab bit 64 buat fungsinya nyalain set auto di mode,fan,swing dsbnya, gw set topicnya di robert/NodeMCU-v3-fd8575/set/universal/GREE/64
note : 
    
    UID itu disesuaikan aja di sketch arduino atau [UID][ChipID] itu bisa dilihat berdasarkan nama SSID waktu ESP di mode webconfig

Untuk struktur topic ACnya lebih lanjut cek pake [MQTT Explorer](https://mqtt-explorer.com/) aja yaa karna mungkin aja nanti akan ada update sketch kalau lagi gabut. ESP akan ngirim state AC & sensor terus menerus secara berkala, dan struktur topic kurang lebih gini (abaikan yang homeassistant) :

![MQTT Struct](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/fab3467e-4508-46a2-be18-f7ab97fc22d2)

## Library
Beberapa library arduino tambahan yang perlu di install : 

* [IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266) buat encode dan decode data IR
* [ESP_WiFiManager](https://github.com/khoih-prog/ESP_WiFiManager) buat konektivitas WiFi & WebConfig  
* [ESP_DoubleResetDetector](https://github.com/khoih-prog/ESP_DoubleResetDetector) buat jadi detektor reset waktu mau config ulang atau ngerubah parameter wifi & mqtt
* [PubSubClient](https://github.com/knolleary/pubsubclient) buat koneksi mqtt
* [AHTxx](https://github.com/enjoyneering/AHTxx) buat sensor suhu & kelembaban (disesuaikan aja)
* [LITTLEFS](https://github.com/lorol/LITTLEFS), [ArduinoJson](https://github.com/bblanchon/ArduinoJson), dsb. waktu compile dicek lagi aja kalau ada library yang kurang cari, download & install aja...
pubsubclient


## Module & Components Diagram
* [LED IR Transmitter](https://www.vishay.com/docs/81011/tsal6400.pdf) 
![tsal6400](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/64e7a0ab-72c4-494f-bddd-782917146ad8)
* [LED IR Receiver](https://www.sparkfun.com/datasheets/Sensors/Infrared/tsop382.pdf) 
![vs1838b](https://cdn.webshopapp.com/shops/144750/files/220123337/vs1838b-infrared-receiver-38khz.jpg)
* [Sensor Suhu & Kelembaban](https://server4.eca.ir/eshop/AHT10/Aosong_AHT10_en_draft_0c.pdf) 
![AHT10](https://ezmation.com/101-medium_default/aht10-temperature-and-humidity-sensor-i2c.jpg)
* Resistor
* Mosfet atau Transistor




Pin IR Emitter ESP32 di PIN GPIO 27

Pin IR Emitter ESP8266 di PIN GPIO 12

Pin IR Receiver ESP32 di PIN GPIO 16

Pin IR Receiver ESP32 di PIN GPIO 14

Wiring untuk rangkaian bisa dibikin gini :
![photo_2024-04-11_19-18-00](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/f3449755-5e8f-4832-9187-043411cd3fff)

Modul IR Emitter bisa diganti dengan rangkaian mosfet driver gini :
![mosfetdriver](https://electricfiredesign.com/wp-content/uploads/2021/02/screen-shot-2021-02-24-at-11.25.10-am.png?w=684)
## Demo

Integrasi dengan Home Assistant:

![photo_2023-05-05_16-09-44](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/ba63651d-536f-444e-b8f3-36488e225426)

https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/75182d38-0b2f-40bb-9a3a-3af6e9f59962


https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/f4d919bb-fa94-40f4-b695-e09f36397f24  

Kontrol menggunakan Android MQTT Dashboard Client:

![photo_2024-04-11_19-15-40](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/c1a319ea-93e0-438a-b82d-86c7dd958124)

## Authors

- [Robert Rully Pernando](https://github.com/robertrullyp)

