# ArduinoESP Smart IR Remote MQTT

Memungkinkan untuk melakukan pengendalian dan otomatisasi untuk perangkat yang berbasis Infra Merah bahkan dapat diintegrasi dengan berbagai dashboard yang mendukung integrasi dengan MQTT.

Sketch Arduino cek di sini [ArduinoESP Smart IR Remote MQTT](https://github.com/robertrullyp/IRRemote-MQTTS/blob/main/src/IRRemote-MQTTS.ino)

## New Updates
Untuk ESP32 jika koneksi menggunakan SSL/TLS, masukin certificate for formatnya setiap baris baru (new line) diganti dengan \n misalnya isi file certifikat gini :

    -----BEGIN CERTIFICATE-----
    MIIH0zCCBbugAwIBAgIIXsO3pkN/pOAwDQYJKoZIhvcNAQEFBQAwQjESMBAGA1UE
    CQYDVQQGEwJFUzAeFw0xMTA1MDUwOTM3MzdaFw0zMDEyMzEwOTM3MzdaMEIxEjAQ
    pPVWQxaZLPSkVrQ0uGE3ycJYgBugl6H8WY3pEfbRD0tVNEYqi4Y7
    -----END CERTIFICATE-----

Untuk masukin certificatenya di webconfig cp cukup tulis gini :

    -----BEGIN CERTIFICATE-----\nMIIH0zCCBbugAwIBAgIIXsO3pkN/pOAwDQYJKoZIhvcNAQEFBQAwQjESMBAGA1UE\nCQYDVQQGEwJFUzAeFw0xMTA1MDUwOTM3MzdaFw0zMDEyMzEwOTM3MzdaMEIxEjAQ\npPVWQxaZLPSkVrQ0uGE3ycJYgBugl6H8WY3pEfbRD0tVNEYqi4Y7\n-----END CERTIFICATE-----\n

Fitur OTA buat ESP8266 dibuang karna bermasalah sama memory, terkadang bikin perangkat restart dengan kode exception. Jadi sementara OTA Update ada buat ESP32 aja.
Update untuk set ac pake urutan data dengan format json lewat komuniasi serial, contoh formatnya gini :

    {"Protocol":"GREE","Model":"YB1FA","Power":1,"Mode":"fan","Temperature":25,"Celsius":1,"Fan":"max","SwingV":"highest","SwingH":"Auto","Quiet":false,"Turbo":false,"Eco":false,"Light":true,"Filter":true,"Clean":false,"Beep":true,"Sleep":-1,"Clock":-1}
dan akan ngirim balik keterangan parameter sesuai ketentuan bahkan jika gagal buat baca parameter yg dikirim akan tampil di serial monitor.

Untuk set melalui mqtt kirim topic ke [usermqtt]/[UID][ChipID]/set/ac misalnya: robert/NodeMCU-v3-fd8575/set/ac format data jsonnya misalnya gini :

    {
        "Protocol":"GREE",
        "Model":"YB1FA",
        "Power": true,
        "Mode": "cool",
        "Temperature": 21,
        "Celsius": true,
        "Fan": "max",
        "SwingV": "Auto",
        "SwingH": "Auto",
        "Quiet": false,
        "Turbo": false,
        "Eco": false,
        "Light": true,
        "Filter": true,
        "Clean": false,
        "Beep": true,
        "Sleep": -1,
        "Clock": -1
    }
Status(state) topicnya dengan format json yg sama ada di [usermqtt]/[UID][ChipID]/state/ac misalnya: robert/NodeMCU-v3-fd8575/state/ac

Perlu dipastiin juga MQTT_MAX_PACKET_SIZE di library PubSubClient sesuai karna default 256 byte (edit file PubSubClient.h) biar bisa jalan lancar buat kirim terima data dengan json format seperti diatas, berdasarkan hasil kalkulator online ukurannya sekitar 360 byte, biar aman edit PubSubClient.h cari MQTT_MAX_PACKET_SIZE rubah jadi gini aja :
    
    #define MQTT_MAX_PACKET_SIZE 512

untuk set melalui topic mqtt dengan masing-masing parameter punya topic sendiri-sendiri struktur topicnya gini :

![Capture](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/76ba07eb-329e-4e6e-8fc4-949b09ebbdef)


[usermqtt]/[UID][ChipID]/set/ac/[ModelRemoteAC]/[PROTOKOL]/[ParameterAC] misalnya robert/NodeMCU-v3-fd8575/set/ac/yb1fa/gree/power
cek di bagian demo buat lebih lanjutnya integrasi di home assistant..!

Struktur topic sama aja kalau mau kirim pake code universal, misalnya udah dicatat code buat nyalain TV Panasonic, kirim data mqtt ke topic [usermqtt]/[UID][ChipID]/set/universal/[PROTOKOL]/[nBit] payload datanya isi dengan codenya misal "0x40040100BCBD". Atau bisa dikirim dengan format json ke topic [usermqtt]/[UID][ChipID]/set/universal payload datanya contohnya:

    {
        "Protocol":"PANASONIC",
        "Code":"0x400401000405",
        "Bit":48,
        "Repeat":0
    }

Topic di [usermqtt]/[UID][ChipID]/state/ir-receiver/hexcode berisi code dari pembacaan ir receiver perangkat, kalau pake home assistan atau dashboard lain yang mendukung, ini juga bisa dimanfaatkan buat trigger automation misalnya dengan remot tv bisa buat nyalain lampu, dsb.. coba oprek aja di dashboard kalian hehe 

Untuk model ada apa aja yang didukung di [List Protokol](https://github.com/crankyoldgit/IRremoteESP8266/blob/master/SupportedProtocols.md). Untuk mode, fan, swing, dsb cek di enum librarynya atau sumber bacaanya ada [disini referensinya](https://crankyoldgit.github.io/IRremoteESP8266/doxygen/html/namespacestdAc.html#a8bb0dbf18fe69f639f4ac0b3ff133383)
Untuk nentuin prioritas server mqtt juga udah bisa dimasukin langsung di webconfig, bisa dipilih lebih prefer ke server lokal dulu nyoba konek atau ke server iot dulu. Buat opsi use ssl/tls masih dikembangin, sementara buat nentuin koneksi mau pake ssl atau non ssl lewat sketch arduino dulu.. kalau pake ssl uncomment aja, kalau mau pake koneksi non-ssl jadiin comment, code di sketchnya yang ini : 

    #define USE_SSL



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
Buat nambahin perangkat MQTT HVAC di Home Assistant pake device ini, configuration.yaml kurang lebih gini :
    
    mqtt: 
      - climate:
          name: SHARP AH-A5UCYN
          unique_id: nodemcu_mqtt_hvac
          modes:
            - "off"
            - "auto"
            - "cool"
            - "dry"
            - "fan_only"
        #    - "heat"
          swing_modes:
            - "Off"
            - "Auto"
            - "Highest"
            - "High"
            - "Middle"
            - "Low"
            - "Lowest"
          fan_modes:
            - "Auto"
            - "Min"
            - "Medium"
            - "Max"
          max_temp: 30
          min_temp: 16
        #  initial: 25
          power_command_topic: "robert/NodeMCU-v3-fd8575/set/universal/yb1fa/gree/power"
          power_command_template: "{{ value }}"
        #  power_state_topic: "usertest/NodeMCU-v3-fd8575/state/ac/power"
        #  power_state_template: "{{ value }}"
          payload_on: "1"
          payload_off: "0"
          mode_command_topic: "robert/NodeMCU-v3-fd8575/set/universal/yb1fa/gree/mode"
        #  mode_command_template: "{{ value if value=="off" else "on" }}"
          mode_state_topic: "robert/NodeMCU-v3-fd8575/state/ac/mode"
          mode_state_template: "{{ value }}"
          temperature_command_topic: "robert/NodeMCU-v3-fd8575/set/universal/yb1fa/gree/temperature"
          temperature_state_topic: "robert/NodeMCU-v3-fd8575/state/ac/temperature"
          temperature_state_template: "{{ value }}"
          fan_mode_command_topic: "robert/NodeMCU-v3-fd8575/set/universal/yb1fa/gree/fan"
          fan_mode_state_topic: "robert/NodeMCU-v3-fd8575/state/ac/fan"
          fan_mode_state_template: "{{ value }}"
          swing_mode_command_topic: "robert/NodeMCU-v3-fd8575/set/universal/yb1fa/gree/swingv"
          swing_mode_state_topic: "robert/NodeMCU-v3-fd8575/state/ac/swingv"
          precision: 1.0
          retain: true
          current_temperature_topic: "robert/NodeMCU-v3-fd8575/Sensors/Temperature"
          current_temperature_template: "{{ value }}"
          current_humidity_topic : "robert/NodeMCU-v3-fd8575/Sensors/Humidity"
          current_humidity_template : "{{ value }}"

Contoh diatas pake parameter dengan masing-masing topic berisi satu raw data untuk satu parameter, kalau mau pake format json tinggal sesuaikan aja topic nya sesuai struktur topic yang udah dijelasin sebelumnya di awal, lalu sesuaikan template buat kirim dan terima data.

![photo_2024-04-29_11-30-25](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/13e39945-4168-4955-b34a-bdf801088f3d)

![photo_2024-04-29_02-07-51](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/978de153-3a56-4625-8522-3db6fb481595)

https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/75182d38-0b2f-40bb-9a3a-3af6e9f59962


https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/f4d919bb-fa94-40f4-b695-e09f36397f24  


Kontrol menggunakan Android MQTT Dashboard Client:

![photo_2024-04-29_11-26-11](https://github.com/robertrullyp/IRRemote-MQTTS/assets/12167355/5c8b46ef-1d80-4432-aa61-305c7b2f0447)


## Authors

- [Robert Rully Pernando](https://github.com/robertrullyp)

